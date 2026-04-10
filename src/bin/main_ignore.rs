// mod display;
// mod gps;
// mod gps_serial;
// mod motor;
// mod logging;
// mod ntrip;
// mod usb_serial;
// mod yaw_control;
// mod distance_tracker; // Ensure you have the new distance_tracker.rs file

// use clap::Parser;
// use crossterm::execute;
// use std::io::{self, Write, stdout};
// use std::net::UdpSocket;
// use std::path::PathBuf;
// use std::sync::mpsc::{self};

// use yaw_control::PDController as YawController;
// use yaw_control::MotorCommands as YawCommands;
// use distance_tracker::DistanceTracker;

// #[derive(Parser, Debug)]
// #[command(version, about, long_about = None)]
// struct Args {
//     #[arg(long)]
//     ntrip_mount: Option<String>,
//     #[arg(long, default_value = "/dev/ttyS0")]
//     gps_port: PathBuf,
//     #[arg(long, default_value = "/dev/ttyACM0")]
//     arduino_port: PathBuf,
//     #[arg(long)]
//     log_file: Option<PathBuf>,
// }

// fn main() -> std::io::Result<()> {
//     let args = Args::parse();
    
//     // Sockets
//     let socket = std::net::UdpSocket::bind("127.0.0.1:0")?; 
   
//     let log_file = match args.log_file {
//         Some(path) => path,
//         None => {
//             std::fs::create_dir_all("logs")?;
//             let now = chrono::Local::now();
//             let filename = format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv"));
//             PathBuf::from("logs").join(filename)
//         }
//     };

//     let logger = logging::Logger::new(log_file)?;
//     let mut gps_port = gps_serial::open_port(args.gps_port);
//     let mut arduino_port = usb_serial::open_port(args.arduino_port);

//     let gps_connected = gps_port.is_ok();
//     let arduino_connected = arduino_port.is_ok();

//     if !gps_connected && !arduino_connected {
//         eprintln!("No serial ports connected. Exiting.");
//         return Ok(());
//     }

//     let mut parser = gps::parser::build_parser();
//     let mut gga_fix_quality: Option<String> = None;
//     let mut stdout = stdout();
//     let mut display = display::Display::new();

//     execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

//     let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();

//     if let Some(mount) = args.ntrip_mount {
//         std::thread::spawn(move || {
//             ntrip::connect_rtk2go_ntrip(tx, &mount);
//         });
//     }

//     let mut ntrip_status: String = String::from("No connection");
//     let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18).expect("Failed to initialize pins");

//     // Trajectory & Mission State
//     let mut yaw_control = YawController::new(15.0, 1.5);
//     let mut dist_tracker = DistanceTracker::new();
//     let mut waypoints: Vec<(f64, f64)> = Vec::new();
//     let mut current_wp_idx = 0;
//     let mut target_yaw = 0.0; 
//     let mut is_mission_running = false;

//     println!("Enter Left motor base pwm (e.g. 1500):");
//     let mut left_target_pwm_str = String::new();
//     io::stdin().read_line(&mut left_target_pwm_str).expect("Failed to read line");
//     let left_target_pwm: f64 = left_target_pwm_str.trim().parse().unwrap_or(1500.0);

//     loop {
//         // --- 1. LISTEN FOR WAYPOINTS FROM PYTHON ---
//         let mut wp_buf = [0; 1024];
//         if let Ok((len, _)) = wp_rx_socket.recv_from(&mut wp_buf) {
//             let msg = String::from_utf8_lossy(&wp_buf[..len]);
//             match msg.as_ref() {
//                 "SET_ORIGIN" => {
//                     if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
//                         dist_tracker.set_origin(lat, lon);
//                     }
//                 },
//                 "START_MISSION" => {
//                     is_mission_running = true;
//                     println!("Mission Started!");
//                 },
//                 _ if msg.starts_with("WP,") => {
//                     let parts: Vec<&str> = msg.split(',').collect();
//                     if parts.len() == 3 {
//                         let x: f64 = parts[1].parse().unwrap_or(0.0);
//                         let y: f64 = parts[2].parse().unwrap_or(0.0);
//                         waypoints.push((x, y));
//                     }
//                 },
//                 _ => {}
//             }
//         }

//         // --- 2. GPS DATA PROCESSING ---
//         if gps_connected {
//             if let Ok(sentences) = gps_port.as_mut().unwrap().read_sentences() {
//                 for sentence in &sentences {
//                     gps::parser::parse_nmea_sentence(&mut parser, sentence);
//                     if sentence.contains("GGA") {
//                         let parts: Vec<&str> = sentence.split(',').collect();
//                         if parts.len() > 6 { gga_fix_quality = Some(parts[6].to_string()); }
//                     }
//                 }
//             }

//             // Update XY Position and calculate target yaw
//             if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                
//                 let speed_kmh = (parser.speed_over_ground.unwrap_or(0.0) as f64) * 1.852;
//                 if let Some(state) = dist_tracker.update_gps(lat, lon, speed_kmh) {    
//                     // Logic to find next heading
//                     if is_mission_running && current_wp_idx < waypoints.len() {
//                         let (tx, ty) = waypoints[current_wp_idx];
//                         let dx = tx - state.x_m;
//                         let dy = ty - state.y_m;
                        
//                         let dist = (dx.powi(2) + dy.powi(2)).sqrt();
//                         if dist < 0.6 {
//                             current_wp_idx += 1;
//                             println!("WP {} reached!", current_wp_idx);
//                         } else {
//                             // Calculate angle to target
//                             target_yaw = dy.atan2(dx).to_degrees();
//                         }
//                     }
//                 }
//             }
            
//             while let Ok(msg) = rx.try_recv() {
//                 ntrip_status = msg.to_string();
//                 if let ntrip::NTRIPMessage::Rtcm(data) = msg {
//                     let _ = gps_port.as_mut().unwrap().write_all(&data);
//                 }
//             }
//         }

//         // --- 3. ARDUINO & CONTROL ---
//         if arduino_connected {
//             if let Ok(Some(sensor_data)) = arduino_port.as_mut().unwrap().read_data() {
//                 if let Some(euler_x) = sensor_data.euler_x {
//                     let corrected_yaw = -(euler_x as f64);
                    
//                     // NEW: Check if we are actually supposed to be moving
//                     let (final_l_pwm, final_r_pwm) = if is_mission_running && current_wp_idx < waypoints.len() {
//                         // ACTIVE NAVIGATION
//                         let commands = yaw_control.compute_motor_commands(
//                             corrected_yaw, 
//                             target_yaw, 
//                             1700,
//                             left_target_pwm as u64 
//                         );
//                         (commands.left_pwm_us as i64, commands.right_pwm_us as i64)
//                     } else {
//                         // MISSION COMPLETE or NOT STARTED: Stop Motors (1500 is neutral)
//                         if is_mission_running && current_wp_idx >= waypoints.len() && waypoints.len() > 0 {
//                             // This only prints once when the mission finishes
//                             println!("--- MISSION COMPLETE: STOPPING MOTORS ---");
//                             is_mission_running = false; 
//                         }
//                         (1500, 1500)
//                     };

//                     // Command the hardware with the decided PWM values
//                     let _ = motor::update_pwm_l(&mut motor_pin_l, final_l_pwm);
//                     let _ = motor::update_pwm_r(&mut motor_pin_r, final_r_pwm);
//                 }
//                 display.update_arduino(&mut stdout, &sensor_data)?;
//             }
//         }       
//     }
// }
