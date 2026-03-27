// use clap::Parser;
// use crossterm::execute;
// use std::io::{self, Write, stdout};
// use std::net::UdpSocket;
// use std::path::PathBuf;
// use std::sync::mpsc;
// use std::time::Instant; // Added for EKF timing

// use maarco::{
//     display, 
//     gps, 
//     gps_serial, 
//     logging::Logger, 
//     motor, 
//     ntrip, 
//     usb_serial,
//     yaw_control::{PDController as YawController, MotorCommands as YawCommands},
//     // Ensure DistanceTracker and RoverState are exported from your EKF file
//     distance_tracker_EKF::{DistanceTracker, RoverState}, 
// };

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
//     let socket = std::net::UdpSocket::bind("0.0.0.0:0")?;

//     let log_file = match args.log_file {
//         Some(path) => path,
//         None => {
//             std::fs::create_dir_all("logs")?;
//             let now = chrono::Local::now();
//             let filename = format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv"));
//             PathBuf::from("logs").join(filename)
//         }
//     };

//     let logger = Logger::new(log_file)?;
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
//     let wp_rx_socket = UdpSocket::bind("0.0.0.0:5007")?;
//     wp_rx_socket.set_nonblocking(true)?;

//     if let Some(mount) = args.ntrip_mount {
//         let tx_clone = tx.clone();
//         std::thread::spawn(move || {
//             ntrip::connect_rtk2go_ntrip(tx_clone, &mount);
//         });
//     }

//     let mut ntrip_status: String = String::from("No connection");
//     let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18).expect("Failed motors");

//     let mut yaw_control = YawController::new(2.0, 0.2);
    
//     // NAVIGATION STATE
//     let mut waypoints: Vec<(f64, f64)> = Vec::new();
//     let mut current_wp_idx = 0;
//     let mut target_yaw = 0.0; 
//     let mut is_mission_running = false;
//     let mut heading_offset = 0.0;
//     let mut should_calibrate_heading = false;
//     let mut corrected_yaw = 0.0;

//     // EKF STATE
//     let mut last_prediction_time = Instant::now();
//     let mut dist_tracker = DistanceTracker::new();
//     let mut current_state = RoverState { x_m: 0.0, y_m: 0.0, velocity_ms: 0.0, yaw_rad: 0.0 };

//     println!("Enter Left motor pwm (base speed):");
//     let mut input = String::new();
//     io::stdin().read_line(&mut input)?;
//     let left_target_pwm: f64 = input.trim().parse().unwrap_or(1500.0);

//     loop {
//         // 1. LISTEN FOR WAYPOINTS/COMMANDS
//         let mut wp_buf = [0; 1024];
//         if let Ok((len, _)) = wp_rx_socket.recv_from(&mut wp_buf) {
//             let msg = String::from_utf8_lossy(&wp_buf[..len]);
//             match msg.as_ref() {
//                 "SET_ORIGIN" => {
//                     if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
//                         dist_tracker.set_origin(lat, lon);
//                         waypoints.clear(); 
//                         current_wp_idx = 0;
//                         should_calibrate_heading = true;
//                         println!("Origin Set Successfully.");
//                     }
//                 },
//                 "START_MISSION" => { is_mission_running = true; println!("Mission Start!"); },
//                 "STOP" => { is_mission_running = false; waypoints.clear(); println!("STOPPED"); },
//                 _ if msg.starts_with("WP,") => {
//                     let parts: Vec<&str> = msg.split(',').collect();
//                     if parts.len() == 3 {
//                         let x = parts[1].parse().unwrap_or(0.0);
//                         let y = parts[2].parse().unwrap_or(0.0);
//                         waypoints.push((x, y));
//                     }
//                 },
//                 _ => {}
//             }
//         }

//         // 2. ARDUINO / IMU (FAST LOOP - PREDICT)
//         if arduino_connected {
//             if let Ok(Some(sensor_data)) = arduino_port.as_mut().unwrap().read_data() {
//                 let dt = last_prediction_time.elapsed().as_secs_f64();
//                 last_prediction_time = Instant::now();

//                 if let Some(euler_x) = sensor_data.euler_x {
//                     if should_calibrate_heading {
//                         heading_offset = -(euler_x as f64) - 90.0;
//                         should_calibrate_heading = false;
//                     }
//                     corrected_yaw = -(euler_x as f64) - heading_offset;
//                 }

//                 // EKF PREDICTION
//                 dist_tracker.predict_imu(sensor_data.acc_lin_y.unwrap_or(0.0) as f64, corrected_yaw, dt);
                
//                 logger.log_sensor_data(&sensor_data);
//                 let _ = display.update_arduino(&mut stdout, &sensor_data);
//             }
//         }

//         // 3. GPS (SLOW LOOP - UPDATE)
//         if gps_connected {
//             if let Ok(sentences) = gps_port.as_mut().unwrap().read_sentences() {
//                 for sentence in sentences {
//                     gps::parser::parse_nmea_sentence(&mut parser, &sentence);
//                     if sentence.contains("GGA") {
//                         let parts: Vec<&str> = sentence.split(',').collect();
//                         if parts.len() > 6 { gga_fix_quality = Some(parts[6].to_string()); }
//                     }
//                 }
//             }

//             if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
//                 let gps_speed_ms = (parser.speed_over_ground.unwrap_or(0.0) as f64) * 0.5144;
                
//                 // EKF UPDATE (Truth)
//                 current_state = dist_tracker.update_gps(lat, lon, gps_speed_ms);

//                 // Send to Python GCS
//                 let telemetry = format!("POS,{:.2},{:.2},{:.1},{:.2},{:.2}", 
//                     current_state.x_m, current_state.y_m, corrected_yaw, 
//                     waypoints.get(current_wp_idx).map(|w| w.0).unwrap_or(current_state.x_m),
//                     waypoints.get(current_wp_idx).map(|w| w.1).unwrap_or(current_state.y_m));
//                 let _ = socket.send_to(telemetry.as_bytes(), "172.20.10.5:5008");
                
//                 let _ = display.update_gps(&mut stdout, &parser, gga_fix_quality.clone(), &ntrip_status);
//             }

//             // NTRIP handling
//             while let Ok(msg) = rx.try_recv() {
//                 ntrip_status = msg.to_string();
//                 if let ntrip::NTRIPMessage::Rtcm(data) = msg {
//                     let _ = gps_port.as_mut().unwrap().write_all(&data);
//                 }
//             }
//         }

//     // 4. NAVIGATION & MOTOR CONTROL
//     let (final_l, final_r) = if is_mission_running && current_wp_idx < waypoints.len() {
//         let (tx, ty) = waypoints[current_wp_idx];
        
//         // Use the EKF's filtered X and Y for distance calculation
//         let dx = tx - current_state.x_m;
//         let dy = ty - current_state.y_m;
//         let dist = (dx.powi(2) + dy.powi(2)).sqrt();

//         if dist < 0.25 {
//             current_wp_idx += 1;
//             println!("Waypoint {} reached! Moving to next...", current_wp_idx);
//             (1500, 1500) // Brief pause/straighten at waypoint
//         } else {
//             // Calculate the angle to the target coordinate
//             target_yaw = dy.atan2(dx).to_degrees();

//             // --- YOUR YAW CONTROLLER IN ACTION ---
//             // We pass the corrected_yaw (IMU + Offset) and the target_yaw (Coordinate math)
//             let commands = yaw_control.compute_motor_commands(
//                 corrected_yaw, 
//                 target_yaw, 
//                 1700,              // Max PWM limit
//                 left_target_pwm as u64 // Your base cruising speed
//             );

//             (commands.left_pwm_us as i64, commands.right_pwm_us as i64)
//         }
//     } else {
//         // Mission not active or finished
//         (1500, 1500)
//     };

//     // Send the actual PD-calculated pulses to the GPIO pins
//     let _ = motor::update_pwm_l(&mut motor_pin_l, final_l);
//     let _ = motor::update_pwm_r(&mut motor_pin_r, final_r);
//     }
// }

use clap::Parser;
use crossterm::execute;
use std::io::{self, Write, stdout};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Instant;

use maarco::{
    display, 
    gps, 
    gps_serial, 
    logging::Logger, 
    motor, 
    ntrip, 
    usb_serial,
    yaw_control::{PDController as YawController},
    distance_tracker_EKF::{DistanceTracker, RoverState}, 
};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(long)]
    ntrip_mount: Option<String>,
    #[arg(long, default_value = "/dev/ttyS0")]
    gps_port: PathBuf,
    #[arg(long, default_value = "/dev/ttyACM0")]
    arduino_port: PathBuf,
    #[arg(long)]
    log_file: Option<PathBuf>,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();
    // Socket for sending data TO Python
    let socket = std::net::UdpSocket::bind("0.0.0.0:0")?;

    let log_file = match args.log_file {
        Some(path) => path,
        None => {
            std::fs::create_dir_all("logs")?;
            let now = chrono::Local::now();
            let filename = format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv"));
            PathBuf::from("logs").join(filename)
        }
    };

    let logger = Logger::new(log_file)?;
    let mut gps_port = gps_serial::open_port(args.gps_port);
    let mut arduino_port = usb_serial::open_port(args.arduino_port);

    let gps_connected = gps_port.is_ok();
    let arduino_connected = arduino_port.is_ok();

    if !gps_connected && !arduino_connected {
        eprintln!("No serial ports connected. Exiting.");
        return Ok(());
    }

    let mut parser = gps::parser::build_parser();
    let mut gga_fix_quality: Option<String> = None;
    let mut stdout = stdout();
    let mut display = display::Display::new();

    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();
    // Socket for receiving commands FROM Python
    let wp_rx_socket = UdpSocket::bind("0.0.0.0:5007")?;
    wp_rx_socket.set_nonblocking(true)?;

    if let Some(mount) = args.ntrip_mount {
        let tx_clone = tx.clone();
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx_clone, &mount);
        });
    }

    let mut ntrip_status: String = String::from("No connection");
    let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18).expect("Failed motors");

    let mut yaw_control = YawController::new(2.0, 0.2);
    
    let mut waypoints: Vec<(f64, f64)> = Vec::new();
    let mut current_wp_idx = 0;
    let mut target_yaw = 0.0; 
    let mut is_mission_running = false;
    let mut heading_offset = 0.0;
    let mut should_calibrate_heading = false;
    let mut corrected_yaw = 0.0;

    let mut last_prediction_time = Instant::now();
    let mut dist_tracker = DistanceTracker::new();
    
    // Initialize state at 0,0
    let mut current_state = RoverState { x_m: 0.0, y_m: 0.0, velocity_ms: 0.0, yaw_rad: 0.0 };

    println!("Enter Left motor pwm (base speed):");
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let left_target_pwm: f64 = input.trim().parse().unwrap_or(1500.0);

    loop {
        // 1. LISTEN FOR COMMANDS
        let mut wp_buf = [0; 1024];
        if let Ok((len, _)) = wp_rx_socket.recv_from(&mut wp_buf) {
            let msg = String::from_utf8_lossy(&wp_buf[..len]);
            match msg.as_ref() {
                "SET_ORIGIN" => {
                    // Indoor bypass: If no GPS, just set 0,0 as origin
                    let lat = parser.latitude.unwrap_or(0.0);
                    let lon = parser.longitude.unwrap_or(0.0);
                    dist_tracker.set_origin(lat, lon);
                    waypoints.clear(); 
                    current_wp_idx = 0;
                    should_calibrate_heading = true;
                    println!("Origin Reset (0,0 if no GPS).");
                },
                "START_MISSION" => { is_mission_running = true; println!("Mission Start!"); },
                "STOP" => { is_mission_running = false; waypoints.clear(); println!("STOPPED"); },
                _ if msg.starts_with("WP,") => {
                    let parts: Vec<&str> = msg.split(',').collect();
                    if parts.len() == 3 {
                        let x = parts[1].parse().unwrap_or(0.0);
                        let y = parts[2].parse().unwrap_or(0.0);
                        waypoints.push((x, y));
                    }
                },
                _ => {}
            }
        }

        // 2. ARDUINO / IMU (FAST LOOP - PREDICT)
        if arduino_connected {
            if let Ok(Some(sensor_data)) = arduino_port.as_mut().unwrap().read_data() {
                let dt = last_prediction_time.elapsed().as_secs_f64();
                last_prediction_time = Instant::now();

                if let Some(euler_x) = sensor_data.euler_x {
                    if should_calibrate_heading {
                        heading_offset = -(euler_x as f64) - 90.0;
                        should_calibrate_heading = false;
                    }
                    corrected_yaw = -(euler_x as f64) - heading_offset;
                }

                // EKF PREDICTION (Dead Reckoning)
                dist_tracker.predict_imu(sensor_data.acc_lin_y.unwrap_or(0.0) as f64, corrected_yaw, dt);
                
                // CRITICAL: Update current_state from EKF even without GPS
                // Assuming your DistanceTracker struct allows public access to state
                // Or you might need to add: pub fn get_state(&self) -> RoverState to your EKF file
                current_state = RoverState {
                    x_m: dist_tracker.state[0],
                    y_m: dist_tracker.state[1],
                    velocity_ms: dist_tracker.state[2],
                    yaw_rad: dist_tracker.state[3],
                };

                // SEND TELEMETRY TO PYTHON (Now moved here so it sends @ ~10Hz)
                let tx_coord = waypoints.get(current_wp_idx).map(|w| w.0).unwrap_or(current_state.x_m);
                let ty_coord = waypoints.get(current_wp_idx).map(|w| w.1).unwrap_or(current_state.y_m);
                
                let telemetry = format!("POS,{:.2},{:.2},{:.1},{:.2},{:.2}", 
                    current_state.x_m, current_state.y_m, corrected_yaw, tx_coord, ty_coord);
                
                // Make sure this IP (172.20.10.5) is your Python machine!
                let _ = socket.send_to(telemetry.as_bytes(), "172.20.10.5:5008");

                logger.log_sensor_data(&sensor_data);
                let _ = display.update_arduino(&mut stdout, &sensor_data);
            }
        }

        // 3. GPS (SLOW LOOP - UPDATE)
        if gps_connected {
            if let Ok(sentences) = gps_port.as_mut().unwrap().read_sentences() {
                for sentence in sentences {
                    gps::parser::parse_nmea_sentence(&mut parser, &sentence);
                    if sentence.contains("GGA") {
                        let parts: Vec<&str> = sentence.split(',').collect();
                        if parts.len() > 6 { gga_fix_quality = Some(parts[6].to_string()); }
                    }
                }
            }
            // --- MOVE THE STRING DEFINITION HERE ---
            let fix_quality_str = match gga_fix_quality.as_deref() {
                Some("0") => "Invalid",
                Some("1") => "GPS Fix",
                Some("2") => "DGPS Fix",
                Some("3") => "PPS Fix",
                Some("4") => "Fixed RTK",
                Some("5") => "Float RTK",
                _ => "Unknown",
            };

            if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                let gps_speed_ms = (parser.speed_over_ground.unwrap_or(0.0) as f64) * 0.5144;
                // EKF UPDATE (Corrects the drift when outdoors)

                current_state = dist_tracker.update_gps(lat, lon, gps_speed_ms, fix_quality_str);
                // current_state = dist_tracker.update_gps(lat, lon, gps_speed_ms);
                let _ = display.update_gps(&mut stdout, &parser, gga_fix_quality.clone(), &ntrip_status);
            }

            while let Ok(msg) = rx.try_recv() {
                ntrip_status = msg.to_string();
                if let ntrip::NTRIPMessage::Rtcm(data) = msg {
                    let _ = gps_port.as_mut().unwrap().write_all(&data);
                }
            }
        }

        // 4. NAVIGATION & MOTOR CONTROL
        let (final_l, final_r) = if is_mission_running && current_wp_idx < waypoints.len() {
            let (tx, ty) = waypoints[current_wp_idx];
            let dx = tx - current_state.x_m;
            let dy = ty - current_state.y_m;
            let dist = (dx.powi(2) + dy.powi(2)).sqrt();

            if dist < 0.25 {
                current_wp_idx += 1;
                (1500, 1500)
            } else {
                target_yaw = dy.atan2(dx).to_degrees();
                let commands = yaw_control.compute_motor_commands(
                    corrected_yaw, 
                    target_yaw, 
                    1700,
                    left_target_pwm as u64 
                );
                (commands.left_pwm_us as i64, commands.right_pwm_us as i64)
            }
        } else {
            (1500, 1500)
        };

        let _ = motor::update_pwm_l(&mut motor_pin_l, final_l);
        let _ = motor::update_pwm_r(&mut motor_pin_r, final_r);
    }
}