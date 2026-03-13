// mod display;
// mod gps;
// mod gps_serial;
// mod motor;
// mod logging;
// mod ntrip;
// mod usb_serial;
// mod yaw_control;
// mod rpm_control;
// // src/main.rs
// use clap::Parser;
// use crossterm::execute;
// use std::io::{self, Write, stdout};
// use std::path::PathBuf;
// use std::sync::mpsc::{self, TryRecvError};
// // use std::time::Duration;
// // Replace your current imports with these aliased ones:
// use yaw_control::PDController as YawController;
// use yaw_control::MotorCommands as YawCommands;

// use rpm_control::PDController as RpmController;
// use rpm_control::MotorCommands as RpmCommands;


// #[derive(Parser, Debug)]
// #[command(version, about, long_about = None)]
// struct Args {
//     /// NTRIP mountpoint (e.g., MOUNTPOINT)
//     #[arg(long)]
//     ntrip_mount: Option<String>, // E.g. "VMAX-LAND-1"
//     /// GPS serial port path (e.g., /dev/ttyUSB0)
//     #[arg(long, default_value = "/dev/ttyS0")]
//     gps_port: PathBuf,
//     /// Arduino serial port path (e.g., /dev/ttyACM0)
//     #[arg(long, default_value = "/dev/ttyACM0")]
//     arduino_port: PathBuf,
//     /// Log file path (default: logs/YYYY-MM-DD_HH-MM-SS.csv)
//     #[arg(long)]
//     log_file: Option<PathBuf>,
// }

// fn main() -> std::io::Result<()> {
//     let args = Args::parse();

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
    
//     // Prompt user for input
 
//     let mut gps_port = gps_serial::open_port(args.gps_port);
//     let mut arduino_port = usb_serial::open_port(args.arduino_port);

//     let gps_connected = gps_port.is_ok();
//     let arduino_connected = arduino_port.is_ok();

//     // This creates both variables at once from the tuple returned by the function
//     let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18).expect("Failed to initialize motor pins");

//     if !gps_connected && !arduino_connected {
//         eprintln!("No serial ports connected. Exiting.");
//         return Ok(());
//     }

//     let mut parser = gps::parser::build_parser();
//     let mut gga_fix_quality: Option<String> = None;
//     let mut stdout = stdout();
//     let mut display = display::Display::new();

//     // Initialize terminal
//     execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

//     // Channel for NTRIP data to write to serial
//     let (tx, rx) = mpsc::channel::<Vec<u8>>();

//     // Start NTRIP thread if configured
//     if let Some(mount) = args.ntrip_mount {
//         std::thread::spawn(move || {
//             ntrip::connect_rtk2go_ntrip(tx, &mount);
//         });
//         println!("Started NTRIP thread");
//     }

//     // Prompt user for proportional gain
//     println!("Enter proportional gain (Kp):");
//     let mut kp_input_yaw = String::new();
//     io::stdin().read_line(&mut kp_input_yaw).expect("Failed to read line");
//     let kp_input_yaw: f64 = kp_input_yaw.trim().parse().expect("Please enter a valid number");

//     // Prompt user for derivative gain
//     println!("Enter derivative gain (Kd):");
//     let mut kd_input_yaw = String::new();
//     io::stdin().read_line(&mut kd_input_yaw).expect("Failed to read line");
//     let kd_input_yaw: f64 = kd_input_yaw.trim().parse().expect("Please enter a valid number");

//     // Create PDController with user input
//     let mut yaw_control = YawController::new(kp_input_yaw, kd_input_yaw);

//     println!("Created PDController with Kp = {}, Kd = {}", kp_input_yaw, kd_input_yaw);

//     // ===============================================================================================
    
//     // Prompt user for proportional gain
//     println!("Enter proportional gain (Kp):");
//     let mut kp_input_rpm = String::new();
//     io::stdin().read_line(&mut kp_input_rpm).expect("Failed to read line");
//     let kp_input_rpm: f64 = kp_input_rpm.trim().parse().expect("Please enter a valid number");

//     // Prompt user for derivative gain
//     println!("Enter derivative gain (Kd):");
//     let mut kd_input_rpm = String::new();
//     io::stdin().read_line(&mut kd_input_rpm).expect("Failed to read line");
//     let kd_input_rpm: f64 = kd_input_rpm.trim().parse().expect("Please enter a valid number");

//     // Create PDController with user input
//     let mut rpm_control = RpmController::new(kp_input_rpm, kd_input_rpm);

//     println!("Created PDController with Kp = {}, Kd = {}", kp_input_rpm, kd_input_rpm);

    
//     // ===============================================================================================
    
//     // Adjust these gains (2.0, 0.5) once you see how the robot behaves
//     // let mut yaw_control = PDController::new(2.0, 0.5); 
//     let target_yaw = 0.0; // Straight ahead

//     // =============================================

//         // Prompt user for proportional gain
//     println!("Enter target RPM:");
//     let mut target_rpm = String::new();
//     io::stdin().read_line(&mut target_rpm).expect("Failed to read line");
//     let target_rpm: f64 = target_rpm.trim().parse().expect("Please enter a valid number");

//     println!("Target RPM{}", target_rpm);

//     //=============================================================================================
//     loop {
//         // Read from GPS serial port, if connected
//         if gps_connected {
//             let gps_serial_data = gps_port.as_mut().unwrap().read_sentences();
//             if gps_serial_data.is_err() {
//                 continue;
//             }
//             let sentences = gps_serial_data.unwrap();
//             for sentence in &sentences {
//                 let mut next_parser = parser.clone();
//                 gps::parser::parse_nmea_sentence(&mut next_parser, sentence);

//                 let mut next_gga_fix_quality = gga_fix_quality.clone();
//                 if sentence.contains("GGA") {
//                     // Manually parse GGA fix quality
//                     // $GPGGA,time,lat,NS,lon,EW,quality,num_sats,hdop,alt,M,sep,M,diff_age,diff_station*cs
//                     let parts: Vec<&str> = sentence.split(',').collect();
//                     if parts.len() > 6 {
//                         next_gga_fix_quality = Some(parts[6].to_string());
//                     }
//                 }

//                 // If the time has changed, it means we've started a new epoch.
//                 // We should log the *previous* epoch's fully accumulated data.
//                 if next_parser.fix_time != parser.fix_time {
//                     if parser.fix_time.is_some() {
//                         logger.log_nmea(parser.clone(), gga_fix_quality.clone());
//                         display.update_gps(&mut stdout, &parser, gga_fix_quality.clone())?;
//                     }
//                 }

//                 parser = next_parser;
//                 gga_fix_quality = next_gga_fix_quality;
//             }
            
//             // Write any pending NTRIP correction data to serial
//             loop {
//                 match rx.try_recv() {
//                     Ok(data) => {
//                         // println!("Writing {} bytes of NTRIP data to serial", data.len());
//                         // Log RTCM data
//                         logger.log_rtcm(&data);
//                         gps_port.as_mut().unwrap().write_all(&data)?;
//                         gps_port.as_mut().unwrap().flush()?;
//                     }
//                     Err(TryRecvError::Empty) => break,
//                     Err(e) => {
//                         eprintln!("Channel error: {:?}", e);
//                         break;
//                     }
//                 }
//             }
//         }

//         // Read from Arduino serial port, if connected
//         if arduino_connected {
//             let arduino_serial_data = arduino_port.as_mut().unwrap().read_data();
//             if arduino_serial_data.is_err() {
//                 // println!("Error reading from Arduino serial port");
//                 continue;
//             }
//             if let Some(sensor_data) = arduino_serial_data.unwrap() {

//                 if let Some(euler_x) = sensor_data.euler_x {
//                 let corrected_yaw = -(euler_x as f64);
//                 // Call controller to command right motor speed
//                 let commands_yaw: YawCommands = yaw_control.compute_motor_commands(
//                 corrected_yaw, 
//                 target_yaw, 
//                 1700, 
//                 );

//                 if let Some(rpm_left) = sensor_data.rpm_left {
//                 // Call controller with 1600 as the constant left speed
//                 let commands_rpm: RpmCommands = rpm_control.compute_motor_commands(
//                 rpm_left as f64, 
//                 target_rpm, 
//                 1300, 
//                 );

//                 // Command the hardware
//                 let _ = motor::update_pwm_l(&mut motor_pin_l, commands_rpm.left_pwm_us as i64);
//                 let _ = motor::update_pwm_r(&mut motor_pin_r, commands_yaw.right_pwm_us as i64);
//             }

//                 logger.log_sensor_data(&sensor_data);
//                 display.update_arduino(&mut stdout, &sensor_data)?;
//             }       
//         };
//     } 
// }
// }


// mod display;
// mod gps;
// mod gps_serial;
// mod motor;
// mod logging;
// mod ntrip;
// mod usb_serial;
// mod yaw_control;
// mod rpm_control;
// mod distance_tracker;

// use clap::Parser;
// use crossterm::execute;
// use std::io::{self, Write, stdout};
// use std::path::PathBuf;
// use std::sync::mpsc::{self, TryRecvError};
// use yaw_control::PDController as YawController;
// use yaw_control::MotorCommands as YawCommands;
// use rpm_control::PDController as RpmController;
// use rpm_control::MotorCommands as RpmCommands;
// use distance_tracker::DistanceTracker;

// #[derive(Parser, Debug)]
// #[command(version, about, long_about = None)]
// struct Args {
//     /// NTRIP mountpoint (e.g., MOUNTPOINT)
//     #[arg(long)]
//     ntrip_mount: Option<String>,
//     /// GPS serial port path (e.g., /dev/ttyUSB0)
//     #[arg(long, default_value = "/dev/ttyS0")]
//     gps_port: PathBuf,
//     /// Arduino serial port path (e.g., /dev/ttyACM0)
//     #[arg(long, default_value = "/dev/ttyACM0")]
//     arduino_port: PathBuf,
//     /// Log file path
//     #[arg(long)]
//     log_file: Option<PathBuf>,
// }

// fn main() -> std::io::Result<()> {
//     let args = Args::parse();

//     let log_file = match args.log_file {
//         Some(path) => path,
//         None => {
//             std::fs::create_dir_all("logs")?;
//             let now = chrono::Local::now();
//             PathBuf::from("logs").join(format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv")))
//         }
//     };

//     let logger = logging::Logger::new(log_file)?;
//     let mut gps_port = gps_serial::open_port(args.gps_port);
//     let mut arduino_port = usb_serial::open_port(args.arduino_port);

//     let gps_connected = gps_port.is_ok();
//     let arduino_connected = arduino_port.is_ok();

//     let (mut motor_pin_l, mut motor_pin_r) =
//         motor::get_motor_pins(13, 18).expect("Failed to initialize motor pins");

//     if !gps_connected && !arduino_connected {
//         eprintln!("No serial ports connected. Exiting.");
//         return Ok(());
//     }

//     let mut buf = String::new();

//     println!("Enter Yaw proportional gain (Kp):");
//     io::stdin().read_line(&mut buf)?;
//     let kp_yaw: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter Yaw derivative gain (Kd):");
//     io::stdin().read_line(&mut buf)?;
//     let kd_yaw: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter target yaw heading in degrees (0=North, 90=East):");
//     io::stdin().read_line(&mut buf)?;
//     let target_yaw: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter RPM proportional gain (Kp):");
//     io::stdin().read_line(&mut buf)?;
//     let kp_rpm: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter RPM derivative gain (Kd):");
//     io::stdin().read_line(&mut buf)?;
//     let kd_rpm: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter target left motor RPM:");
//     io::stdin().read_line(&mut buf)?;
//     let target_rpm: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter target distance in meters:");
//     io::stdin().read_line(&mut buf)?;
//     let target_dist: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter base throttle PWM (1500=stop, 2000=full forward):");
//     io::stdin().read_line(&mut buf)?;
//     let base_throttle: u64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     // Initialize control variables
//     let mut yaw_ctrl  = YawController::new(kp_yaw, kd_yaw);
//     let mut rpm_ctrl  = RpmController::new(kp_rpm, kd_rpm);
//     let mut dist_tracker = DistanceTracker::new();

//     // Initialize gps variables
//     let mut parser = gps::parser::build_parser();
//     let mut gga_fix_quality: Option<String> = None;
//     let mut stdout = stdout();
//     let mut display = display::Display::new();

//     execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

//     let (tx, rx) = mpsc::channel::<Vec<u8>>();
//     if let Some(mount) = args.ntrip_mount {
//         std::thread::spawn(move || {
//             ntrip::connect_rtk2go_ntrip(tx, &mount);
//         });
//         println!("Started NTRIP thread");
//     }

//     loop {
//         // ── GPS branch (~1 Hz) ────────────────────────────────────────────────
//         if gps_connected {
//             let sentences = match gps_port.as_mut().unwrap().read_sentences() {
//                 Ok(s) => s,
//                 Err(_) => continue,
//             };

//             for sentence in &sentences {
//                 let mut next_parser = parser.clone();
//                 gps::parser::parse_nmea_sentence(&mut next_parser, sentence);

//                 let mut next_gga_fix_quality = gga_fix_quality.clone();
//                 if sentence.contains("GGA") {
//                     let parts: Vec<&str> = sentence.split(',').collect();
//                     if parts.len() > 6 {
//                         next_gga_fix_quality = Some(parts[6].to_string());
//                     }
//                 }

//                 if next_parser.fix_time != parser.fix_time && parser.fix_time.is_some() {
//                     logger.log_nmea(parser.clone(), gga_fix_quality.clone());
//                     display.update_gps(&mut stdout, &parser, gga_fix_quality.clone())?;

//                     let fix_quality: u8 = gga_fix_quality
//                         .as_deref()
//                         .and_then(|q| q.parse().ok())
//                         .unwrap_or(0);
                    
//                     // CALL: Anchor the dead reckoning with fresh GPS coordinates
//                     if fix_quality >= 1 {
//                         if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
//                             dist_tracker.update_gps(lat, lon);
//                         }
//                     }
//                 }

//                 parser = next_parser;
//                 gga_fix_quality = next_gga_fix_quality;
//             }
//             loop {
//                 match rx.try_recv() {
//                     Ok(data) => {
//                         logger.log_rtcm(&data);
//                         gps_port.as_mut().unwrap().write_all(&data)?;
//                         gps_port.as_mut().unwrap().flush()?;
//                     }
//                     Err(TryRecvError::Empty) => break,
//                     Err(e) => {
//                         eprintln!("NTRIP channel error: {:?}", e);
//                         break;
//                     }
//                 }
//             }
//         } 

//         // ── Arduino branch (~10 Hz) ───────────────────────────────────────────
//         if arduino_connected {
//             let sensor_data = match arduino_port.as_mut().unwrap().read_data() {
//                 Ok(Some(d)) => d,
//                 Ok(None) => continue,
//                 Err(_) => continue,
//             };

//             logger.log_sensor_data(&sensor_data);
//             display.update_arduino(&mut stdout, &sensor_data)?;

//             // CALL: Update IMU/Odometer and check for Arrival
//             if let (Some(rpm_l), Some(rpm_r), Some(heading)) = (
//                 sensor_data.rpm_left,
//                 sensor_data.rpm_right,
//                 sensor_data.euler_x,
//             ) {
//                 // Feed the latest movement data
//                 dist_tracker.update_imu(distance_tracker::ImuSample {
//                     heading_deg: heading as f64,
//                     rpm_left: rpm_l as f64,
//                     rpm_right: rpm_r as f64,
//                 });

//                 // Check if we hit the 5m target
//                 let dist_out = dist_tracker.check(target_dist);

//                 if dist_out.arrived {
//                     println!(
//                         "\n[STOP] Target reached! Traveled {:.3} m. Cutting power.",
//                         dist_out.dist_traveled_m
//                     );
//                     let _ = motor::update_pwm_l(&mut motor_pin_l, 1500);
//                     let _ = motor::update_pwm_r(&mut motor_pin_r, 1500);
//                     rpm_ctrl.reset_trim();
//                     break; // End the benchmark run
//                 }
//             }

//             // Yaw controller — right motor only
//             if let Some(euler_x) = sensor_data.euler_x {
//                 let yaw_cmd: YawCommands = yaw_ctrl.compute_motor_commands(
//                     euler_x as f64,
//                     target_yaw,
//                     base_throttle,
//                 );
//                 let _ = motor::update_pwm_r(&mut motor_pin_r, yaw_cmd.right_pwm_us as i64);
//             }

//             // RPM controller — left motor only
//             if let Some(rpm_l) = sensor_data.rpm_left {
//                 let rpm_cmd: RpmCommands = rpm_ctrl.compute_motor_commands(
//                     rpm_l as f64,
//                     target_rpm,
//                     base_throttle,
//                 );
//                 let _ = motor::update_pwm_l(&mut motor_pin_l, rpm_cmd.left_pwm_us as i64);
//             }
//         }
//     }

//     Ok(())
// }

mod display;
mod gps;
mod gps_serial;
mod motor;
mod logging;
mod ntrip;
mod usb_serial;
mod yaw_control;
mod yaw_control_crab;

use clap::Parser;
use crossterm::execute;
use std::io::{self, Write, stdout};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::sync::mpsc::{self};

use yaw_control::PDController as YawController;
use yaw_control::MotorCommands as YawCommands;
use yaw_control_crab::PDControllerCrab as CrabController;

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
    let socket = std::net::UdpSocket::bind("127.0.0.1:0")?;

    let log_file = match args.log_file {
        Some(path) => path,
        None => {
            std::fs::create_dir_all("logs")?;
            let now = chrono::Local::now();
            let filename = format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv"));
            PathBuf::from("logs").join(filename)
        }
    };

    let logger = logging::Logger::new(log_file)?;
    let mut gps_port = gps_serial::open_port(args.gps_port);
    let mut arduino_port = usb_serial::open_port(args.arduino_port);

    let gps_connected = gps_port.is_ok();
    let arduino_connected = arduino_port.is_ok();

    if !gps_connected && !arduino_connected {
        eprintln!("No serial ports connected. Exiting.");
        return Ok(());
    }

    // --- DRIVE MODE SELECTION ---
    println!("Select Drive Mode: [S]crew (Straight) or [C]rab (Lateral):");
    let mut mode_buf = String::new();
    io::stdin().read_line(&mut mode_buf)?;
    let is_crab = mode_buf.trim().to_lowercase().starts_with('c');

    // Variables to be initialized based on mode
    let mut yaw_control: Option<YawController> = None;
    let mut crab_control: Option<CrabController> = None;
    let mut target_yaw: f64;
    let left_target_pwm: f64;

    if is_crab {
        println!("\n--- CRAB MODE INITIALIZATION ---");
        let mut buf = String::new();
        
        println!("Enter Crab Yaw Kp:");
        io::stdin().read_line(&mut buf)?;
        let kp: f64 = buf.trim().parse().expect("Invalid Kp");
        buf.clear();

        println!("Enter Crab Yaw Kd:");
        io::stdin().read_line(&mut buf)?;
        let kd: f64 = buf.trim().parse().expect("Invalid Kd");
        buf.clear();

        println!("Enter Target Heading (0.0 = Straight):");
        io::stdin().read_line(&mut buf)?;
        target_yaw = buf.trim().parse().expect("Invalid Heading");
        buf.clear();

        println!("Enter Base PWM for Crab (e.g., 1700):");
        io::stdin().read_line(&mut buf)?;
        left_target_pwm = buf.trim().parse().expect("Invalid PWM");

        crab_control = Some(CrabController::new(kp, kd));
        println!("Crab Controller initialized.");
    } else {
        println!("\n--- SCREW MODE INITIALIZATION ---");
        let mut buf = String::new();
        
        // Using your original default gains for Screw mode
        yaw_control = Some(YawController::new(2.0, 0.2));
        target_yaw = 0.0;

        println!("Enter Left motor pwm:");
        io::stdin().read_line(&mut buf)?;
        left_target_pwm = buf.trim().parse().expect("Invalid PWM");
        
        println!("Screw Controller initialized at Kp=2.0, Kd=0.2");
    }

    // Initialize GPS and display
    let mut parser = gps::parser::build_parser();
    let mut gga_fix_quality: Option<String> = None;
    let mut stdout = stdout();
    let mut display = display::Display::new();
    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();
    let ml_rx_socket = UdpSocket::bind("0.0.0.0:5006")?;
    ml_rx_socket.set_nonblocking(true)?;

    if let Some(mount) = args.ntrip_mount {
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx, &mount);
        });
        println!("Started NTRIP thread");
    }

    let mut ntrip_status: String = String::from("No connection");
    let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18).expect("Failed to initialize motor pins");

    loop {
        // --- GPS BRANCH ---
        if gps_connected {
            let gps_serial_data = gps_port.as_mut().unwrap().read_sentences();
            if let Ok(sentences) = gps_serial_data {
                for sentence in &sentences {
                    let mut next_parser = parser.clone();
                    gps::parser::parse_nmea_sentence(&mut next_parser, sentence);

                    let mut next_gga_fix_quality = gga_fix_quality.clone();
                    if sentence.contains("GGA") {
                        let parts: Vec<&str> = sentence.split(',').collect();
                        if parts.len() > 6 {
                            next_gga_fix_quality = Some(parts[6].to_string());
                        }
                    }

                    if next_parser.fix_time != parser.fix_time && parser.fix_time.is_some() {
                        logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                        display.update_gps(&mut stdout, &parser, gga_fix_quality.clone(), &ntrip_status)?;
                    }
                    parser = next_parser;
                    gga_fix_quality = next_gga_fix_quality;
                }
            }

            while let Ok(msg) = rx.try_recv() {
                ntrip_status = msg.to_string();
                if let ntrip::NTRIPMessage::Rtcm(data) = msg {
                    logger.log_rtcm(&data);
                    gps_port.as_mut().unwrap().write_all(&data)?;
                    gps_port.as_mut().unwrap().flush()?;
                }
            }
        }

        // --- ARDUINO BRANCH ---
        if arduino_connected {
            let arduino_serial_data = arduino_port.as_mut().unwrap().read_data();
            if let Ok(Some(sensor_data)) = arduino_serial_data {
                if let Some(euler_x) = sensor_data.euler_x {
                    let corrected_yaw = -(euler_x as f64);
                    
                    // Logic Branch for Controller Choice
                    let commands_yaw: YawCommands = if is_crab {
                        let ctrl = crab_control.as_mut().unwrap();
                        let cmd = ctrl.compute_crab_commands(corrected_yaw, target_yaw, left_target_pwm as u64);
                        YawCommands { 
                            right_pwm_us: cmd.right_pwm_us, 
                            left_pwm_us: left_target_pwm as u64 
                        }
                    } else {
                        let ctrl = yaw_control.as_mut().unwrap();
                        ctrl.compute_motor_commands(corrected_yaw, target_yaw, 1700, left_target_pwm as u64)
                    };

                    // Send Telemetry
                    let msg = format!(
                        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                        sensor_data.time_ms.unwrap_or(0),
                        sensor_data.voltage_left.unwrap_or(0.0),
                        sensor_data.current_left_ma.unwrap_or(0.0),
                        sensor_data.voltage_right.unwrap_or(0.0),
                        sensor_data.current_right_ma.unwrap_or(0.0),
                        sensor_data.motor_current_left.unwrap_or(0.0),
                        sensor_data.motor_current_right.unwrap_or(0.0),
                        sensor_data.euler_x.unwrap_or(0.0),
                        sensor_data.euler_y.unwrap_or(0.0),
                        sensor_data.euler_z.unwrap_or(0.0),
                        sensor_data.acc_lin_x.unwrap_or(0.0),
                        sensor_data.acc_lin_y.unwrap_or(0.0),
                        sensor_data.acc_lin_z.unwrap_or(0.0),
                        sensor_data.sonar_mm.unwrap_or(0.0),
                        sensor_data.tof_mm.unwrap_or(0.0),
                        sensor_data.rpm_left.unwrap_or(0.0),
                        sensor_data.rpm_right.unwrap_or(0.0),
                        sensor_data.rotations_left.unwrap_or(0.0),
                        sensor_data.rotations_right.unwrap_or(0.0),
                        corrected_yaw,
                        0.0, // placeholder for kp if needed
                        0.0  // placeholder for kd if needed
                    );
                    let _ = socket.send_to(msg.as_bytes(), "127.0.0.1:5005");

                    // Hardware Command
                    let _ = motor::update_pwm_l(&mut motor_pin_l, commands_yaw.left_pwm_us as i64);
                    let _ = motor::update_pwm_r(&mut motor_pin_r, commands_yaw.right_pwm_us as i64);
                }
                logger.log_sensor_data(&sensor_data);
                display.update_arduino(&mut stdout, &sensor_data)?;
            }
        }

        // --- ML GAIN SWITCHING BRANCH ---
        let mut buf = [0; 1024];
        if let Ok((len, _)) = ml_rx_socket.recv_from(&mut buf) {
            let msg = String::from_utf8_lossy(&buf[..len]);
            let parts: Vec<&str> = msg.split(',').collect();
            
            if parts.len() >= 4 {
                let terrain_type = parts[1];
                let confidence: f64 = parts[2].trim_matches('%').parse().unwrap_or(0.0);

                display.update_ml(&mut stdout, parts[0].to_string(), parts[1].to_string(), parts[2].to_string(), parts[3].to_string())?;

                // Only update gains if we are in SCREW mode (yaw_control exists)
                if let Some(ref mut ctrl) = yaw_control {
                    if confidence > 30.0 {
                        match terrain_type {
                            "SoftSnow" | "HardIce" | "DrySand" => ctrl.update_gains(15.0, 1.5),
                            "WetSand" => ctrl.update_gains(5.0, 0.5),
                            _ => ctrl.update_gains(15.0, 1.5),
                        }
                    } else {
                        ctrl.update_gains(15.0, 1.5);
                    }
                }
            }
        }
    }
}