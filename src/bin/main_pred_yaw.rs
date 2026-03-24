// ============================== Motor + RPM
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

//     // // Prompt user for proportional gain
//     // println!("Enter proportional gain (Kp):");
//     // let mut kp_input_yaw = String::new();
//     // io::stdin().read_line(&mut kp_input_yaw).expect("Failed to read line");
//     // let kp_input_yaw: f64 = kp_input_yaw.trim().parse().expect("Please enter a valid number");

//     // // Prompt user for derivative gain
//     // println!("Enter derivative gain (Kd):");
//     // let mut kd_input_yaw = String::new();
//     // io::stdin().read_line(&mut kd_input_yaw).expect("Failed to read line");
//     // let kd_input_yaw: f64 = kd_input_yaw.trim().parse().expect("Please enter a valid number");

//     // Create PDController with user input
//     let mut yaw_control = YawController::new(2.0, 0.2);

//     println!("Created PDController with Kp = {}, Kd = {}", 2.0, 0.2);

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

// ==================================== Yaw Control 

use maarco::{
    display_pred, 
    gps, 
    gps_serial, 
    // Notice we go one level deeper here:
    logging::Logger, 
    motor, 
    ntrip, 
    usb_serial,
    yaw_control::{PDController as YawController, MotorCommands as YawCommands},
};


// src/main.rs
use clap::Parser;
use crossterm::execute;
use std::io::{self, Write, stdout};
// use std::io::{Write, stdout};
use std::net::UdpSocket;
use std::path::PathBuf;
// use std::sync::mpsc::{self, TryRecvError};
use std::sync::mpsc::{self};

// use std::time::Duration;
// Replace your current imports with these aliased ones:
// use yaw_control::PDController as YawController;
// use yaw_control::MotorCommands as YawCommands;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// NTRIP mountpoint (e.g., MOUNTPOINT)
    #[arg(long)]
    ntrip_mount: Option<String>, // E.g. "VMAX-LAND-1"
    /// GPS serial port path (e.g., /dev/ttyUSB0)
    #[arg(long, default_value = "/dev/ttyS0")]
    gps_port: PathBuf,
    /// Arduino serial port path (e.g., /dev/ttyACM0)
    #[arg(long, default_value = "/dev/ttyACM0")]
    arduino_port: PathBuf,
    /// Log file path (default: logs/YYYY-MM-DD_HH-MM-SS.csv)
    #[arg(long)]
    log_file: Option<PathBuf>,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();
    let socket = std::net::UdpSocket::bind("127.0.0.1:0")?; //New code

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
    let mut display = display_pred::Display::new();

    // Initialize terminal
    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    // Channel for NTRIP data to write to serial
    let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();


    let ml_rx_socket = UdpSocket::bind("0.0.0.0:5006")?;
    ml_rx_socket.set_nonblocking(true)?;

    // Start NTRIP thread if configured
    if let Some(mount) = args.ntrip_mount {
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx, &mount);
        });
        println!("Started NTRIP thread");
    }

    // NTRIP status string. This gets updated when we receive messages from the NTRIP thread.
    let mut ntrip_status: String = String::from("No connection");

    let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18).expect("Failed to initialize motor pins");

    // Create PDController with user input
    let mut yaw_control = YawController::new(2.0, 0.2);
    println!("Created PDController with Kp = {}, Kd = {}", 2.0, 0.2);

    // Adjust these gains (2.0, 0.5) once you see how the robot behaves
    // let mut yaw_control = PDController::new(2.0, 0.5); 
    let target_yaw = 0.0; // Straight ahead

    // Prompt user for proportional gain
    println!("Enter Left motor pwm:");
    let mut left_target_pwm = String::new();
    io::stdin().read_line(&mut left_target_pwm).expect("Failed to read line");
    let left_target_pwm: f64 = left_target_pwm.trim().parse().expect("Please enter a valid number");

    println!("Left motor target PWM: {}", left_target_pwm);

    loop {
        // Read from GPS serial port, if connected
        if gps_connected {
            let gps_serial_data = gps_port.as_mut().unwrap().read_sentences();
            if gps_serial_data.is_err() {
                continue;
            }
            let sentences = gps_serial_data.unwrap();
            for sentence in &sentences {
                let mut next_parser = parser.clone();
                gps::parser::parse_nmea_sentence(&mut next_parser, sentence);

                let mut next_gga_fix_quality = gga_fix_quality.clone();
                if sentence.contains("GGA") {
                    // Manually parse GGA fix quality
                    // $GPGGA,time,lat,NS,lon,EW,quality,num_sats,hdop,alt,M,sep,M,diff_age,diff_station*cs
                    let parts: Vec<&str> = sentence.split(',').collect();
                    if parts.len() > 6 {
                        next_gga_fix_quality = Some(parts[6].to_string());
                    }
                }

                // If the time has changed, it means we've started a new epoch.
                // We should log the *previous* epoch's fully accumulated data.
                if next_parser.fix_time != parser.fix_time
                    && parser.fix_time.is_some() {
                        logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                        display.update_gps(
                            &mut stdout,
                            &parser,
                            gga_fix_quality.clone(),
                            &ntrip_status,
                        )?;
                    }

                parser = next_parser;
                gga_fix_quality = next_gga_fix_quality;
            }

            // Write any pending NTRIP correction data to serial
            while let Ok(msg) = rx.try_recv() {
                ntrip_status = msg.to_string();
                if let ntrip::NTRIPMessage::Rtcm(data) = msg {
                    logger.log_rtcm(&data);
                    gps_port.as_mut().unwrap().write_all(&data)?;
                    gps_port.as_mut().unwrap().flush()?;
                }
            }
        }

        // Read from Arduino serial port, if connected
        if arduino_connected {
            let arduino_serial_data = arduino_port.as_mut().unwrap().read_data();
            if arduino_serial_data.is_err() {
                // println!("Error reading from Arduino serial port");
                continue;
            }
            if let Some(sensor_data) = arduino_serial_data.unwrap() {
                if let Some(euler_x) = sensor_data.euler_x {
                let corrected_yaw = -(euler_x as f64);
                // Call controller to command right motor speed
                let current_kp = yaw_control.kp; // or yaw_control.get_kp()
                let current_kd = yaw_control.kd; // or yaw_control.get_kd()

                let commands_yaw: YawCommands = yaw_control.compute_motor_commands(
                corrected_yaw, 
                target_yaw, 
                1700,
                left_target_pwm as u64 
                );

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
                    current_kp,
                    current_kd
                );
                let _ = socket.send_to(msg.as_bytes(), "127.0.0.1:5005");          

                // Command the hardware
                let _ = motor::update_pwm_l(&mut motor_pin_l, commands_yaw.left_pwm_us as i64);
                let _ = motor::update_pwm_r(&mut motor_pin_r, commands_yaw.right_pwm_us as i64);
            }
                // println!("Received sensor data: {:?}", sensor_data);
                logger.log_sensor_data(&sensor_data);
                display.update_arduino(&mut stdout, &sensor_data)?;
            }
        };

        let mut buf = [0; 1024];
        if let Ok((len, _)) = ml_rx_socket.recv_from(&mut buf) {
            let msg = String::from_utf8_lossy(&buf[..len]);
            let parts: Vec<&str> = msg.split(',').collect();
            
            if parts.len() >= 4 {

                let terrain_type = parts[1];
                // Parse confidence: remove '%' if present and parse to f64
                let confidence: f64 = parts[2].trim_matches('%').parse().unwrap_or(0.0);

                display.update_ml(
                    &mut stdout, 
                    parts[0].to_string(), // Time
                    parts[1].to_string(), // Terrain
                    parts[2].to_string(),  // Confidence
                    parts[3].to_string(),  // Yaw
                )?; 

                // Dynamic Gain Switching Logic
                if confidence > 30.0 {
                    match terrain_type {
                        "SoftSnow" => {
                            // Example: Higher P for loose surfaces
                            yaw_control.update_gains(15.0, 1.5);
                            // println!("Soft Snow PD Controller updated");
                        },
                        "HardIce" => {
                            yaw_control.update_gains(15.0, 1.5);
                            // println!("Hard Ice PD Controller updated");
                        },

                        "WetSand" => {
                            yaw_control.update_gains(5.0, 0.5);
                            // println!("Wet Sand PD Controller updated");
                        },

                        "DrySand" => {
                            // yaw_control.update_gains(3.2, 0.2);
                            // yaw_control.update_gains(3.2, 0.4);
                            // yaw_control.update_gains(3.6, 0.6);
                            yaw_control.update_gains(15.0, 1.5);

                            
                            // println!("Dry Sand PD Controller updated");
                        },
                        
                        _ => {
                            // This handles any terrain type not explicitly listed above
                            // Use your "safe" or "average" default gains here
                            yaw_control.update_gains(15.0, 1.5);
                            // println!("Unknown terrain type - using default PD gains");
                            }                      
                    }
                }

                else {
                    // Low confidence - use conservative gains
                    yaw_control.update_gains(15.0, 1.5);
                    // yaw_control.update_gains(15.0, 2.5);
                    // println!("Low confidence in terrain prediction - using default PD gains");
                }
            }
        }
    }
}