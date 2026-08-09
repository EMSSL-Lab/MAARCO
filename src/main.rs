mod display;
mod gps;
mod gps_serial;
mod motor;
mod logging;
mod ntrip;
mod usb_serial;
mod yaw_control;
mod rpm_control;
mod distance_tracker;

use clap::Parser;
use crossterm::execute;
use std::io::{Write, stdout};
use std::path::PathBuf;
use std::sync::mpsc::{self, TryRecvError};
use yaw_control::PDController as YawController;
use rpm_control::PDController as RpmController;
use distance_tracker::DistanceTracker;
use std::net::UdpSocket; 
use std::io::stdin;
use std::time::{Instant, Duration};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// NTRIP mountpoint (e.g., MOUNTPOINT) Connect to Cary Base Station by Default
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
/// Prompts user for a new target distance and yaw heading after arrival, for retasking.
/// Returns (new_distance, new_yaw).
// fn prompt_retask() -> (f64, f64, f64) {
//     let mut buf = String::new();

//     println!("\n ------------------------------------------------------------------------------------------");
//     println!("Enter new target distance in meters:");
//     io::stdin().read_line(&mut buf).expect("Failed to read line");
//     let new_dist: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter new target yaw heading in degrees:");
//     io::stdin().read_line(&mut buf).expect("Failed to read line");
//     let new_yaw: f64 = buf.trim().parse().expect("Invalid number");
//     buf.clear();

//     println!("Enter new target left motor RPM:");
//     io::stdin().read_line(&mut buf).expect("Failed to read line");
//     let new_rpm: f64 = buf.trim().parse().expect("Invalid number");

//     println!("New leg: {:.1} m | New heading: {:.1} degrees | New target rpm: {:.1}  Starting...", new_dist, new_yaw, new_rpm);
//     println!(" ------------------------------------------------------------------------------------------\n");
    
//     (new_dist, new_yaw, new_rpm)

// } 


fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let log_file = match args.log_file {
        Some(path) => path,
        None => {
            std::fs::create_dir_all("logs")?;
            let now = chrono::Local::now();
            PathBuf::from("logs").join(format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv")))
        }
    };

    let logger = logging::Logger::new(log_file)?;
    let mut gps_port = gps_serial::open_port(args.gps_port);
    let mut arduino_port = usb_serial::open_port(args.arduino_port);

    let gps_connected = gps_port.is_ok();
    let arduino_connected = arduino_port.is_ok();

    let (mut motor_pin_l, mut motor_pin_r) =
        motor::get_motor_pins(13, 18).expect("Failed to initialize motor pins");

    if !gps_connected && !arduino_connected {
        eprintln!("No serial ports connected. Exiting.");
        return Ok(());
    }

    // ── Startup prompts ───────────────────────────────────────────────────────
    //
    // Control hierarchy:
    //
    // base_throttle (fixed, set once at startup)
    //        │                        
    //        ▼                        
    // rpm_ctrl (LEFT)          yaw_ctrl (RIGHT)
    // holds target_rpm        varies right motor for heading
    // trims for terrain        
    //
    // dist_ctrl: monitors position only.
    //            cuts both motors to 1500 on arrival. nothing else.
    //
    // ─────────────────────────────────────────────────────────────────────────


    // ------------------------------------------------ User input -------------------------
    // Prompt user for control parameters at startup. No dynamic reconfiguration.
    // let mut buf = String::new();

    // println!("Enter Yaw proportional gain (Kp):");
    // io::stdin().read_line(&mut buf)?;
    // let kp_yaw: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // println!("Enter Yaw derivative gain (Kd):");
    // io::stdin().read_line(&mut buf)?;
    // let kd_yaw: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // println!("Enter target yaw heading in degrees (0=North, 90=East):");
    // io::stdin().read_line(&mut buf)?;
    // let mut target_yaw: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // println!("Enter RPM proportional gain (Kp):");
    // io::stdin().read_line(&mut buf)?;
    // let kp_rpm: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // println!("Enter RPM derivative gain (Kd):");
    // io::stdin().read_line(&mut buf)?;
    // let kd_rpm: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // println!("Enter target left motor RPM:");
    // stdin().read_line(&mut buf)?;
    // let target_rpm: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // println!("Enter target distance in meters:");
    // io::stdin().read_line(&mut buf)?;
    // let mut target_dist: f64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();

    // base_throttle: fixed for the whole run.
    // Not touched by any controller — only dist_ctrl can override it to 1500
    // on arrival
    // println!("Enter base throttle PWM (1500=stop, 2000=full forward):");
    // io::stdin().read_line(&mut buf)?;
    // let mut base_throttle: u64 = buf.trim().parse().expect("Invalid number");
    // buf.clear();
    // ─────────────────────────────────────────────────────────────────────────

    // Set controller gains here
    let kp_yaw: f64 = 5.0;
    let kd_yaw: f64 = 1.0;
    let kp_rpm: f64 = 0.8;
    let kd_rpm: f64 = 0.06;
    
    
    // Initialize control variables
    let mut yaw_ctrl  = YawController::new(kp_yaw, kd_yaw);
    let mut rpm_ctrl  = RpmController::new(kp_rpm, kd_rpm);
    let mut dist_tracker = DistanceTracker::new();
    let mut base_throttle: u64 = 1600; 
    
    // Declare targets for yaw, rpm, and distance
    let mut target_yaw: f64 = 0.0;
    let mut target_dist: f64 = 0.0;
    let mut target_rpm: f64 = 0.0;

    let mut is_active_leg: bool = false;
    // ADD SOCKET SETUP HERE
    let socket = UdpSocket::bind("0.0.0.0:5007").expect("Couldn't bind to UDP Socket");
    socket.set_nonblocking(true).expect("Couldn't set non-blocking");
    let mut udp_buf = [0u8; 1024];

    // FAILSAFE Timer for connection distruption between UI and Raspberry Pi
    let mut last_gcs_time = Instant::now();
    let failsafe_timeout = Duration::from_secs(2);
    // Initialize gps variables
    let mut parser = gps::parser::build_parser();
    let mut gga_fix_quality: Option<String> = None;
    let mut stdout = stdout();
    let mut display = display::Display::new();


    // Initialize terminal
    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    // Channel for NTRIP data to write to serial
    let ntrip_rx = if let Some(mount) = args.ntrip_mount {
        let (tx, rx) = mpsc::channel::<Vec<u8>>();
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx, &mount);
        });
        println!("Started NTRIP thread");
        Some(rx)
    } else {
        println!("NTRIP Disabled (no --ntrip-mount argument)");
        None
    };

    // =========================================================================
    loop {

        // RECIEVE DATA FROM UI
        // ***NEW Ground Control Station PARSING LOGIC
        match socket.recv_from(&mut udp_buf) {
            Ok((amt, _src)) => {
                let msg = String::from_utf8_lossy(&udp_buf[..amt]);
                let parts: Vec<&str> = msg.split(',').collect();
                last_gcs_time = Instant::now(); // Reset failsafe timer on any valid message
                // RUST recieves UDP data here!!!
                if parts[0] == "NAV" && parts.len() == 3 {
                    if let (Ok(d), Ok(y)) = (parts[1].parse::<f64>(), parts[2].parse::<f64>()) {
                        target_dist = d;
                        target_yaw = y;
                        is_active_leg = true;
                        dist_tracker.reset_leg();
                        yaw_ctrl.reset();
                        rpm_ctrl.reset();
                        rpm_ctrl.reset_base_throttle(); 
                        println!("NAV: {:.3}m @ {:.3}°", target_dist, target_yaw);
                    } else {
                        eprintln!("Invalid NAV format: {}", msg);
                    }
                } else if parts[0] == "RPM" && parts.len() == 2 {
                    if let Ok(val) = parts[1].parse::<f64>() {
                    target_rpm = val;
                    rpm_ctrl.reset();              // ← clears last_error and dt timer
                    // rpm_ctrl.reset_base_throttle(); // ← unwinds the integrator
                    println!("Target RPM updated via GCS: {:.1}", target_rpm);
                    }

                } else if parts[0] == "STOP" {
                    is_active_leg = false;
                    let _ = motor::update_pwm_l(&mut motor_pin_l, 1500); 
                    let _ = motor::update_pwm_r(&mut motor_pin_r, 1500);
                    println!("Stop command received.");
                } else if parts[0] == "SET_ORIGIN" {
                    dist_tracker.reset_for_new_target();
                    println!("Origin reset.");
                }
                 else if parts [0] == "ADJUST_DISTANCE_TRACKER" && parts.len() == 3 {
                    if let (Ok(new_yaw), Ok(new_dist)) = (parts[1].trim().parse::<f64>(), parts[2].trim().parse::<f64>()) {
                        target_yaw = new_yaw; // Update yaw setpoint to counter cross tracking error
                        target_dist = new_dist; // Update distance setpoint
                        // dist_tracker.reset_leg();
                    }
                }
            

            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {} 
            Err(e) => eprintln!("UDP Error: {}", e),
        }

        // Failsafe check
        if is_active_leg && last_gcs_time.elapsed() > failsafe_timeout {
            println!(" GCS connection lost | Motors set to neutral");
            is_active_leg = false; // cancels current waypoint

            let _ = motor::update_pwm_l(&mut motor_pin_l, 1500);
            let _ = motor::update_pwm_r(&mut motor_pin_r, 1500);    
        }





        // ── GPS branch (~1 Hz) ────────────────────────────────────────────────
        if gps_connected {
            let sentences = match gps_port.as_mut().unwrap().read_sentences() {
                Ok(s) => s,
                Err(_) => continue,
            };

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

                if next_parser.fix_time != parser.fix_time && parser.fix_time.is_some() {
                    logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                    display.update_gps(&mut stdout, &parser, gga_fix_quality.clone())?;

                    let fix_quality: u8 = gga_fix_quality
                        .as_deref()
                        .and_then(|q| q.parse().ok())
                        .unwrap_or(0);
                    if fix_quality >= 1 {
                        if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                            let speed_kmh = parser.speed_over_ground.unwrap_or(0.0) as f64;
                            dist_tracker.update_gps(lat, lon, speed_kmh);

                            // ====== NEW: PURE GPS FALLBACK TELEMETRY & ARRIVAL ======
                            if !arduino_connected {
                                let fix_label = match fix_quality {
                                    1 => "GPS",
                                    2 => "DGPS",
                                    4 => "RTK_FIX",
                                    5 => "RTK_FLOAT",
                                    _ => "NO_FIX",
                                };
                                
                                // Fall back to GPS track-made-good angle for heading UI visualization
                                let track_yaw = parser.true_course.unwrap_or(0.0) as f64;
                                
                                // Stream live positions back to the Python GCS at 1Hz
                                let telem_msg = format!("TELEM,{:.3},{:.3},{:.2},{}", dist_tracker.x, dist_tracker.y, track_yaw, fix_label);
                                let _ = socket.send_to(telem_msg.as_bytes(), "172.20.10.7:5008");

                                // Evaluate arrival using pure GPS leg distance calculation
                                if is_active_leg {
                                    // 0.2m mirrors the ARRIVAL_THRESHOLD_M constant in distance_tracker
                                    if dist_tracker.distance_traveled_m >= target_dist - 0.2 {
                                        println!("[GPS-Only Mode] Target reached! Stopping.");
                                        let _ = socket.send_to(b"ARRIVED", "172.20.10.7:5008");
                                        is_active_leg = false;
                                    }
                                }
                            }
                        }
                    }
                }

                parser = next_parser;
                gga_fix_quality = next_gga_fix_quality;
            }

            if let Some(ref rx) = ntrip_rx {
            loop {
                match rx.try_recv() {
                    Ok(data) => {
                        // println!("Writing {} bytes of NTRIP data to serial", data.len());
                        // Log RTCM data
                        logger.log_rtcm(&data);
                        gps_port.as_mut().unwrap().write_all(&data)?;
                        gps_port.as_mut().unwrap().flush()?;
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(e) => {
                        eprintln!("NTRIP channel error: {:?}", e);
                        break;
                    }
                }
            }
        } 
        // =========================================================================



        // --- ARDUINO SENSOR BRANCH ---
        // This is where the code processes the 10Hz data from the rover
        // --- ARDUINO SENSOR BRANCH ---
        if arduino_connected {
            let sensor_data = match arduino_port.as_mut().unwrap().read_data() {
                Ok(Some(d)) => d,
                Ok(None) => continue,
                Err(_) => continue,
            };
            
            logger.log_sensor_data(&sensor_data);
            display.update_arduino(&mut stdout, &sensor_data)?;
            
            // Bring all sensor data into scope so we can send to UI logger
            // Fall back to 0.0 for any missing IMU fields (IMU disconnected)
                let ax = sensor_data.acc_lin_x.unwrap_or(0.0) as f64;
                let ay = sensor_data.acc_lin_y.unwrap_or(0.0) as f64;
                let az = sensor_data.acc_lin_z.unwrap_or(0.0) as f64;
                let pitch = sensor_data.euler_y.unwrap_or(0.0) as f64;
                let yaw = sensor_data.euler_x.unwrap_or(0.0) as f64;
                let roll = sensor_data.euler_z.unwrap_or(0.0) as f64;
                let voltage_left = sensor_data.voltage_left.unwrap_or(0.0) as f64;
                let voltage_right = sensor_data.voltage_right.unwrap_or(0.0) as f64;
                let current_left = sensor_data.current_left_ma.unwrap_or(0.0) as f64;
                let current_right = sensor_data.current_right_ma.unwrap_or(0.0) as f64;
                let motor_current_left = sensor_data.motor_current_left.unwrap_or(0.0) as f64;
                let motor_current_right = sensor_data.motor_current_right.unwrap_or(0.0) as f64;
                let rpm_left = sensor_data.rpm_left.unwrap_or(0.0) as f64;
                let rpm_right = sensor_data.rpm_right.unwrap_or(0.0) as f64;
                let sonar_mm = sensor_data.sonar_mm.unwrap_or(0.0) as f64;
                let tof_mm = sensor_data.tof_mm.unwrap_or(0.0) as f64;
                let rotations_left = sensor_data.rotations_left.unwrap_or(0.0) as f64;
                let rotations_right = sensor_data.rotations_right.unwrap_or(0.0) as f64;
                let gyro_x = sensor_data.gyro_x.unwrap_or(0.0) as f64;
                let gyro_y = sensor_data.gyro_y.unwrap_or(0.0) as f64;
                let gyro_z = sensor_data.gyro_z.unwrap_or(0.0) as f64;

                // 1. Always update the distance tracker and send telemetry
                let dist_out = dist_tracker.update_imu(ay as f64, pitch as f64, target_dist, yaw as f64);
                let _ = display.update_distance_and_accel(&mut stdout, dist_out.dist_traveled_m as f32, dist_out.accel_filtered as f32);
                
                // Send live position + GPS fix quality back to Python
                let fix_label = match gga_fix_quality
                    .as_deref()
                    .and_then(|q| q.trim().parse::<u8>().ok())
                    .unwrap_or(0)
                {
                    1 => "GPS",
                    2 => "DGPS",
                    4 => "RTK_FIX",
                    5 => "RTK_FLOAT",
                    _ => "NO_FIX",
                };
                let telem_msg = format!("TELEM,{:.3},{:.3},{:.2},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}", dist_tracker.x, dist_tracker.y, yaw, fix_label, ax, ay, az, pitch, roll, voltage_left, voltage_right, current_left, current_right, motor_current_left, motor_current_right, rpm_left, rpm_right, sonar_mm, tof_mm, rotations_left, rotations_right, gyro_x, gyro_y, gyro_z);
                let _ = socket.send_to(telem_msg.as_bytes(), "172.20.10.7:5008");
                
                // 2. Control Logic (Only if we are trying to move somewhere)
                if is_active_leg {
                    // Check for Arrival
                    if dist_out.arrived {
                        println!("Target reached! Stopping.");
                        let _ = socket.send_to(b"ARRIVED", "172.20.10.7:5008");
                        is_active_leg = false; 
                    } else {
                    // Left Motor RPM Control
                    if let Some(rpm_l) = sensor_data.rpm_left {
                        let rpm_cmd = rpm_ctrl.compute_motor_commands(rpm_l as f64, target_rpm);
                        base_throttle = rpm_cmd.base_throttle;
                        let _ = motor::update_pwm_l(&mut motor_pin_l, rpm_cmd.left_pwm as i64);
                    }
                    // Right Motor Yaw Control (Heading)
                    if let Some(euler_x) = sensor_data.euler_x {
                        let yaw_cmd = yaw_ctrl.compute_motor_commands(euler_x as f64, target_yaw, base_throttle);
                        let _ = motor::update_pwm_r(&mut motor_pin_r, yaw_cmd.right_pwm_us as i64);
                    }}
                } else {
                    // MISSION NOT ACTIVE: Force motors to neutral
                    let _ = motor::update_pwm_l(&mut motor_pin_l, 1500);
                    let _ = motor::update_pwm_r(&mut motor_pin_r, 1500);
                }
            } else {
                // SENSOR DATA MISSING: Safety stop
                let _ = motor::update_pwm_l(&mut motor_pin_l, 1500);
                let _ = motor::update_pwm_r(&mut motor_pin_r, 1500);
            }
    } // End of arduino_connected branch
    } // End of arduino_connected branch
        // End of Arduino Sensor Branch
}
        
        
        



