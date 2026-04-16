// src/main.rs
use clap::Parser;
use crossterm::execute;
use std::io::{Write, stdout};
use std::path::PathBuf;
use std::sync::mpsc::{self};
use nalgebra::Vector3;

use maarco::{
    display_gyro, 
    gps, 
    gps_serial, 
    logging_gyro::Logger, 
    ntrip, 
    usb_serial_new, // Ensure this matches your crate's naming
    ekf::EKF,
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

fn project_lat_lon(lat: f64, lon: f64, ref_lat: f64, ref_lon: f64) -> Vector3<f64> {
    const EARTH_RADIUS: f64 = 6_378_137.0;
    let d_lat = (lat - ref_lat).to_radians();
    let d_lon = (lon - ref_lon).to_radians();
    let y = d_lat * EARTH_RADIUS;
    let x = d_lon * EARTH_RADIUS * ref_lat.to_radians().cos();
    Vector3::new(x, y, 0.0) 
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let log_file = match args.log_file {
        Some(path) => path,
        None => {
            std::fs::create_dir_all("logs")?;
            let now = chrono::Local::now();
            let filename = format!("{}", now.format("%Y-%m-%d_%H-%M-%S.csv"));
            PathBuf::from("logs").join(filename)
        }
    };

    let logger = maarco::logging_gyro::Logger::new(log_file)?;
    let mut gps_port = gps_serial::open_port(args.gps_port);
    let mut arduino_port = usb_serial_new::open_port(args.arduino_port);

    let gps_connected = gps_port.is_ok();
    let arduino_connected = arduino_port.is_ok();

    if !gps_connected && !arduino_connected {
        eprintln!("No serial ports connected. Exiting.");
        return Ok(());
    }


    use std::io::{self, Write};

    fn get_tuning_params() -> (f64, f64, f64, f64, f64, f64, f64) {
        // Defaults: q_pos, q_vel, q_ori, r_fix, r_float, r_std, cutoff
        let mut params = (0.01, 0.1, 0.001, 0.0001, 0.25, 25.0, 10.0);

        println!("--- EKF TUNING SETUP ---");
        println!("Enter values separated by commas, or press ENTER to use defaults.");
        println!("Order: q_pos, q_vel, q_ori, r_fix, r_float, r_std, cutoff_hz");
        println!("Default: 0.01, 0.1, 0.001, 0.0001, 0.25, 25.0, 10.0");
        print!("> ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        let trimmed = input.trim();

        if !trimmed.is_empty() {
            let vals: Vec<f64> = trimmed
                .split(',')
                .map(|s| s.trim().parse().unwrap_or(0.0))
                .collect();

            if vals.len() == 7 {
                params = (vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]);
                println!("Tuning loaded successfully!");
            } else {
                println!("Warning: Expected 7 values, but got {}. Using defaults.", vals.len());
            }
        } else {
            println!("Using default tuning.");
        }

        params
    }

    let mut parser = gps::parser::build_parser();
    let mut gga_fix_quality: Option<String> = None;
    let mut stdout = stdout();
    let mut display = display_gyro::Display::new();

    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();
    if let Some(mount) = args.ntrip_mount {
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx, &mount);
        });
    }

    let mut ntrip_status: String = String::from("No connection");
    let mut last_imu_time = std::time::Instant::now();
    let mut last_gps_time = std::time::Instant::now();
    let mut anchor: Option<(f64, f64)> = None;


    // 1. Get the tuning values before the display starts
    let (q_pos, q_vel, q_ori, r_fixed, r_float, r_standard, cutoff_hz) = get_tuning_params();
    
    // // --- EKF TUNING PANEL ---
    // let q_pos = 0.01;   // Trust in position physics. 
    //                     // Increase if the robot is very agile/fast.
    //                     // Decrease if the position drifts too much while sitting still.

    // let q_vel = 0.1;    // Trust in velocity physics. 
    //                     // Usually the "noisiest" part of the model. 
    //                     // If the robot "overshoots" its position when stopping, lower this.

    // let q_ori = 0.001;  // Trust in the Gyro integration for heading.
    //                     // Keep this SMALL. Gyros are usually very precise over short bursts.
    //                     // If your heading "shivers" when stationary, lower this.

    // let r_rtk_fixed = 0.0001; // "The Truth." 
    //                       // At 0.0001, the EKF will "snap" the estimate to the GPS.
    //                       // If the robot "teleports" when getting a fix, increase this slightly.

    // let r_rtk_float = 0.25;   // "The Suggestion." 
    //                         // RTK Float is good for decimeter accuracy. 
    //                         // This value allows the IMU to "smooth out" GPS jumps.

    // let r_standard = 25.0;    // "The Guess." 
    //                         // Standard GPS can be off by meters. 
    //                         // High R tells the EKF: "Mostly ignore this; rely on the IMU/Physics."
                
                                
    // let cutoff_hz = 10.0; // The "Vibration Killer."
    //                     // 10Hz is standard for ground robots.
    //                     // TUNE THIS FIRST: 
    //                     // 1. Set robot on blocks, spin motors. 
    //                     // 2. If 'v' in the EKF starts climbing, LOWER this to 5.0 or 2.0.
    //                     // 3. If the robot feels "delayed" when you push it, RAISE this to 15.0 or 20.0.
                        
    // Update this line to include the new parameter
    let mut ekf = EKF::new(q_pos, q_vel, q_ori, cutoff_hz);

    let mut total_distance = 0.0;
    let mut last_ekf_pos = ekf.p;
    loop {
        // --- 1. GPS HANDLING ---
        if gps_connected {
            if let Ok(sentences) = gps_port.as_mut().unwrap().read_sentences() {
                for sentence in &sentences {
                    let mut next_parser = parser.clone();
                    gps::parser::parse_nmea_sentence(&mut next_parser, sentence);

                    if sentence.contains("GGA") {
                        let parts: Vec<&str> = sentence.split(',').collect();
                        if parts.len() > 6 {
                            gga_fix_quality = Some(parts[6].to_string());
                        }
                    }

                    // Check for new GPS data
                    if next_parser.fix_time != parser.fix_time && parser.fix_time.is_some() {
                        logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                        display.update_gps(&mut stdout, &parser, gga_fix_quality.clone(), &ntrip_status)?;

                        if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                            let (lat_f, lon_f) = (lat as f64, lon as f64);
                            let ref_point = *anchor.get_or_insert((lat_f, lon_f));
                            let gps_pos_meters = project_lat_lon(lat_f, lon_f, ref_point.0, ref_point.1);

                            let r_value = match gga_fix_quality.as_deref() {
                                Some("4") => r_fixed,
                                Some("5") => r_float,
                                _         => r_standard,
                            };

                            ekf.update_gps(gps_pos_meters, r_value);
                            // Inside main loop after ekf.predict or ekf.update_gps
                        }
                        last_gps_time = std::time::Instant::now();
                    }
                    parser = next_parser;
                }
            }

            // Write NTRIP data
            while let Ok(msg) = rx.try_recv() {
                ntrip_status = msg.to_string();
                if let ntrip::NTRIPMessage::Rtcm(data) = msg {
                    logger.log_rtcm(&data);
                    let port = gps_port.as_mut().unwrap();
                    let _ = port.write_all(&data);
                    let _ = port.flush();
                }
            }
        }

        // --- 2. ARDUINO / IMU HANDLING ---
        if arduino_connected {
            if let Ok(Some(sensor_data)) = arduino_port.as_mut().unwrap().read_data() {
                logger.log_sensor_data(&sensor_data);
                display.update_arduino(&mut stdout, &sensor_data)?;

                let now = std::time::Instant::now();
                let dt_imu = now.duration_since(last_imu_time).as_secs_f64();
                last_imu_time = now;

                if dt_imu > 0.0 && dt_imu < 0.5 {
                    // Use .unwrap_or(0.0) since your fields are Option<f32>
                    let accel = Vector3::new(
                        sensor_data.acc_lin_x.unwrap_or(0.0) as f64,
                        sensor_data.acc_lin_y.unwrap_or(0.0) as f64,
                        sensor_data.acc_lin_z.unwrap_or(0.0) as f64,
                    );

                    let gyro = Vector3::new(
                        sensor_data.gyro_x.unwrap_or(0.0) as f64,
                        sensor_data.gyro_y.unwrap_or(0.0) as f64,
                        sensor_data.gyro_z.unwrap_or(0.0) as f64,
                    );

                    ekf.predict(accel, gyro, dt_imu);

                    // Track distance for the log
                    total_distance += (ekf.p - last_ekf_pos).norm();
                    last_ekf_pos = ekf.p;

                    let log_entry = maarco::logging_gyro::EkfLogData {
                        timestamp_ns: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64,
                        pos_x: ekf.p.x,
                        pos_y: ekf.p.y,
                        vel_n: ekf.v.x,
                        vel_e: ekf.v.y,
                        distance_traveled: total_distance,
                        yaw_imu_deg: ekf.get_yaw_degrees(), // Ensure this method exists in your EKF
                        q_pos: q_pos,                      // Your tuning parameter
                        r_fixed: r_fixed,                  // Your tuning parameter
                    };
                    logger.log_ekf(log_entry);
                    display.update_ekf(&mut stdout,ekf.p,ekf.v,ekf.q,total_distance)?;
                }
            }
        }
    }
}