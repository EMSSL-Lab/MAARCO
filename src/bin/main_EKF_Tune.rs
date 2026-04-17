// src/main.rs
use clap::Parser;
use crossterm::execute;
use std::io::{Write, stdout};
use std::path::PathBuf;
use std::sync::mpsc::{self};
use nalgebra::Vector3;
use std::net::UdpSocket;

use maarco::{
    display_gyro, 
    gps, 
    gps_serial, 
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
    let rx_socket = UdpSocket::bind("0.0.0.0:5007")?;
    rx_socket.set_nonblocking(true)?;

    // Define defaults (The Tuning Panel)
    let q_pos_init = 0.01;
    let q_vel_init = 0.1;
    let q_ori_init = 0.001;
    let r_fix_init = 0.0001;
    let r_float_init = 0.25;
    let mut r_standard = 25.0; // Note: r_standard was missing an init value later
    let cutoff_hz_init = 10.0;
    let error_init = 0.0; // Initialize error percentage

    // Now initialize the mutable versions
    let mut q_pos = q_pos_init;
    let mut q_vel = q_vel_init;
    let mut q_ori = q_ori_init;
    let mut r_fixed = r_fix_init;
    let mut r_float = r_float_init;
    let mut cutoff_hz = cutoff_hz_init;
    let mut error = error_init;

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

    println!("GPS connected: {} | Arduino connected: {}", gps_connected, arduino_connected);


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
    let mut anchor: Option<(f64, f64)> = None;
    let mut last_gps_pos = Vector3::new(0.0, 0.0, 0.0);
    let telemetry_socket = std::net::UdpSocket::bind("0.0.0.0:0")?; // Telemetry socket

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
    let mut total_distance = 0.0;
    let relative_distance = 0.0;
    
    let mut ekf = EKF::new(q_pos, q_vel, q_ori, r_fixed, r_float,cutoff_hz);
    let mut last_ekf_pos = ekf.p;    

    loop {

        let mut buf = [0u8; 1024];
        if let Ok((size, _)) = rx_socket.recv_from(&mut buf) {
            let msg = String::from_utf8_lossy(&buf[..size]);
            let parts: Vec<&str> = msg.split(',').collect();
            
            // Packet format from Python: "GAIN,q_pos,q_vel,q_ori,r_fix,r_float,cutoff_hz"
            if parts[0] == "GAIN" && parts.len() == 8 {
                q_pos = parts[1].parse().unwrap_or(q_pos);
                q_vel = parts[2].parse().unwrap_or(q_vel);
                q_ori = parts[3].parse().unwrap_or(q_ori);
                r_fixed = parts[4].parse().unwrap_or(r_fixed);
                r_float = parts[5].parse().unwrap_or(r_float);
                cutoff_hz = parts[6].parse().unwrap_or(cutoff_hz);
                r_standard = 25.0;
                error = parts[7].parse().unwrap_or(error);
                
                // Apply to the live EKF instance WITHOUT losing state
                ekf.update_tuning(q_pos, q_vel, q_ori, r_fixed, r_float);
                
                // Log it so you know the command worked
                // println!("Gains Updated: Qp:{:.4} Qv:{:.4} Qo:{:.4} Rf:{:.4} Rfl:{:.4}", q_pos, q_vel, q_ori, r_fixed, r_float);

            }
        }
        
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
                            last_gps_pos = gps_pos_meters;
                            let gps_telemetry = format!("POS,{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2}",
                                ekf.p.x,
                                ekf.p.y,
                                ekf.get_yaw_degrees(),
                                total_distance,
                                relative_distance,
                                gps_pos_meters.x, // Raw GPS X
                                gps_pos_meters.y  // Raw GPS Y
                            );
                            // println!("TX telemetry (GPS update): {}", gps_telemetry);
                            if let Err(_err) = telemetry_socket.send_to(gps_telemetry.as_bytes(), "172.20.10.5:5008") {
                                // eprintln!("Telemetry send error: {}", err);
                            }
                            // Inside main loop after ekf.predict or ekf.update_gps
                        }
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

                let lin_accel_x = sensor_data.acc_lin_x.unwrap_or(0.0) as f64;
                let lin_accel_y = sensor_data.acc_lin_y.unwrap_or(0.0) as f64;
                let lin_accel_z = sensor_data.acc_lin_z.unwrap_or(0.0) as f64;

                let gyro_x = sensor_data.gyro_x.unwrap_or(0.0) as f64;
                let gyro_y = sensor_data.gyro_y.unwrap_or(0.0) as f64;
                let gyro_z = sensor_data.gyro_z.unwrap_or(0.0) as f64;

                if dt_imu > 0.0 && dt_imu < 0.5 {
                    // Use .unwrap_or(0.0) since your fields are Option<f32>
                    let lin_accel_z_new = lin_accel_z;
                    let lin_accel_x_new = lin_accel_y;
                    let lin_accel_y_new = -lin_accel_x;
                    let accel = Vector3::new(
                        lin_accel_z_new,
                        lin_accel_x_new,
                        lin_accel_y_new,
                    );

                    let pitch_rate = gyro_z;
                    let roll_rate = -gyro_y;
                    let yaw_rate = -gyro_x;
                    let gyro = Vector3::new(
                        pitch_rate,
                        roll_rate,
                        yaw_rate,
                    );

                    ekf.predict(accel, gyro, dt_imu);
                    
                    let diff = (ekf.p - last_ekf_pos).norm();
                    // Define your "Reality Band"
                    let floor = 0.005;   // 5mm: Ignore anything smaller (high-frequency noise)
                    let ceiling = 0.5;   // 0.5m: Ignore any single-frame jump larger than this (EKF glitches)
                    // Only accumulate if the movement is "sane"
                    if diff > floor && diff < ceiling {
                        total_distance += diff;
                    }
                    // Note: We update last_ekf_pos REGARDLESS of the filter to keep the delta small
                    last_ekf_pos = ekf.p;
                    // Calculate the relative distance (displacement from start point 0,0,0)
                    let relative_distance = ekf.p.norm();
                    // Protocol: POS, x, y, heading_degrees, total_distance
                    let telemetry = format!("POS,{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2}", 
                        ekf.p.x, 
                        ekf.p.y, 
                        ekf.get_yaw_degrees(), 
                        total_distance,
                        relative_distance, // You can also send this if you want to track how far the robot has moved since the last GPS fix
                        last_gps_pos.x, // Last known GPS X
                        last_gps_pos.y  // Last known GPS Y
                    );

                    // println!("TX telemetry (IMU update): {}", telemetry);
                    if let Err(_err) = telemetry_socket.send_to(telemetry.as_bytes(), "172.20.10.5:5008") {
                        // eprintln!("Telemetry send error: {}", err);
                    }

                    let log_entry = maarco::logging_gyro::EkfLogData {
                        timestamp_ns: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64,
                        pos_x: ekf.p.x,
                        pos_y: ekf.p.y,
                        vel_n: ekf.v.x,
                        vel_e: ekf.v.y,
                        last_gps_pos_x: last_gps_pos.x, // Last known GPS X
                        last_gps_pos_y: last_gps_pos.y,  // Last known GPS Y
                        distance_traveled: total_distance,
                        relative_distance: relative_distance,
                        yaw_imu_deg: ekf.get_yaw_degrees(), // Ensure this method exists in your EKF
                        euler_x: sensor_data.euler_x.unwrap_or(0.0) as f64,
                        q_pos: q_pos,                      // Your tuning parameter
                        q_vel: q_vel,
                        q_ori: q_ori,
                        r_fixed: r_fixed,                  // Your tuning parameter
                        r_float: r_float,
                        error_percent: error,
                    };
                    logger.log_ekf(log_entry);
                    display.update_ekf(&mut stdout,ekf.p,ekf.v,ekf.q,total_distance)?;
                }
            }
        }
    }
}