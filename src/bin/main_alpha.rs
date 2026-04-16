// src/main.rs
use clap::Parser;
use crossterm::execute;
use std::io::{Write, stdout};
use std::path::PathBuf;
use std::sync::mpsc::{self};

mod EKF_alpha;
use nalgebra::Vector3;

use maarco::{
    display_gyro, 
    gps, 
    gps_serial, 
    // Notice we go one level deeper here:
    logging::Logger, 
    ntrip, 
    usb_serial_new,
    EKF_alpha::EKF;

};

// In main.rs
struct NavState {
    anchor_lat_lon: Option<(f64, f64)>,
}

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

fn project_lat_lon(lat: f64, lon: f64, ref_lat: f64, ref_lon: f64) -> Vector3<f64> {
    const EARTH_RADIUS: f64 = 6_378_137.0; // WGS84 Semi-major axis
    
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

    let logger = logging::Logger::new(log_file)?;
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

    // Initialize terminal
    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    // Channel for NTRIP data to write to serial
    let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();

    // Start NTRIP thread if configured
    if let Some(mount) = args.ntrip_mount {
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx, &mount);
        });
        println!("Started NTRIP thread");
    }

    // NTRIP status string. This gets updated when we receive messages from the NTRIP thread.
    let mut ntrip_status: String = String::from("No connection");

    let mut ekf = EKF::new(); // 10 Hz IMU
    let mut last_imu_time = std::time::Instant::now();
    let mut last_gps_time = std::time::Instant::now();
    // Outside the loop
    let mut anchor: Option<(f64, f64)> = None;

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

                // CHECK FOR NEW GPS EPOCH (New data arrived)
                if next_parser.fix_time != parser.fix_time && parser.fix_time.is_some() {
                    logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                    display.update_gps(&mut stdout, &parser, gga_fix_quality.clone(), &ntrip_status)?;

                    if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                        let (lat_f, lon_f) = (lat as f64, lon as f64);
                        
                        // Set Anchor (Origin)
                        let ref_point = *anchor.get_or_insert((lat_f, lon_f));
                        
                        // Convert to Meters
                        let gps_pos_meters = project_lat_lon(lat_f, lon_f, ref_point.0, ref_point.1);

                        // Calculate time since last GPS update
                        let now = std::time::Instant::now();
                        let dt_gps = now.duration_since(last_gps_time).as_secs_f64();
                        last_gps_time = now;

                        // 1. Determine trust levels based on RTK status
                        let (alpha_p, alpha_v) = match gga_fix_quality.as_deref() {
                            Some("4") => (0.6, 0.3),  // RTK Fixed: Trust heavily
                            Some("5") => (0.2, 0.1),  // RTK Float: Trust moderately
                            _ => (0.05, 0.01),        // Standard Fix: Trust lightly
                        };

                        // 2. Perform the update ONLY if time has passed
                        if dt_gps > 0.001 {
                            ekf.update_gps(gps_pos_meters, dt_gps, alpha_p, alpha_v);
                        }
                        println!("EKF Fused Pos: x={:.2}, y={:.2}", ekf.p.x, ekf.p.y);
                    }
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
                // println!("Received sensor data: {:?}", sensor_data);
                logger.log_sensor_data(&sensor_data);
                display.update_arduino(&mut stdout, &sensor_data)?;

                let now = std::time::Instant::now();
                let dt_imu = now.duration_since(last_imu_time).as_secs_f64();
                last_imu_time = now;

                // Ensure dt is sane (e.g., ignore if the loop was stuck for 1 second)
                if dt_imu > 0.0 && dt_imu < 0.5 {
                    let accel = Vector3::new(
                        sensor_data.ax as f64,
                        sensor_data.ay as f64,
                        sensor_data.az as f64,
                    );

                    let gyro = Vector3::new(
                        sensor_data.gx as f64,
                        sensor_data.gy as f64,
                        sensor_data.gz as f64,
                    );

                    // Clean syntax:
                    ekf.predict(accel, gyro, dt_imu);
                }
            }
        };
    }
}