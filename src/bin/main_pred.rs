// // src/main.rs
// use clap::Parser;
// use crossterm::execute;
// use std::io::{Write, stdout};
// use std::path::PathBuf;
// use std::sync::mpsc::{self};

// mod display;
// mod gps;
// mod gps_serial;
// mod logging;
// mod ntrip;
// mod usb_serial;
// // src/main.rs
// // use std::io::{self, Write, stdout};
// use std::net::UdpSocket;
// // use std::sync::mpsc::{self, TryRecvError};

// // use std::time::Duration;

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

//     let socket = std::net::UdpSocket::bind("127.0.0.1:0")?; //New code


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

//     // Initialize terminal
//     execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

//     // Channel for NTRIP data to write to serial
//     let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();

//     let ml_rx_socket = UdpSocket::bind("0.0.0.0:5006")?;
//     ml_rx_socket.set_nonblocking(true)?;

//     // Start NTRIP thread if configured
//     if let Some(mount) = args.ntrip_mount {
//         std::thread::spawn(move || {
//             ntrip::connect_rtk2go_ntrip(tx, &mount);
//         });
//         println!("Started NTRIP thread");
//     }

//     // NTRIP status string. This gets updated when we receive messages from the NTRIP thread.
//     let mut ntrip_status: String = String::from("No connection");

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
//                 if next_parser.fix_time != parser.fix_time
//                     && parser.fix_time.is_some() {
//                         logger.log_nmea(parser.clone(), gga_fix_quality.clone());
//                         display.update_gps(
//                             &mut stdout,
//                             &parser,
//                             gga_fix_quality.clone(),
//                             &ntrip_status,
//                         )?;
//                     }

//                 parser = next_parser;
//                 gga_fix_quality = next_gga_fix_quality;
//             }

//             // Write any pending NTRIP correction data to serial
//             while let Ok(msg) = rx.try_recv() {
//                 ntrip_status = msg.to_string();
//                 if let ntrip::NTRIPMessage::Rtcm(data) = msg {
//                     logger.log_rtcm(&data);
//                     gps_port.as_mut().unwrap().write_all(&data)?;
//                     gps_port.as_mut().unwrap().flush()?;
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
//                 // println!("Received sensor data: {:?}", sensor_data);

//                 let msg = format!(
//                     "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
//                     sensor_data.time_ms.unwrap_or(0),
//                     sensor_data.voltage_left.unwrap_or(0.0),
//                     sensor_data.current_left_ma.unwrap_or(0.0),
//                     sensor_data.voltage_right.unwrap_or(0.0),
//                     sensor_data.current_right_ma.unwrap_or(0.0),
//                     sensor_data.motor_current_left.unwrap_or(0.0),
//                     sensor_data.motor_current_right.unwrap_or(0.0),
//                     sensor_data.euler_x.unwrap_or(0.0),
//                     sensor_data.euler_y.unwrap_or(0.0),
//                     sensor_data.euler_z.unwrap_or(0.0),
//                     sensor_data.acc_lin_x.unwrap_or(0.0),
//                     sensor_data.acc_lin_y.unwrap_or(0.0),
//                     sensor_data.acc_lin_z.unwrap_or(0.0),
//                     sensor_data.sonar_mm.unwrap_or(0.0),
//                     sensor_data.tof_mm.unwrap_or(0.0),
//                     sensor_data.rpm_left.unwrap_or(0.0),
//                     sensor_data.rpm_right.unwrap_or(0.0),
//                     sensor_data.rotations_left.unwrap_or(0.0),
//                     sensor_data.rotations_right.unwrap_or(0.0)
//                 );
//                 let _ = socket.send_to(msg.as_bytes(), "127.0.0.1:5005");          

//                 logger.log_sensor_data(&sensor_data);
//                 display.update_arduino(&mut stdout, &sensor_data)?;
//             }
//         };

//         // Receive ML predictions from Python via UDP
//         let mut buf = [0; 1024];
//         if let Ok((len, _)) = ml_rx_socket.recv_from(&mut buf) {
//             let msg = String::from_utf8_lossy(&buf[..len]);
//             let parts: Vec<&str> = msg.split(',').collect();
            
//             if parts.len() >= 3 {
//                 display.update_ml(
//                     &mut stdout, 
//                     parts[0].to_string(), // Time
//                     parts[1].to_string(), // Terrain
//                     parts[2].to_string()  // Confidence
//                 )?; 
//             }
//         }
//     }
// }