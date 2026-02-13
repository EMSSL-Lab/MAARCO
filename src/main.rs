// src/main.rs
use clap::Parser;
use crossterm::execute;
use std::io::{self, Write, stdout};
use std::path::PathBuf;
use std::sync::mpsc::{self, TryRecvError};
use std::time::Duration;


mod display;
mod gps;
mod gps_serial;
mod motor;
mod logging;
mod ntrip;
mod usb_serial;
mod yaw_control;
use yaw_control::PDController;

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
    
    // Prompt user for input
 
    let mut gps_port = gps_serial::open_port(args.gps_port);
    let mut arduino_port = usb_serial::open_port(args.arduino_port);

    let gps_connected = gps_port.is_ok();
    let arduino_connected = arduino_port.is_ok();

    // This creates both variables at once from the tuple returned by the function
    let (mut motor_pin_L, mut motor_pin_R) = motor::get_motor_pins(13, 18).expect("Failed to initialize motor pins");

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
    let (tx, rx) = mpsc::channel::<Vec<u8>>();

    // Start NTRIP thread if configured
    if let Some(mount) = args.ntrip_mount {
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(tx, &mount);
        });
        println!("Started NTRIP thread");
    }
    
    // Adjust these gains (2.0, 0.5) once you see how the robot behaves
    let mut yaw_control = PDController::new(2.0, 0.5); 
    let target_yaw = 0.0; // Straight ahead

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
                if next_parser.fix_time != parser.fix_time {
                    if parser.fix_time.is_some() {
                        logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                        display.update_gps(&mut stdout, &parser, gga_fix_quality.clone())?;
                    }
                }

                parser = next_parser;
                gga_fix_quality = next_gga_fix_quality;
            }
            
            // Write any pending NTRIP correction data to serial
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
                        eprintln!("Channel error: {:?}", e);
                        break;
                    }
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
                // if let Some(euler_x) = sensor_data.euler_x {
        
                //     // 1. Map using the unwrapped f32 value
                //     let target_pulse = map_range(euler_x, -180.0, 180.0, 1000, 2000);

                //     // 2. Safety Clamp
                //     let safe_pulse = target_pulse.clamp(1000, 2000);

                //     // 3. Command the motor
                //     let _ = motor::update_pwm(&mut motor_pin_L,motor_pin_R, safe_pulse);
    
                // }

                if let Some(euler_x) = sensor_data.euler_x {
                let corrected_yaw = -(euler_x as f64);
                // Call controller with 1600 as the constant left speed
                let commands = yaw_control.compute_motor_commands(
                    corrected_yaw, 
                    target_yaw, // Target yaw is 0 degrees (straight ahead)
                    1300, // Constant Left Motor Speed
                    1700
                );

                // Command the hardware
                let _ = motor::update_pwm(&mut motor_pin_L, &mut motor_pin_R, commands.left_pwm_us as i64,commands.right_pwm_us as i64);
                }

                logger.log_sensor_data(&sensor_data);
                display.update_arduino(&mut stdout, &sensor_data)?;
            }
            
        };
    }

   fn map_range(val: f32, in_min: f32, in_max: f32, out_min: u64, out_max: u64) -> u64 {
    let result = (val - in_min) * (out_max - out_min) as f32 / (in_max - in_min) + out_min as f32;
    result as u64
}
}
