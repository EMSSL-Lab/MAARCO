use clap::Parser;
use std::io::{self, Write};
use std::path::PathBuf;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use maarco::{
    gps, 
    gps_serial, 
    logging::Logger, 
    motor, 
    ntrip,
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

    // 1. Setup Logging
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
    
    // 2. Initialize Motor Pins
    let (mut motor_pin_l, mut motor_pin_r) = motor::get_motor_pins(13, 18)
        .expect("Failed to initialize motor pins");

    // 3. Spawn Background Logging Thread
    // This thread handles GPS reading, NTRIP corrections, and CSV logging
    let gps_port_path = args.gps_port.clone();
    let ntrip_mount = args.ntrip_mount.clone();
    
    thread::spawn(move || {
        let mut local_logger = logger; 
        let mut parser = gps::parser::build_parser();
        let mut gga_fix_quality: Option<String> = None;
        
        let (tx, rx) = mpsc::channel::<ntrip::NTRIPMessage>();
        
        if let Some(mount) = ntrip_mount {
            let tx_clone = tx.clone();
            thread::spawn(move || {
                ntrip::connect_rtk2go_ntrip(tx_clone, &mount);
            });
        }

        if let Ok(mut port) = gps_serial::open_port(gps_port_path) {
            loop {
                // Read and Parse GPS
                if let Ok(sentences) = port.read_sentences() {
                    for sentence in sentences {
                        let mut next_parser = parser.clone();
                        gps::parser::parse_nmea_sentence(&mut next_parser, &sentence);

                        if sentence.contains("GGA") {
                            let parts: Vec<&str> = sentence.split(',').collect();
                            if parts.len() > 6 {
                                gga_fix_quality = Some(parts[6].to_string());
                            }
                        }

                        // Log data to file on every new GPS epoch
                        if next_parser.fix_time != parser.fix_time && parser.fix_time.is_some() {
                            let _ = local_logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                        }
                        parser = next_parser;
                    }
                }

                // Inject NTRIP RTCM corrections into GPS port
                while let Ok(msg) = rx.try_recv() {
                    if let ntrip::NTRIPMessage::Rtcm(data) = msg {
                        let _ = local_logger.log_rtcm(&data);
                        let _ = port.write_all(&data);
                        let _ = port.flush();
                    }
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
    });

    // 4. Main PWM Input Loop
    let mut left_pwm: i64 = 1500;
    let mut right_pwm: i64 = 1500;

    println!("============================================");
    println!("   RAW PWM CONTROL + BACKGROUND LOGGING     ");
    println!("============================================");
    println!("Commands:");
    println!("  [Left,Right] : Set PWM (e.g., [1600,1400])");
    println!("  c            : Center/Stop (1500,1500)");
    println!("  r            : Re-enter new PWM values");
    println!("--------------------------------------------");

    loop {
        let mut input = String::new();
        print!("PWM > ");
        io::stdout().flush().unwrap();
        
        if io::stdin().read_line(&mut input).is_err() {
            continue;
        }
        
        let command = input.trim().to_lowercase();

        match command.as_str() {
            "c" => {
                left_pwm = 1500;
                right_pwm = 1500;
                println!("Motors Centered: [1500, 1500]");
            }
            "r" => {
                println!("Ready for new PWM input...");
                continue;
            }
            _ => {
                if let Some((l, r)) = parse_pwm(&command) {
                    left_pwm = l as i64;
                    right_pwm = r as i64;
                    println!("PWM Set: [Left: {}, Right: {}]", left_pwm, right_pwm);
                } else {
                    println!("Invalid input. Use [Left,Right] or 'c'/'r'.");
                    continue;
                }
            }
        }

        // Apply PWM updates to hardware
        let _ = motor::update_pwm_l(&mut motor_pin_l, left_pwm);
        let _ = motor::update_pwm_r(&mut motor_pin_r, right_pwm);
    }
}

fn parse_pwm(input: &str) -> Option<(u16, u16)> {
    let cleaned = input.replace('[', "").replace(']', "");
    let parts: Vec<&str> = cleaned.split(',').collect();

    if parts.len() == 2 {
        let left = parts[0].trim().parse::<u16>().ok();
        let right = parts[1].trim().parse::<u16>().ok();
        
        if let (Some(l), Some(r)) = (left, right) {
            return Some((l, r));
        }
    }
    None
}