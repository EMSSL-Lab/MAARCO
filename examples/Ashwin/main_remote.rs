// ==================================== Remote Sensor Bridge ====================================
// src/bin/main_remote.rs
//
// Lightweight sensor relay for laptop-based closed-loop control of the SPV rover.
//
// Architecture:
//   [GPS + Arduino] --> [Pi: this binary] --UDP/WiFi--> [Laptop: Python controllers]
//                                         <--UDP/WiFi--
//
// This binary does NOT run any controller onboard. It:
//   1. Reads GPS NMEA sentences and computes XY position from an origin
//   2. Reads Arduino sensor data (IMU, currents, RPM, ToF, sonar)
//   3. Sends a rich telemetry packet to the laptop at the Arduino rate (~50 Hz)
//   4. Receives motor PWM commands from the laptop and applies them to GPIO
//   5. Enforces a safety watchdog: if no command within timeout, motors stop
//
// Telemetry format (CSV, 26 comma-separated fields):
//   TEL,gps_x,gps_y,heading_deg,gps_speed_kmh,gps_lat,gps_lon,fix_quality,
//       euler_x,euler_y,euler_z,acc_x,acc_y,acc_z,
//       motor_current_L,motor_current_R,rpm_L,rpm_R,rot_L,rot_R,
//       voltage_L,voltage_R,current_L_mA,current_R_mA,sonar_mm,tof_mm
//
// Command protocol (from laptop):
//   CMD,<left_pwm_us>,<right_pwm_us>   -- Motor command (1000-2000 us)
//   SET_ORIGIN                          -- Set GPS origin to current position
//   CALIBRATE_HEADING                   -- Re-calibrate IMU heading offset
//   STOP                                -- Emergency stop (motors to neutral)
//   HEARTBEAT                           -- Keep watchdog alive without moving
//
// Usage:
//   cargo run --bin main_remote -- --ntrip-mount VMAX-LAND-1
//   cargo run --bin main_remote -- --laptop-ip 172.20.10.5

use clap::Parser;
use crossterm::execute;
use std::io::{Write, stdout};
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
    distance_tracker::DistanceTracker,
};

#[derive(Parser, Debug)]
#[command(version, about = "SPV Remote Sensor Bridge - relays all sensor data to laptop controller")]
struct Args {
    /// NTRIP mountpoint for RTK corrections (e.g., VMAX-LAND-1)
    #[arg(long)]
    ntrip_mount: Option<String>,

    /// GPS serial port path
    #[arg(long, default_value = "/dev/ttyS0")]
    gps_port: PathBuf,

    /// Arduino serial port path
    #[arg(long, default_value = "/dev/ttyACM0")]
    arduino_port: PathBuf,

    /// Log file path (default: logs/YYYY-MM-DD_HH-MM-SS_remote.csv)
    #[arg(long)]
    log_file: Option<PathBuf>,

    /// Laptop IP address for telemetry output
    #[arg(long, default_value = "172.20.10.5")]
    laptop_ip: String,

    /// UDP port to send telemetry on
    #[arg(long, default_value_t = 5008)]
    telem_port: u16,

    /// UDP port to receive commands on
    #[arg(long, default_value_t = 5007)]
    cmd_port: u16,

    /// Safety watchdog timeout in milliseconds (stop motors if no command received)
    #[arg(long, default_value_t = 500)]
    watchdog_ms: u64,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    // ── Network configuration ────────────────────────────────────────────
    let laptop_addr = format!("{}:{}", args.laptop_ip, args.telem_port);
    let watchdog_timeout = std::time::Duration::from_millis(args.watchdog_ms);

    let telem_socket = UdpSocket::bind("0.0.0.0:0")?;
    let cmd_socket = UdpSocket::bind(format!("0.0.0.0:{}", args.cmd_port))?;
    cmd_socket.set_nonblocking(true)?;

    // ── Logging ──────────────────────────────────────────────────────────
    let log_file = match args.log_file {
        Some(path) => path,
        None => {
            std::fs::create_dir_all("logs")?;
            let now = chrono::Local::now();
            let filename = format!("{}_remote.csv", now.format("%Y-%m-%d_%H-%M-%S"));
            PathBuf::from("logs").join(filename)
        }
    };
    let logger = Logger::new(log_file)?;

    // ── Serial ports ─────────────────────────────────────────────────────
    let mut gps_port = gps_serial::open_port(args.gps_port);
    let mut arduino_port = usb_serial::open_port(args.arduino_port);
    let gps_connected = gps_port.is_ok();
    let arduino_connected = arduino_port.is_ok();

    if !gps_connected && !arduino_connected {
        eprintln!("ERROR: No serial ports connected. Exiting.");
        return Ok(());
    }
    println!("GPS:     {}", if gps_connected { "Connected" } else { "Not found" });
    println!("Arduino: {}", if arduino_connected { "Connected" } else { "Not found" });

    // ── GPS parser & display ─────────────────────────────────────────────
    let mut parser = gps::parser::build_parser();
    let mut gga_fix_quality: Option<String> = None;
    let mut stdout = stdout();
    let mut display = display::Display::new();
    execute!(stdout, crossterm::cursor::SetCursorStyle::BlinkingBlock)?;

    // ── NTRIP correction stream ──────────────────────────────────────────
    let (ntrip_tx, ntrip_rx) = mpsc::channel::<ntrip::NTRIPMessage>();
    if let Some(mount) = args.ntrip_mount {
        std::thread::spawn(move || {
            ntrip::connect_rtk2go_ntrip(ntrip_tx, &mount);
        });
        println!("NTRIP:   Thread started");
    }
    let mut ntrip_status = String::from("No connection");

    // ── Motor GPIO ───────────────────────────────────────────────────────
    let (mut motor_pin_l, mut motor_pin_r) =
        motor::get_motor_pins(13, 18).expect("Failed to initialize motor GPIO pins");

    // ── Position tracking ────────────────────────────────────────────────
    let mut dist_tracker = DistanceTracker::new();

    // ── IMU heading calibration ──────────────────────────────────────────
    let mut heading_offset: f64 = 0.0;
    let mut should_calibrate_heading = false;
    let mut corrected_yaw: f64 = 0.0;

    // ── Latest GPS-derived state (updated at ~1 Hz, sent in every packet)
    let mut gps_x: f64 = 0.0;
    let mut gps_y: f64 = 0.0;
    let mut gps_speed_kmh: f64 = 0.0;
    let mut gps_lat: f64 = 0.0;
    let mut gps_lon: f64 = 0.0;
    let mut gps_fix = String::from("N/A");

    // ── Motor command state (from laptop) ────────────────────────────────
    let mut cmd_pwm_l: i64 = 1500;   // Neutral (stopped)
    let mut cmd_pwm_r: i64 = 1500;
    let mut last_cmd_time = Instant::now();
    let mut remote_active = false;

    println!("════════════════════════════════════════════");
    println!("  SPV Remote Sensor Bridge");
    println!("  Telemetry  -> {}", laptop_addr);
    println!("  Commands   <- 0.0.0.0:{}", args.cmd_port);
    println!("  Watchdog   :  {} ms", args.watchdog_ms);
    println!("════════════════════════════════════════════");
    println!("Waiting for laptop connection...\n");

    // ══════════════════════════════════════════════════════════════════════
    //  MAIN LOOP
    // ══════════════════════════════════════════════════════════════════════
    loop {
        // ── 1. RECEIVE COMMANDS FROM LAPTOP ──────────────────────────────
        // Drain all pending UDP packets (non-blocking). Process commands
        // immediately. Keep the latest CMD for motor application.
        let mut cmd_buf = [0u8; 1024];
        while let Ok((len, _src)) = cmd_socket.recv_from(&mut cmd_buf) {
            let msg = String::from_utf8_lossy(&cmd_buf[..len]);
            let msg = msg.trim();

            if msg.starts_with("CMD,") {
                // Motor command: CMD,<left_pwm>,<right_pwm>
                let parts: Vec<&str> = msg.split(',').collect();
                if parts.len() >= 3 {
                    if let (Ok(l), Ok(r)) = (
                        parts[1].parse::<i64>(),
                        parts[2].parse::<i64>(),
                    ) {
                        cmd_pwm_l = l.clamp(1000, 2000);
                        cmd_pwm_r = r.clamp(1000, 2000);
                        last_cmd_time = Instant::now();
                        remote_active = true;
                    }
                }
            } else {
                match msg.as_ref() {
                    "SET_ORIGIN" => {
                        if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                            dist_tracker.set_origin(lat, lon);
                            should_calibrate_heading = true;
                            gps_x = 0.0;
                            gps_y = 0.0;
                            println!("[REMOTE] Origin set at ({:.7}, {:.7})", lat, lon);
                        } else {
                            println!("[REMOTE] ERROR: No GPS fix - cannot set origin");
                        }
                    }
                    "CALIBRATE_HEADING" => {
                        should_calibrate_heading = true;
                        println!("[REMOTE] Heading calibration requested");
                    }
                    "STOP" => {
                        cmd_pwm_l = 1500;
                        cmd_pwm_r = 1500;
                        remote_active = false;
                        println!("[REMOTE] EMERGENCY STOP received");
                    }
                    "HEARTBEAT" => {
                        last_cmd_time = Instant::now();
                    }
                    _ => {
                        // Unknown command — ignore silently
                    }
                }
            }
        }

        // ── 2. SAFETY WATCHDOG ───────────────────────────────────────────
        // If the laptop controller crashes or WiFi drops, stop the motors.
        if remote_active && last_cmd_time.elapsed() > watchdog_timeout {
            cmd_pwm_l = 1500;
            cmd_pwm_r = 1500;
            remote_active = false;
            println!("[WATCHDOG] No command for {} ms - motors stopped",
                     args.watchdog_ms);
        }

        // ── 3. READ GPS + PROCESS NTRIP ─────────────────────────────────
        if gps_connected {
            match gps_port.as_mut().unwrap().read_sentences() {
                Ok(sentences) => {
                    for sentence in &sentences {
                        let mut next_parser = parser.clone();
                        gps::parser::parse_nmea_sentence(&mut next_parser, sentence);

                        let mut next_fix = gga_fix_quality.clone();
                        if sentence.contains("GGA") {
                            let parts: Vec<&str> = sentence.split(',').collect();
                            if parts.len() > 6 {
                                next_fix = Some(parts[6].to_string());
                            }
                        }

                        // Log the previous epoch when a new time arrives
                        if next_parser.fix_time != parser.fix_time
                            && parser.fix_time.is_some()
                        {
                            logger.log_nmea(parser.clone(), gga_fix_quality.clone());
                            let _ = display.update_gps(
                                &mut stdout,
                                &parser,
                                gga_fix_quality.clone(),
                                &ntrip_status,
                            );
                        }

                        parser = next_parser;
                        gga_fix_quality = next_fix;
                    }

                    // Update XY position from GPS
                    if let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) {
                        gps_lat = lat;
                        gps_lon = lon;
                        gps_speed_kmh =
                            (parser.speed_over_ground.unwrap_or(0.0) as f64) * 1.852;
                        gps_fix = gga_fix_quality
                            .clone()
                            .unwrap_or_else(|| "N/A".to_string());

                        if let Some(state) =
                            dist_tracker.update_gps(lat, lon, gps_speed_kmh)
                        {
                            gps_x = state.x_m;
                            gps_y = state.y_m;
                        }
                    }
                }
                Err(_) => {
                    // GPS read error — skip this iteration, try again next loop
                }
            }

            // Forward NTRIP RTK corrections to GPS receiver
            while let Ok(msg) = ntrip_rx.try_recv() {
                ntrip_status = msg.to_string();
                if let ntrip::NTRIPMessage::Rtcm(data) = msg {
                    logger.log_rtcm(&data);
                    let _ = gps_port.as_mut().unwrap().write_all(&data);
                    let _ = gps_port.as_mut().unwrap().flush();
                }
            }
        }

        // ── 4. READ ARDUINO SENSORS + SEND TELEMETRY ────────────────────
        if arduino_connected {
            match arduino_port.as_mut().unwrap().read_data() {
                Ok(Some(sensor_data)) => {
                    // Update heading from IMU euler_x
                    if let Some(euler_x) = sensor_data.euler_x {
                        if should_calibrate_heading {
                            // Set offset so that corrected_yaw = 90 deg (east)
                            // at the moment of calibration. This matches the
                            // convention: 0 = north, 90 = east in the GCS.
                            heading_offset = -(euler_x as f64) - 90.0;
                            should_calibrate_heading = false;
                            println!(
                                "[IMU] Heading calibrated: offset = {:.1} deg",
                                heading_offset
                            );
                        }
                        corrected_yaw = -(euler_x as f64) - heading_offset;
                    }

                    // Helper: format Option<f32> as "value" or "nan"
                    let f = |v: Option<f32>| -> String {
                        v.map(|x| format!("{:.4}", x))
                            .unwrap_or_else(|| "nan".to_string())
                    };

                    // Build telemetry packet (25 data fields after TEL prefix)
                    let telemetry = format!(
                        "TEL,{:.4},{:.4},{:.2},{:.4},{:.8},{:.8},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                        gps_x,                                   // [1]  meters east
                        gps_y,                                   // [2]  meters north
                        corrected_yaw,                           // [3]  heading deg
                        gps_speed_kmh,                           // [4]  GPS speed
                        gps_lat,                                 // [5]  latitude
                        gps_lon,                                 // [6]  longitude
                        gps_fix,                                 // [7]  fix quality
                        f(sensor_data.euler_x),                  // [8]  raw IMU X
                        f(sensor_data.euler_y),                  // [9]  raw IMU Y
                        f(sensor_data.euler_z),                  // [10] raw IMU Z
                        f(sensor_data.acc_lin_x),                // [11] accel X
                        f(sensor_data.acc_lin_y),                // [12] accel Y
                        f(sensor_data.acc_lin_z),                // [13] accel Z
                        f(sensor_data.motor_current_left),       // [14] motor I_L (A)
                        f(sensor_data.motor_current_right),      // [15] motor I_R (A)
                        f(sensor_data.rpm_left),                 // [16] RPM left
                        f(sensor_data.rpm_right),                // [17] RPM right
                        f(sensor_data.rotations_left),           // [18] cumul rot L
                        f(sensor_data.rotations_right),          // [19] cumul rot R
                        f(sensor_data.voltage_left),             // [20] voltage L (V)
                        f(sensor_data.voltage_right),            // [21] voltage R (V)
                        f(sensor_data.current_left_ma),          // [22] supply I_L mA
                        f(sensor_data.current_right_ma),         // [23] supply I_R mA
                        f(sensor_data.sonar_mm),                 // [24] sonar (mm)
                        f(sensor_data.tof_mm),                   // [25] ToF (mm)
                    );

                    let _ =
                        telem_socket.send_to(telemetry.as_bytes(), &laptop_addr);

                    // Log sensor data and update terminal display
                    logger.log_sensor_data(&sensor_data);
                    let _ = display.update_arduino(&mut stdout, &sensor_data);
                }
                Ok(None) => {
                    // No complete packet yet — buffer is accumulating
                }
                Err(_) => {
                    // Arduino read error — skip, try again next loop
                }
            }
        }

        // ── 5. APPLY MOTOR COMMANDS ──────────────────────────────────────
        // rppal PWM runs continuously at the last set duty cycle.
        // Re-setting to the same value is a no-op, so this is cheap.
        let _ = motor::update_pwm_l(&mut motor_pin_l, cmd_pwm_l);
        let _ = motor::update_pwm_r(&mut motor_pin_r, cmd_pwm_r);
    }
}
