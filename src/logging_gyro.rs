// src/logging.rs
use crate::usb_serial_new::SensorData;
use csv::Writer;
use nmea::Nmea;
use serde::Serialize;
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Serialize)]
pub struct RtcmData {
    pub timestamp_ns: u64,
    message_type: Option<u16>,
    data_length: usize,
    data_hex: String, // Hex representation of RTCM data
}

#[derive(Debug, Clone, Serialize)]
pub struct EkfLogData {
    pub timestamp_ns: u64,
    // Position/Velocity
    pub pos_x: f64,
    pub pos_y: f64,
    pub vel_n: f64,
    pub vel_e: f64,
    pub distance_traveled: f64,
    // Heading Comparison
    pub yaw_imu_deg: f64,
    // Metadata/Tuning
    pub q_pos: f64,
    pub q_vel: f64,
    pub q_ori: f64,
    pub r_fixed: f64,
    pub r_float: f64,
}

#[derive(Debug, Clone, Serialize)]
pub enum LoggerPackets {
    NmeaSentence(Nmea, Option<String>),
    RtcmData(RtcmData),
    SensorData(SensorData),
    EkfData(EkfLogData), // New variant
}

#[derive(Debug, Clone, Serialize)]
pub struct GpsLogData {
    pub timestamp_ns: u64,
    pub fix_time: Option<String>,
    pub fix_date: Option<String>,
    pub gga_fix_quality: Option<String>,
    pub avg_snr: Option<f32>,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub altitude_m: Option<f32>,
    pub speed_over_ground: Option<f32>,
    pub true_course: Option<f32>,
    pub num_of_fix_satellites: Option<u32>,
    pub hdop: Option<f32>,
    pub vdop: Option<f32>,
    pub pdop: Option<f32>,
    pub geoid_separation: Option<f32>,
}

impl GpsLogData {
    fn from_nmea(nmea: Nmea, gga_fix_quality: Option<String>, timestamp_ns: u64) -> Self {
        let sats = nmea.satellites();
        let mut avg_snr = 0.0;
        let mut count = 0;
        for sat in &sats {
            if let Some(snr) = sat.snr() {
                avg_snr += snr as f32;
                count += 1;
            }
        }
        let avg_snr_opt = if count > 0 {
            Some(avg_snr / count as f32)
        } else {
            None
        };

        let fix_quality_str = match gga_fix_quality.as_deref() {
            Some("0") => Some("Invalid".to_string()),
            Some("1") => Some("GPS Fix".to_string()),
            Some("2") => Some("DGPS".to_string()),
            Some("3") => Some("PPS".to_string()),
            Some("4") => Some("Fixed".to_string()),
            Some("5") => Some("Float".to_string()),
            Some("6") => Some("Estimated (dead reckoning)".to_string()),
            Some("7") => Some("Manual input mode".to_string()),
            Some("8") => Some("Simulation mode".to_string()),
            Some(other) => Some(format!("Unknown ({})", other)),
            None => None,
        };

        GpsLogData {
            timestamp_ns,
            fix_time: nmea.fix_time.map(|t| format!("{:?}", t)),
            fix_date: nmea.fix_date.map(|d| format!("{:?}", d)),
            gga_fix_quality: fix_quality_str,
            avg_snr: avg_snr_opt,
            latitude: nmea.latitude,
            longitude: nmea.longitude,
            altitude_m: nmea.altitude,
            speed_over_ground: nmea.speed_over_ground,
            true_course: nmea.true_course,
            num_of_fix_satellites: nmea.num_of_fix_satellites,
            hdop: nmea.hdop,
            vdop: nmea.vdop,
            pdop: nmea.pdop,
            geoid_separation: nmea.geoid_separation,
        }
    }
}

/// Logger handle that can be cloned and sent to other threads
#[derive(Clone)]
pub struct Logger {
    tx: Sender<LoggerPackets>,
}

impl Logger {
    /// Create a new logger that writes to the specified file
    pub fn new(log_file_path: PathBuf) -> std::io::Result<Self> {
        let (tx, rx) = mpsc::channel::<LoggerPackets>();

        // Spawn logging thread
        thread::spawn(move || {
            if let Err(e) = run_logger(rx, log_file_path) {
                eprintln!("Logger error: {}", e);
            }
        });

        Ok(Logger { tx })
    }

    /// Log a NMEA sentence
    pub fn log_nmea(&self, parser: Nmea, gga_fix_quality: Option<String>) {
        let _ = self
            .tx
            .send(LoggerPackets::NmeaSentence(parser, gga_fix_quality));
    }

    pub fn log_ekf(&self, data: EkfLogData) {
        let _ = self.tx.send(LoggerPackets::EkfData(data));
    }

    /// Log RTCM correction data
    pub fn log_rtcm(&self, data: &[u8]) {
        let message_type = extract_rtcm_message_type(data);
        let data_hex = hex_encode(data);
        let timestamp_ns = get_timestamp_nanos();
        let data = RtcmData {
            timestamp_ns,
            message_type,
            data_length: data.len(),
            data_hex: data_hex,
        };
        let _ = self.tx.send(LoggerPackets::RtcmData(data));
    }

    pub fn log_sensor_data(&self, data: &SensorData) {
        let _ = self.tx.send(LoggerPackets::SensorData(data.clone()));
    }
}

/// Main logging thread function
fn run_logger(rx: Receiver<LoggerPackets>, log_file_path: PathBuf) -> std::io::Result<()> {
    // Create writers for different data types
    let file_stem = log_file_path
        .file_stem()
        .unwrap_or_default()
        .to_string_lossy();
    let extension = log_file_path
        .extension()
        .unwrap_or_default()
        .to_string_lossy();
    let parent = log_file_path.parent().unwrap_or(std::path::Path::new("."));

    let make_path = |suffix: &str| -> PathBuf {
        let mut name = file_stem.to_string();
        name.push_str(suffix);
        if !extension.is_empty() {
            name.push('.');
            name.push_str(&extension);
        }
        parent.join(name)
    };

    let ekf_path = make_path("_ekf"); // New file: [timestamp]_ekf.csv
    let gps_path = make_path("_gps");
    let rtcm_path = make_path("_rtcm");
    let sensor_path = make_path("_sensor");

    let mut gps_writer = Writer::from_path(gps_path)?;
    let mut rtcm_writer = Writer::from_path(rtcm_path)?;
    let mut sensor_writer = Writer::from_path(sensor_path)?;
    let mut ekf_writer = Writer::from_path(ekf_path)?; // New writer
    
    loop {
        match rx.recv() {
            Ok(packet) => match packet {
                LoggerPackets::NmeaSentence(data, gga_fix_quality) => {
                    let timestamp_ns = get_timestamp_nanos();
                    let gps_data = GpsLogData::from_nmea(data, gga_fix_quality, timestamp_ns);
                    gps_writer.serialize(gps_data)?;
                    gps_writer.flush()?;
                }
                LoggerPackets::RtcmData(data) => {
                    rtcm_writer.serialize(data)?;
                    rtcm_writer.flush()?;
                }
                LoggerPackets::SensorData(data) => {
                    sensor_writer.serialize(data)?;
                    sensor_writer.flush()?;
                }

                LoggerPackets::EkfData(data) => {
                    ekf_writer.serialize(data)?;
                    ekf_writer.flush()?;
                }
            },
            Err(err) => {
                // Channel closed, exit logging thread
                eprintln!("Logging thread error: {}", err);
                break;
            }
        }
    }

    Ok(())
}

/// Get current timestamp in nanoseconds since UNIX epoch
fn get_timestamp_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_nanos() as u64
}

/// Extract RTCM message type from data
/// RTCM v3 format: first 12 bits after preamble contain message type
fn extract_rtcm_message_type(data: &[u8]) -> Option<u16> {
    if data.len() < 3 {
        return None;
    }

    // RTCM v3 starts with 0xD3 preamble
    if data[0] != 0xD3 {
        return None;
    }

    // Message type is in bits 12-23 of the header
    // Byte 1 bits 0-5 and byte 2 bits 0-5 contain the message type
    let msg_type = ((data[1] as u16 & 0x3F) << 6) | ((data[2] as u16) >> 2);
    Some(msg_type)
}

/// Convert bytes to hex string
fn hex_encode(data: &[u8]) -> String {
    data.iter()
        .map(|b| format!("{:02x}", b))
        .collect::<Vec<_>>()
        .join("")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rtcm_message_type_extraction() {
        // RTCM message 1005 example
        let data = vec![0xD3, 0x00, 0x13, 0x3E, 0xD0]; // Simplified
        let msg_type = extract_rtcm_message_type(&data);
        assert!(msg_type.is_some());
    }

    #[test]
    fn test_hex_encode() {
        let data = vec![0xD3, 0x00, 0x13];
        assert_eq!(hex_encode(&data), "d30013");
    }
}
