// src/logging.rs
use csv::Writer;
use nmea::Nmea;
use serde::Serialize;
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};
use crate::usb_serial::SensorData;


#[derive(Debug, Clone, Serialize)]
pub struct RtcmData {
    message_type: Option<u16>,
    data_length: usize,
    data_hex: String, // Hex representation of RTCM data
}

#[derive(Debug, Clone, Serialize)]
pub struct LogEntry<T> {
    pub timestamp_ns: u64,
    #[serde(flatten)]
    pub data: T,
}

#[derive(Debug, Clone, Serialize)]
pub enum LoggerPackets {
    NmeaSentence(Nmea),
    RtcmData(RtcmData),
    SensorData(SensorData),
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
    pub fn log_nmea(&self, parser: Nmea) {
        let _ = self.tx.send(LoggerPackets::NmeaSentence(parser));
    }

    /// Log RTCM correction data
    pub fn log_rtcm(&self, data: &[u8]) {
        let message_type = extract_rtcm_message_type(data);
        let data_hex = hex_encode(data);
        let data = RtcmData {
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
    let file_stem = log_file_path.file_stem().unwrap_or_default().to_string_lossy();
    let extension = log_file_path.extension().unwrap_or_default().to_string_lossy();
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

    let gps_path = make_path("_gps");
    let rtcm_path = make_path("_rtcm");
    let sensor_path = make_path("_sensor");

    let mut gps_writer = Writer::from_path(gps_path)?;
    let mut rtcm_writer = Writer::from_path(rtcm_path)?;
    let mut sensor_writer = Writer::from_path(sensor_path)?;

    loop {
        match rx.recv() {
            Ok(packet) => {
                let timestamp_ns = get_timestamp_nanos();
                match packet {
                    LoggerPackets::NmeaSentence(data) => {
                        let entry = LogEntry { timestamp_ns, data };
                        gps_writer.serialize(entry)?;
                        gps_writer.flush()?;
                    }
                    LoggerPackets::RtcmData(data) => {
                        let entry = LogEntry { timestamp_ns, data };
                        rtcm_writer.serialize(entry)?;
                        rtcm_writer.flush()?;
                    }
                    LoggerPackets::SensorData(data) => {
                        let entry = LogEntry { timestamp_ns, data };
                        sensor_writer.serialize(entry)?;
                        sensor_writer.flush()?;
                    }
                }
            }
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
