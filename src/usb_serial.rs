use serde::Serialize;
use serialport;
use serialport::Error;
use serialport::TTYPort;
use std::io::{self, BufRead, BufReader, Write};
use std::path::PathBuf;
use std::time::Duration;


#[derive(Debug, Clone, Serialize)]
pub struct SensorData {
    pub motor_1_rpm: Option<f32>,
    pub motor_2_rpm: Option<f32>,
    pub motor_1_tot_rotations: Option<f32>,
    pub motor_2_tot_rotations: Option<f32>,
    pub time_ms: Option<u32>,
    pub roll: Option<f32>,
    pub pitch: Option<f32>,
    pub yaw: Option<f32>,
    pub current_motor_1_ma: Option<f32>,
    pub current_motor_2_ma: Option<f32>,
    pub acc_x: Option<f32>,
    pub acc_y: Option<f32>,
    pub acc_z: Option<f32>,
    pub sonar_mm: Option<f32>,
    pub tof1_mm: Option<f32>,
    pub tof2_mm: Option<f32>,
}

impl Default for SensorData {
    fn default() -> Self {
        SensorData {
            motor_1_rpm: None,
            motor_2_rpm: None,
            motor_1_tot_rotations: None,
            motor_2_tot_rotations: None,
            time_ms: None,
            roll: None,
            pitch: None,
            yaw: None,
            current_motor_1_ma: None,
            current_motor_2_ma: None,
            acc_x: None,
            acc_y: None,
            acc_z: None,
            sonar_mm: None,
            tof1_mm: None,
            tof2_mm: None,
        }
    }
}


#[derive(Debug)]
pub struct ArduinoSerialPort {
    reader: BufReader<TTYPort>,  // Buffered reader for line-based reads
}


pub fn open_port(port: PathBuf) -> Result<ArduinoSerialPort, Error> {
    const BAUD_RATE: u32 = 9600;

    match serialport::new(port.to_string_lossy(), BAUD_RATE)
        .timeout(Duration::from_secs(1))  // 1s timeout for reads
        .open_native()
    {
        Ok(arduino_port) => {
            println!("Successfully opened port {} at {} baud.", port.to_string_lossy(), BAUD_RATE);
            let reader = BufReader::new(arduino_port);  // Wrap for buffered line reads
            Ok(ArduinoSerialPort { reader })
        }
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port.to_string_lossy(), e);
            Err(e)
        }
    }
}


impl ArduinoSerialPort {
    pub fn read_line(&mut self) -> Result<SensorData, Error> {
        let mut line = String::new();
        match self.reader.read_line(&mut line) {
            Ok(bytes_read) if bytes_read > 0 => {
                // Remove trailing newline chars
                let trimmed = line.trim_end_matches(|c| c == '\r' || c == '\n');
                if trimmed.is_empty() {
                    return Ok(SensorData::default());
                }
                // println!("Read line: {} ({} bytes)", trimmed, bytes_read);

                // Parse the line into SensorData
                let parts: Vec<&str> = trimmed.split(",").map(|s| s.trim()).collect();
                if parts.len() != 10 {
                    println!("Invalid Arduino data line: {}", trimmed);
                    return Err(Error::new(
                        serialport::ErrorKind::Io(std::io::ErrorKind::InvalidData),
                        format!("Expected 10 parts, got {}", parts.len()),
                    ));
                }

                let motor_1_rpm: f32 = parts[0].parse().unwrap();
                let motor_2_rpm: f32 = parts[1].parse().unwrap();
                let motor_1_tot_rotations: f32 = parts[2].parse().unwrap();
                let motor_2_tot_rotations: f32 = parts[3].parse().unwrap();
                let time_ms: u32 = parts[4].parse().unwrap();
                // let roll: f32 = parts[5].parse().unwrap();
                // let pitch: f32 = parts[6].parse().unwrap();
                // let yaw: f32 = parts[7].parse().unwrap();
                let current_motor_1_ma: f32 = parts[5].parse().unwrap();
                let current_motor_2_ma: f32 = parts[6].parse().unwrap();
                let sonar_mm: f32 = parts[7].parse().unwrap();
                let tof1_mm: f32 = parts[8].parse().unwrap();
                let tof2_mm: f32 = parts[9].parse().unwrap();
                // let acc_x: f32 = parts[7].parse().unwrap();
                // let acc_y: f32 = parts[8].parse().unwrap();
                // let acc_z: f32 = parts[9].parse().unwrap();

                let data = SensorData {
                    motor_1_rpm: Some(motor_1_rpm),
                    motor_2_rpm: Some(motor_2_rpm),
                    motor_1_tot_rotations: Some(motor_1_tot_rotations),
                    motor_2_tot_rotations: Some(motor_2_tot_rotations),
                    time_ms: Some(time_ms),
                    sonar_mm: Some(sonar_mm),
                    tof1_mm: Some(tof1_mm),
                    tof2_mm: Some(tof2_mm),
                    // roll,
                    // pitch,
                    // yaw,
                    current_motor_1_ma: Some(current_motor_1_ma),
                    current_motor_2_ma: Some(current_motor_2_ma),
                    // acc_x,
                    // acc_y,
                    // acc_z,
                    ..Default::default()
                };

                // println!("Parsed data: {:?}", data);
                Ok(data)
            }
            Ok(_) => Err(Error::new(
                serialport::ErrorKind::Io(std::io::ErrorKind::UnexpectedEof),
                "Unexpected EOF",
            )),
            Err(e) => {
                eprintln!("Error reading line from Arduino serial port: {}", e);
                Err(Error::from(e))
            }
        }
    }

    pub fn get_port_mut(&mut self) -> &mut TTYPort {
        self.reader.get_mut()
    }
}


impl Write for ArduinoSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.get_port_mut().write(buf)
    }

    fn flush(&mut self, ) -> io::Result<()> {
        self.get_port_mut().flush()
    }
}

