use serialport::Error;
use serialport::TTYPort;
use std::io::Read;
use std::io::{self, Write};
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;
use std::sync::mpsc::TryRecvError;
use std::thread;
use std::thread::JoinHandle;

use crate::parsing;
use crate::parsing::PqtmParser;
use crate::protocol::PQTMCommand;
use crate::protocol::PqtmOutput;

#[derive(Debug)]
pub struct BaseGPS {
    base_gps_port: TTYPort,
    pqtm_receive_buffer: Option<Receiver<PqtmOutput>>,
    stop_signal: Arc<AtomicBool>,
}

impl BaseGPS {
    const BAUD_RATE: u32 = 115_200;

    /// Starts a thread to read data from the GPS port, extracts complete NMEA sentences.
    pub fn start(&mut self) -> JoinHandle<()> {
        let (tx, rx) = mpsc::channel();
        self.pqtm_receive_buffer = Some(rx);
        self.rtk_reader_thread(tx)
    }

    /// Pops the next available PqtmOutput from the internal buffer, if any.
    pub fn get_pqtm_data(&mut self) -> Option<PqtmOutput> {
        if let Some(rx) = &self.pqtm_receive_buffer {
            match rx.try_recv() {
                Ok(data) => Some(data),
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => {
                    // Thread exited or panicked
                    None
                }
            }
        } else {
            None
        }
    }

    pub fn open_port(port: PathBuf) -> Result<BaseGPS, Error> {
        match serialport::new(port.to_string_lossy(), Self::BAUD_RATE)
            .timeout(std::time::Duration::from_millis(5000))
            .open_native()
        {
            Ok(base_gps_port) => {
                println!("Successfully opened port {}", port.to_string_lossy());
                Ok(BaseGPS {
                    base_gps_port,
                    pqtm_receive_buffer: None,
                    stop_signal: Arc::new(AtomicBool::new(false)),
                })
            }
            Err(e) => {
                eprintln!(
                    "Failed to open \"{}\". Error: {}",
                    port.to_string_lossy(),
                    e
                );
                Err(e)
            }
        }
    }

    fn rtk_reader_thread(&self, tx: Sender<PqtmOutput>) -> JoinHandle<()> {
        let mut reader = BufReader::new(
            self.base_gps_port
                .try_clone_native()
                .expect("Failed to clone GPS port"),
        );
        let mut serial_buf: Vec<u8> = vec![0; 512];
        let stop_signal = self.stop_signal.clone();
        let mut parser = PqtmParser::new();

        thread::spawn(move || {
            while !stop_signal.load(Ordering::Acquire) {
                match reader.read(&mut serial_buf) {
                    Ok(t) if t > 0 => {
                        let text = String::from_utf8_lossy(&serial_buf[..t]).into_owned();
                        // println!("Read line: {}", text.trim());
                        // println!("received_strings: {:?}", text);
                        parser.parse_data(&text);

                        // Process the buffer, search for $PQTM* sentences, parse into objects,
                        // and send via channel (not implemented here)
                    }

                    Ok(_) => {
                        // No data read; continue
                        continue;
                    }
                    Err(e) => {
                        eprintln!("Error reading from GPS port: {}", e);
                        break;
                    }
                }
            }
        })
    }
}

impl Write for BaseGPS {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.base_gps_port.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.base_gps_port.flush()
    }
}
