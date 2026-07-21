use crate::parsing::PQTMParser;
use crate::protocol::response::WireMessage;

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::fd::AsRawFd;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

#[cfg(target_env = "musl")]
const I2C_SLAVE_IOCTL: libc::c_int = 0x0703;
#[cfg(not(target_env = "musl"))]
const I2C_SLAVE_IOCTL: libc::c_ulong = 0x0703;

const ADDR_CMD: u16 = 0x50;
const ADDR_READ: u16 = 0x54;
const ADDR_WRITE: u16 = 0x58;

const CMD_READ: u16 = 0xAA51;
const CMD_WRITE: u16 = 0xAA53;

const TX_LEN_OFFSET: u16 = 0x0008;
const TX_BUF_OFFSET: u16 = 0x2000;
const RX_LEN_OFFSET: u16 = 0x0004;
const RX_BUF_OFFSET: u16 = 0x1000;

const MAX_I2C_BUFFER: usize = 1024;
// Quectel specifies about 10 ms between protocol stages. Use 20 ms to leave
// margin for module processing and Linux scheduling jitter.
const STAGE_DELAY: Duration = Duration::from_millis(20);
const POLL_DELAY: Duration = Duration::from_millis(20);
const WRITE_FREE_RETRIES: usize = 20;

/// LC29H I2C transport.
///
/// Quectel exposes the module as three 7-bit I2C addresses:
/// 0x50 for command words, 0x54 for reading queued GNSS bytes, and 0x58 for
/// writing input bytes. Commands and lengths are little-endian 32-bit words.
pub struct BaseGpsI2c {
    bus: Arc<Mutex<File>>,
    activity_lock: Arc<Mutex<()>>,
    stream_rx: Option<Receiver<WireMessage>>,
    stop_signal: Arc<AtomicBool>,
}

impl BaseGpsI2c {
    pub fn open_bus(path: impl AsRef<Path>) -> io::Result<Self> {
        Self::open_bus_with_activity_lock(path, Arc::new(Mutex::new(())))
    }

    /// Opens the bus with a lock shared by hardware that must not operate
    /// concurrently with a complete LC29H I2C flow.
    pub fn open_bus_with_activity_lock(
        path: impl AsRef<Path>,
        activity_lock: Arc<Mutex<()>>,
    ) -> io::Result<Self> {
        let bus = OpenOptions::new().read(true).write(true).open(path)?;
        Ok(Self {
            bus: Arc::new(Mutex::new(bus)),
            activity_lock,
            stream_rx: None,
            stop_signal: Arc::new(AtomicBool::new(false)),
        })
    }

    /// Starts a polling thread that drains the LC29H transmit buffer and parses
    /// NMEA/PQTM/PAIR sentences into [`WireMessage`] values.
    pub fn start(&mut self) -> JoinHandle<()> {
        let (stream_tx, stream_rx) = mpsc::channel();
        self.stream_rx = Some(stream_rx);

        let bus = Arc::clone(&self.bus);
        let activity_lock = Arc::clone(&self.activity_lock);
        let stop_signal = Arc::clone(&self.stop_signal);

        thread::spawn(move || {
            let mut parser = PQTMParser::new();

            while !stop_signal.load(Ordering::Acquire) {
                match read_available(&activity_lock, &bus) {
                    Ok(bytes) if !bytes.is_empty() => {
                        log::trace!("[GPS-I2C] Read {} bytes", bytes.len());
                        let chunk = String::from_utf8_lossy(&bytes);
                        for msg in parser.parse_data(&chunk) {
                            let _ = stream_tx.send(msg);
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        log::warn!("[GPS-I2C] read error: {}", e);
                        thread::sleep(Duration::from_millis(100));
                    }
                }

                thread::sleep(POLL_DELAY);
            }
        })
    }

    /// Non-blocking: returns the next parsed GPS message if one is queued.
    pub fn try_get_gps_data(&mut self) -> Option<WireMessage> {
        self.stream_rx.as_ref()?.try_recv().ok()
    }

    /// Returns a write-only handle sharing this port's bus and activity lock,
    /// so another thread can inject input bytes (e.g. RTCM corrections)
    /// without borrowing the port itself.
    pub fn writer(&self) -> GpsI2cWriter {
        GpsI2cWriter {
            bus: Arc::clone(&self.bus),
            activity_lock: Arc::clone(&self.activity_lock),
        }
    }
}

/// Cloneable write-only handle to the LC29H input buffer.
///
/// Writes are serialized against the polling thread and any hardware sharing
/// the activity lock, exactly like writes through [`BaseGpsI2c`] itself.
#[derive(Clone)]
pub struct GpsI2cWriter {
    bus: Arc<Mutex<File>>,
    activity_lock: Arc<Mutex<()>>,
}

impl Write for GpsI2cWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        write_input(&self.activity_lock, &self.bus, buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Write for BaseGpsI2c {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        write_input(&self.activity_lock, &self.bus, buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for BaseGpsI2c {
    fn drop(&mut self) {
        self.stop_signal.store(true, Ordering::Release);
    }
}

fn read_available(activity_lock: &Arc<Mutex<()>>, bus: &Arc<Mutex<File>>) -> io::Result<Vec<u8>> {
    let _activity = activity_lock.lock().unwrap();
    let mut bus = bus.lock().unwrap();

    thread::sleep(STAGE_DELAY);
    write_command(&mut bus, CMD_READ, TX_LEN_OFFSET, 4)
        .map_err(|e| with_context("read length command", e))?;
    thread::sleep(STAGE_DELAY);

    let available = read_u32(&mut bus, ADDR_READ)
        .map_err(|e| with_context("read available length", e))? as usize;
    if available == 0 {
        return Ok(Vec::new());
    }

    let read_len = available.min(MAX_I2C_BUFFER);
    thread::sleep(STAGE_DELAY);
    write_command(&mut bus, CMD_READ, TX_BUF_OFFSET, read_len as u32)
        .map_err(|e| with_context("read data command", e))?;
    thread::sleep(STAGE_DELAY);

    let mut out = vec![0; read_len];
    read_exact_from(&mut bus, ADDR_READ, &mut out).map_err(|e| with_context("read data", e))?;
    Ok(out)
}

fn write_input(
    activity_lock: &Arc<Mutex<()>>,
    bus: &Arc<Mutex<File>>,
    buf: &[u8],
) -> io::Result<usize> {
    if buf.is_empty() {
        return Ok(0);
    }

    let _activity = activity_lock.lock().unwrap();
    let mut bus = bus.lock().unwrap();

    let mut free = 0usize;
    for _ in 0..WRITE_FREE_RETRIES {
        thread::sleep(STAGE_DELAY);
        write_command(&mut bus, CMD_READ, RX_LEN_OFFSET, 4)
            .map_err(|e| with_context("write free-length command", e))?;
        thread::sleep(STAGE_DELAY);

        free = read_u32(&mut bus, ADDR_READ).map_err(|e| with_context("read free length", e))?
            as usize;
        if free > 0 {
            break;
        }

        thread::sleep(STAGE_DELAY);
    }

    if free == 0 {
        return Err(io::Error::new(
            io::ErrorKind::WouldBlock,
            "LC29H I2C receive buffer has no free space",
        ));
    }

    let write_len = buf.len().min(free).min(MAX_I2C_BUFFER);
    thread::sleep(STAGE_DELAY);
    write_command(&mut bus, CMD_WRITE, RX_BUF_OFFSET, write_len as u32)
        .map_err(|e| with_context("write data command", e))?;
    thread::sleep(STAGE_DELAY);

    set_slave_address(&bus, ADDR_WRITE).map_err(|e| with_context("select write address", e))?;
    bus.write_all(&buf[..write_len])
        .map_err(|e| with_context("write data", e))?;
    thread::sleep(STAGE_DELAY);

    Ok(write_len)
}

fn write_command(bus: &mut File, cmd: u16, offset: u16, len: u32) -> io::Result<()> {
    let bytes = command_bytes(cmd, offset, len);

    set_slave_address(bus, ADDR_CMD)?;
    bus.write_all(&bytes)
}

fn command_bytes(cmd: u16, offset: u16, len: u32) -> [u8; 8] {
    let mut bytes = [0u8; 8];
    let word = ((cmd as u32) << 16) | offset as u32;
    bytes[..4].copy_from_slice(&word.to_le_bytes());
    bytes[4..].copy_from_slice(&len.to_le_bytes());
    bytes
}

fn read_u32(bus: &mut File, addr: u16) -> io::Result<u32> {
    let mut bytes = [0u8; 4];
    read_exact_from(bus, addr, &mut bytes)?;
    Ok(u32::from_le_bytes(bytes))
}

fn read_exact_from(bus: &mut File, addr: u16, buf: &mut [u8]) -> io::Result<()> {
    set_slave_address(bus, addr)?;
    bus.read_exact(buf)
}

fn set_slave_address(bus: &File, addr: u16) -> io::Result<()> {
    let ret = unsafe { libc::ioctl(bus.as_raw_fd(), I2C_SLAVE_IOCTL, addr as libc::c_ulong) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

fn with_context(context: &'static str, source: io::Error) -> io::Error {
    io::Error::new(source.kind(), format!("{context}: {source}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lc29h_addresses_match_application_note() {
        assert_eq!(ADDR_CMD, 0x50);
        assert_eq!(ADDR_READ, 0x54);
        assert_eq!(ADDR_WRITE, 0x58);
    }

    #[test]
    fn read_commands_match_application_note() {
        assert_eq!(
            command_bytes(CMD_READ, TX_LEN_OFFSET, 4),
            [0x08, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            command_bytes(CMD_READ, TX_BUF_OFFSET, 1024),
            [0x00, 0x20, 0x51, 0xAA, 0x00, 0x04, 0x00, 0x00]
        );
    }

    #[test]
    fn write_commands_match_application_note() {
        assert_eq!(
            command_bytes(CMD_READ, RX_LEN_OFFSET, 4),
            [0x04, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            command_bytes(CMD_WRITE, RX_BUF_OFFSET, 15),
            [0x00, 0x10, 0x53, 0xAA, 0x0F, 0x00, 0x00, 0x00]
        );
    }
}
