//! NTRIP Rev1 source support for publishing RTCM correction streams.

use std::collections::HashMap;
use std::env;
use std::fs;
use std::io::{self, BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
const IO_TIMEOUT: Duration = Duration::from_secs(10);
const INITIAL_RECONNECT_DELAY: Duration = Duration::from_secs(2);
const MAX_RECONNECT_DELAY: Duration = Duration::from_secs(30);
const HEALTH_REPORT_INTERVAL: Duration = Duration::from_secs(10);

/// Connection details for an NTRIP Rev1 source.
///
/// Rev1 authenticates a source with its mountpoint and upload password; it has
/// no username field.
#[derive(Clone)]
pub struct NtripSourceConfig {
    host: String,
    port: u16,
    mountpoint: String,
    password: String,
}

impl NtripSourceConfig {
    pub fn new(
        host: impl Into<String>,
        port: u16,
        mountpoint: impl Into<String>,
        password: impl Into<String>,
    ) -> io::Result<Self> {
        let config = Self {
            host: host.into(),
            port,
            mountpoint: mountpoint.into(),
            password: password.into(),
        };
        config.validate()?;
        Ok(config)
    }

    /// Load source credentials from the nearest `.env` file.
    ///
    /// Process environment values override values from the file. The required
    /// keys are `NTRIP_IP`, `NTRIP_PORT`, `NTRIP_MOUNTPOINT`, and
    /// `NTRIP_PASSWORD`.
    pub fn from_dotenv() -> io::Result<(Self, PathBuf)> {
        let (path, values) = load_dotenv()?;
        let required = |key: &str| -> io::Result<String> {
            setting(&values, key).ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("missing required setting {key}"),
                )
            })
        };

        let host = required("NTRIP_IP")?;
        let port = required("NTRIP_PORT")?.parse::<u16>().map_err(|error| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("NTRIP_PORT is not a valid port: {error}"),
            )
        })?;
        let mountpoint = required("NTRIP_MOUNTPOINT")?;
        let password = required("NTRIP_PASSWORD")?;

        Ok((Self::new(host, port, mountpoint, password)?, path))
    }

    pub fn host(&self) -> &str {
        &self.host
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn mountpoint(&self) -> &str {
        &self.mountpoint
    }

    fn validate(&self) -> io::Result<()> {
        if self.host.is_empty() || self.host.chars().any(char::is_whitespace) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "NTRIP_IP must be non-empty and must not contain whitespace",
            ));
        }
        if self.port == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "NTRIP_PORT must be greater than zero",
            ));
        }
        let mountpoint_starts_with_letter = self
            .mountpoint
            .chars()
            .next()
            .is_some_and(|c| c.is_ascii_alphabetic());
        if !mountpoint_starts_with_letter
            || !self
                .mountpoint
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_'))
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "NTRIP_MOUNTPOINT must start with a letter and contain only ASCII letters, digits, '-' and '_'",
            ));
        }
        if self.password.is_empty() || self.password.chars().any(char::is_whitespace) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "NTRIP_PASSWORD must be non-empty and must not contain whitespace",
            ));
        }
        Ok(())
    }
}

/// Result of attempting to publish one RTCM frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendOutcome {
    /// The frame was written to the existing source connection.
    Sent,
    /// The connection had failed and was re-established. The stale frame that
    /// exposed the failure was intentionally dropped.
    Reconnected,
}

/// Persistent NTRIP source connection with retry and health reporting.
pub struct NtripSource {
    config: NtripSourceConfig,
    stream: TcpStream,
    frame_count: u64,
    byte_count: u64,
    report_started: Instant,
}

impl NtripSource {
    /// Connect to the caster, retrying transient network failures with capped
    /// exponential backoff. Authentication rejection is returned immediately.
    pub fn connect(config: NtripSourceConfig) -> io::Result<Self> {
        let stream = connect_with_retry(&config)?;
        Ok(Self {
            config,
            stream,
            frame_count: 0,
            byte_count: 0,
            report_started: Instant::now(),
        })
    }

    /// Publish one complete RTCM frame.
    ///
    /// If the existing connection has died, this reconnects and reports
    /// [`SendOutcome::Reconnected`] without sending the now-stale frame.
    pub fn send_rtcm(&mut self, frame: &[u8]) -> io::Result<SendOutcome> {
        match self.stream.write_all(frame) {
            Ok(()) => {
                self.frame_count += 1;
                self.byte_count += frame.len() as u64;
                self.report_health();
                Ok(SendOutcome::Sent)
            }
            Err(error) => {
                eprintln!("NTRIP source connection was lost ({error}); reconnecting");
                self.stream = connect_with_retry(&self.config)?;
                self.frame_count = 0;
                self.byte_count = 0;
                self.report_started = Instant::now();
                Ok(SendOutcome::Reconnected)
            }
        }
    }

    fn report_health(&mut self) {
        if self.report_started.elapsed() >= HEALTH_REPORT_INTERVAL {
            println!(
                "NTRIP upload healthy: {} RTCM3 frames ({} bytes)",
                self.frame_count, self.byte_count
            );
            self.frame_count = 0;
            self.byte_count = 0;
            self.report_started = Instant::now();
        }
    }
}

fn connect_with_retry(config: &NtripSourceConfig) -> io::Result<TcpStream> {
    let mut delay = INITIAL_RECONNECT_DELAY;
    loop {
        match connect_once(config) {
            Ok(stream) => return Ok(stream),
            Err(error) if error.kind() == io::ErrorKind::PermissionDenied => return Err(error),
            Err(error) => {
                eprintln!(
                    "Could not connect to NTRIP source {}:{}/{}: {error}; retrying in {} seconds",
                    config.host,
                    config.port,
                    config.mountpoint,
                    delay.as_secs()
                );
                thread::sleep(delay);
                delay = delay.saturating_mul(2).min(MAX_RECONNECT_DELAY);
            }
        }
    }
}

fn connect_once(config: &NtripSourceConfig) -> io::Result<TcpStream> {
    let addresses = (config.host.as_str(), config.port).to_socket_addrs()?;
    let mut last_error = None;
    let mut stream = None;

    for address in addresses {
        match TcpStream::connect_timeout(&address, CONNECT_TIMEOUT) {
            Ok(connected) => {
                stream = Some(connected);
                break;
            }
            Err(error) => last_error = Some(error),
        }
    }

    let mut stream = stream.ok_or_else(|| {
        last_error.unwrap_or_else(|| {
            io::Error::new(
                io::ErrorKind::AddrNotAvailable,
                "NTRIP host resolved to no addresses",
            )
        })
    })?;
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(IO_TIMEOUT))?;
    stream.set_write_timeout(Some(IO_TIMEOUT))?;
    stream.write_all(source_request(config).as_bytes())?;

    let mut status_line = String::new();
    BufReader::new(stream.try_clone()?).read_line(&mut status_line)?;
    let status_line = status_line.trim_end_matches(['\r', '\n']);
    if !status_is_success(status_line) {
        return Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            format!("NTRIP caster rejected the source connection: {status_line}"),
        ));
    }

    println!(
        "Connected to NTRIP source {}:{}/{} ({status_line})",
        config.host, config.port, config.mountpoint
    );
    Ok(stream)
}

fn source_request(config: &NtripSourceConfig) -> String {
    format!(
        "SOURCE {} /{}\r\nSource-Agent: NTRIP MAARCO/0.1\r\n\r\n",
        config.password, config.mountpoint
    )
}

fn status_is_success(status_line: &str) -> bool {
    status_line == "OK"
        || status_line == "ICY 200 OK"
        || (status_line.starts_with("HTTP/") && status_line.contains(" 200 "))
}

fn setting(file_values: &HashMap<String, String>, key: &str) -> Option<String> {
    env::var(key).ok().or_else(|| file_values.get(key).cloned())
}

fn load_dotenv() -> io::Result<(PathBuf, HashMap<String, String>)> {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut candidates = Vec::new();
    if let Ok(current_dir) = env::current_dir() {
        candidates.push(current_dir.join(".env"));
    }
    candidates.push(manifest_dir.join(".env"));
    if let Some(workspace_dir) = manifest_dir.parent() {
        candidates.push(workspace_dir.join(".env"));
    }

    for path in candidates {
        if path.is_file() {
            let contents = fs::read_to_string(&path)?;
            return Ok((path, parse_dotenv(&contents)?));
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        "could not find .env in the current, crate, or workspace directory",
    ))
}

fn parse_dotenv(contents: &str) -> io::Result<HashMap<String, String>> {
    let mut values = HashMap::new();

    for (line_index, raw_line) in contents.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let line = line.strip_prefix("export ").unwrap_or(line);
        let (key, raw_value) = line.split_once('=').ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid .env entry on line {}", line_index + 1),
            )
        })?;
        let key = key.trim();
        if key.is_empty() || !key.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid .env key on line {}", line_index + 1),
            ));
        }

        let raw_value = raw_value.trim();
        let value = if raw_value.len() >= 2
            && ((raw_value.starts_with('"') && raw_value.ends_with('"'))
                || (raw_value.starts_with('\'') && raw_value.ends_with('\'')))
        {
            &raw_value[1..raw_value.len() - 1]
        } else {
            raw_value
        };
        values.insert(key.to_string(), value.to_string());
    }

    Ok(values)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> NtripSourceConfig {
        NtripSourceConfig::new("127.0.0.1", 2101, "test-base", "test-password").unwrap()
    }

    #[test]
    fn parses_dotenv_entries() {
        let values = parse_dotenv(
            "# comment\nNTRIP_IP=127.0.0.1\nexport NTRIP_PORT=\"2101\"\nPASSWORD='secret'\n",
        )
        .unwrap();

        assert_eq!(values.get("NTRIP_IP").unwrap(), "127.0.0.1");
        assert_eq!(values.get("NTRIP_PORT").unwrap(), "2101");
        assert_eq!(values.get("PASSWORD").unwrap(), "secret");
    }

    #[test]
    fn builds_ntrip_rev1_source_request() {
        assert_eq!(
            source_request(&test_config()),
            "SOURCE test-password /test-base\r\nSource-Agent: NTRIP MAARCO/0.1\r\n\r\n"
        );
    }

    #[test]
    fn recognizes_successful_ntrip_status_lines() {
        assert!(status_is_success("OK"));
        assert!(status_is_success("ICY 200 OK"));
        assert!(status_is_success("HTTP/1.1 200 OK"));
        assert!(!status_is_success("ERROR - Bad Password"));
        assert!(!status_is_success("HTTP/1.1 401 Unauthorized"));
    }

    #[test]
    fn rejects_invalid_source_configuration() {
        assert!(NtripSourceConfig::new("", 2101, "test", "password").is_err());
        assert!(NtripSourceConfig::new("localhost", 0, "test", "password").is_err());
        assert!(NtripSourceConfig::new("localhost", 2101, "1test", "password").is_err());
        assert!(NtripSourceConfig::new("localhost", 2101, "test", "bad pass").is_err());
    }
}
