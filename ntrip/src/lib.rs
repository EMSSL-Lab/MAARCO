pub mod source;

use base64::{Engine as _, engine::general_purpose};
use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::TcpStream;
use std::sync::mpsc::Sender;
use std::thread::{self};
use std::time::Duration;

pub enum NTRIPMessage {
    Rtcm(Vec<u8>),
    ConnectionFailed,
    AuthenticationFailed,
    MountpointNotFound,
    ReadError,
}

impl std::fmt::Display for NTRIPMessage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NTRIPMessage::Rtcm(_) => write!(f, "Receiving RTCM data"),
            NTRIPMessage::ConnectionFailed => write!(f, "Connection failed"),
            NTRIPMessage::AuthenticationFailed => write!(f, "Authentication failed"),
            NTRIPMessage::MountpointNotFound => write!(f, "Mountpoint not found"),
            NTRIPMessage::ReadError => write!(f, "Read error"),
        }
    }
}

pub fn connect_rtk2go_ntrip(tx: Sender<NTRIPMessage>, mountpoint: &str) -> () {
    // Connect to the RTK2GO NTRIP caster
    let maybe_stream = TcpStream::connect(("rtk2go.com", 2101));
    let mut stream = maybe_stream.expect("Could not connect to RTK2GO!");

    // Prepare the auth header
    let auth = format!("{}:{}", "hoppingturtles@proton.me", "none");
    let auth_b64 = general_purpose::STANDARD.encode(auth);

    // Construct NTRIP request header
    let request = format!(
        "GET /{} HTTP/1.0\r\n\
        User-Agent: NTRIP RustClient\r\n\
        Accept: */*\r\n\
        Authorization: Basic {}\r\n\
        Connection: close\r\n\r\n",
        mountpoint, auth_b64
    );

    if let Err(e) = stream.write_all(request.as_bytes()) {
        eprintln!("Failed to send NTRIP request: {:?}", e);
        tx.send(NTRIPMessage::ConnectionFailed)
            .expect("Could not send connection error to main");
        return;
    }

    // Check if our server accepted the connection:
    let mut reader = BufReader::new(&stream);
    let mut line = String::new();

    // Read status line
    if let Err(e) = reader.read_line(&mut line) {
        eprintln!("Failed to read NTRIP response: {:?}", e);
        return;
    }

    if !line.contains("200 OK") && !line.contains("ICY 200 OK") {
        // Some servers use ICY
        eprintln!("NTRIP connection failed: {}", line.trim());
        tx.send(NTRIPMessage::AuthenticationFailed)
            .expect("Could not send auth error to main");
        return;
    }

    // When the mountpoint is not found, the server responds with a SOURCETABLE
    if line.contains("SOURCETABLE") {
        tx.send(NTRIPMessage::MountpointNotFound)
            .expect("Could not send mtpt error to main");
        return;
    }

    // Read and print incoming RTCM data chunks
    let mut buf = [0u8; 4096];
    loop {
        let n = stream.read(&mut buf);

        match n {
            Ok(n) => {
                if n == 0 {
                    eprintln!("NTRIP stream closed");
                }
                // Here you can process or forward the data. We'll just print length for this example:
                // println!("Received {} bytes of RTCM data", n);

                let _ = tx
                    .send(NTRIPMessage::Rtcm(buf[0..n].to_vec()))
                    .map_err(io::Error::other);
            }

            Err(_) => {
                tx.send(NTRIPMessage::ReadError)
                    .expect("Could not send read error to main");
                thread::sleep(Duration::from_secs(5));
            }
        }
    }
}
