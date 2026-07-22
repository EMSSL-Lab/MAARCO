// src/ntrip.rs
use base64::{Engine as _, engine::general_purpose};
use std::io::{self, BufRead, BufReader, ErrorKind, Read, Write};
use std::net::TcpStream;
use std::sync::mpsc::Sender;
use std::thread;
use std::time::Duration;

pub fn connect_rtk2go_ntrip(tx: Sender<Vec<u8>>, mountpoint: &str) {
    loop {  // ← outer reconnect loop
        println!("[NTRIP] Connecting to rtk2go.com:2101 mountpoint /{}", mountpoint);

        let stream = match TcpStream::connect(("rtk2go.com", 2101)) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[NTRIP] TCP connect failed: {:?}. Retrying in 10s", e);
                thread::sleep(Duration::from_secs(10));
                continue;  // retry from top of outer loop
            }
        };

        let auth = format!("{}:{}", "hoppingturtles@proton.me", "none");
        let auth_b64 = general_purpose::STANDARD.encode(auth);
        let request = format!(
            "GET /{} HTTP/1.0\r\n\
            User-Agent: NTRIP RustClient\r\n\
            Accept: */*\r\n\
            Authorization: Basic {}\r\n\
            Connection: close\r\n\r\n",
            mountpoint, auth_b64
        );

        {
            // Scope the clone so we don't hold two mutable refs to stream
            let mut write_stream = stream.try_clone().expect("stream clone failed");
            if let Err(e) = write_stream.write_all(request.as_bytes()) {
                eprintln!("[NTRIP] Failed to send request: {:?}. Retrying in 10s", e);
                thread::sleep(Duration::from_secs(10));
                continue;
            }
        }

        let mut reader = BufReader::new(&stream);
        let mut status_line = String::new();
        if let Err(e) = reader.read_line(&mut status_line) {
            eprintln!("[NTRIP] Failed to read response: {:?}. Retrying in 10s", e);
            thread::sleep(Duration::from_secs(10));
            continue;
        }

        if !status_line.contains("200 OK") && !status_line.contains("ICY 200 OK") {
            eprintln!("[NTRIP] Server rejected connection: {}. Retrying in 30s", status_line.trim());
            thread::sleep(Duration::from_secs(30));
            continue;  // ← was `return`, which killed the thread permanently
        }

        println!("[NTRIP] Connected. Streaming RTCM corrections.");

        // Drain the HTTP headers before binary data starts
        loop {
            let mut header_line = String::new();
            match reader.read_line(&mut header_line) {
                Ok(_) if header_line == "\r\n" => break,  // blank line = end of headers
                Ok(0) => break,
                _ => {}
            }
        }

        // Read RTCM binary data
        let mut buf = [0u8; 4096];
        loop {
            match reader.get_mut().read(&mut buf) {
                Ok(0) => {
                    eprintln!("[NTRIP] Stream closed by server. Reconnecting in 5s.");
                    thread::sleep(Duration::from_secs(5));
                    break;  // break inner loop → retry outer loop
                }
                Ok(n) => {
                    if tx.send(buf[..n].to_vec()).is_err() {
                        // main.rs receiver dropped — process is shutting down
                        eprintln!("[NTRIP] Main thread receiver gone. NTRIP thread exiting.");
                        return;  // exit thread entirely, don't reconnect
                    }
                }
                Err(e) => {
                    eprintln!("[NTRIP] Read error: {:?}. Reconnecting in 5s.", e);
                    thread::sleep(Duration::from_secs(5));
                    break;  // break inner loop → retry outer loop
                }
            }
        }
    } // outer reconnect loop
}