/// This file is just for testing the rtkbase crate.
///
use rtkbase;
use rtkbase::port::BaseGPS;
use rtkbase::protocol::response::WireMessage;
use std::{path::PathBuf};

fn test_reading_sentences() {
    let mut rtk = BaseGPS::open_port(PathBuf::from("/dev/ttyUSB0")).unwrap();
    let _ = rtk.start();
    println!("Opened RTK GPS port successfully.");
    let timeout = std::time::Duration::from_secs(2);
    
    let mut count = 0;
    while count < 5 {
        if let Some(msg) = rtk.get_gps_data(timeout) {
            match &msg {
                WireMessage::PQTMMessage(resp) => {
                    println!("Received PQTM Response: {:?}", resp);
                }
                WireMessage::PairMessage(pair) => {
                    println!("Received PAIR Message: {:?}", pair);
                }
            }
        }
        count += 1;
    }

    let ver_no = rtk.verno(timeout).unwrap();
    println!("Module Version: {:?}", ver_no.version);
    
    let rtcm_output_mode = rtk.pair_get_rtcm_mode(timeout).unwrap();
    println!("Current RTCM Output Mode: {:?}", rtcm_output_mode);
}

fn main() {
    test_reading_sentences();
}
