/// This file is just for testing the rtkbase crate.
///
use rtkbase;
use rtkbase::port::BaseGPS;
use rtkbase::protocol::PqtmOutput;
use std::sync::atomic::AtomicBool;
use std::thread::sleep;
use std::{path::PathBuf, sync::mpsc};

fn test_reading_sentences() {
    let (tx, rx): (mpsc::Sender<PqtmOutput>, mpsc::Receiver<PqtmOutput>) = mpsc::channel();

    let mut rtk = BaseGPS::open_port(PathBuf::from("/dev/ttyUSB0"));

    if let Ok(ref mut rtk) = rtk {
        let start_handle = rtk.start();
        println!("Opened RTK GPS port successfully.");
        rtk.get_pqtm_data();
    }
    sleep(std::time::Duration::from_secs(100));
}

fn main() {
    test_reading_sentences();
}
