pub mod dispatcher;
pub mod i2c_port;
mod methods;
pub mod parsing;
pub mod port;
pub mod protocol;
pub mod rtcm_parser;

pub use protocol::nmea::{GgaData, GpsFixQuality, GsvConstellation, GsvData};
pub use protocol::response::WireMessage;
