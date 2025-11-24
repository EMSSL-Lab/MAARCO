use nmea::{self, Nmea};

pub fn build_parser() -> Nmea {
    let parser = Nmea::default();
    parser
}

/// Modifies the Nmea parser in place by parsing the given input sentence
pub fn parse_nmea_sentence(parser: &mut Nmea, input: &str) -> () {
    let _ = parser.parse(input);
}
