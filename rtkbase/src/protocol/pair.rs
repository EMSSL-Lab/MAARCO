pub enum Pair {
    ACK(PairACK), // PAIR001

    RtcmSetOutputMode(PairRTCMSetOutputMode), // PAIR432
    RtcmGetOutputMode(PairRTCMGetOutputMode), // PAIR433

    RtcmSetOutputAntPnt(PairRTCMSetOutputAntPnt), // PAIR434
    RtcmGetOutputAntPnt(PairRTCMGetOutputAntPnt), // PAIR435

    RtcmSetOutputEphemeris(PairRTCMSetOutputEphemeris), // PAIR436
    RtcmGetOutputEphemeris(PairRTCMGetOutputEphemeris), // PAIR437

    RequestAiding(PairRequestAiding), // PAIR010

    SystemWakeUp, // PAIR012
}

#[derive(Debug, Clone)]
pub struct PairACK {
    pub command_id: u16,
    pub result: AckResult,
}

#[derive(Debug, Clone)]
pub enum AckResult {
    Success = 0,
    Processing = 1,
    Failed = 2,
    NotSupported = 3,
    Error = 4,
    Busy = 5,
}

#[derive(Debug, Clone)]
pub struct PairRTCMSetOutputMode {
    pub mode: RtcmMode,
}

#[derive(Debug, Clone)]
pub enum RtcmMode {
    Disable = -1,
    Rtcm3Msm4 = 0,
    Rtcm3Msm7 = 1,
}

type PairRTCMGetOutputMode = PairRTCMSetOutputMode;

/// Enable/disable outputting stationary RTK reference station ARP (message type 1005).
#[derive(Debug, Clone)]
pub struct PairRTCMSetOutputAntPnt {
    pub ant_pnt: RtcmAntPnt,
}

type PairRTCMGetOutputAntPnt = PairRTCMSetOutputAntPnt;

#[derive(Debug, Clone)]
pub enum RtcmAntPnt {
    Disable = 0,
    Enable = 1,
}

#[derive(Debug, Clone)]
pub struct PairRTCMSetOutputEphemeris {
    pub ephemeris: RtcmEphemeris,
}

type PairRTCMGetOutputEphemeris = PairRTCMSetOutputEphemeris;

#[derive(Debug, Clone)]
pub enum RtcmEphemeris {
    Disable = 0,
    Enable = 1,
}

#[derive(Debug, Clone)]
pub struct PairRequestAiding {
    /// Type of data to be updated
    pub aiding_type: AidingType,
    /// Type of required GNSS data
    pub gnss_system: GnssSystem,
    /// Week number (accommodating rollover)
    pub week_number: u16,
    /// Time of week in seconds
    pub time_of_week: u64,
}

#[derive(Debug, Clone)]
pub enum AidingType {
    EpoData = 0,
    Time = 1,
    Location = 2,
}

#[derive(Debug, Clone)]
pub enum GnssSystem {
    Gps = 0,
    Glonass = 1,
    Galileo = 2,
    BeiDou = 3,
    Qzss = 4,
}
