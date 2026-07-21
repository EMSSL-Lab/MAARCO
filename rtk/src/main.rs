//! LC29H base station: survey the receiver and publish RTCM3 over NTRIP.

use ntrip::source::{NtripSource, NtripSourceConfig, SendOutcome};
use rtk::WireMessage;
use rtk::port::BaseGPS;
use rtk::protocol::commands::{PQTMCfgMsgRate, PQTMCfgNmeaDp, PQTMCfgSvin, PQTMMsgName};
use rtk::protocol::pair::{PairRTCMSetOutputMode, RtcmMode};
use rtk::protocol::response::PQTMResponse;
use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

type AppResult<T> = Result<T, Box<dyn Error>>;

// Keep this base-station initialization aligned with PinPointer's sopdet GPS
// subsystem, which uses the Raspberry Pi mini-UART for the LC29H.
const DEFAULT_GPS_PORT: &str = "/dev/ttyS0";
const SVIN_MODE: u8 = 1;
const DEFAULT_SVIN_ACC_LIMIT_M: f32 = 15.0;
const DEFAULT_SVIN_DURATION_S: u32 = 150;
const CMD_TIMEOUT: Duration = Duration::from_secs(5);
const STARTUP_SETTLE: Duration = Duration::from_millis(800);
const GPS_POLL_SLEEP: Duration = Duration::from_millis(100);

struct BaseConfig {
    gps_port: PathBuf,
    svin_duration_s: u32,
    svin_accuracy_limit_m: f32,
}

impl BaseConfig {
    fn from_dotenv(path: &Path) -> AppResult<Self> {
        let values = parse_base_settings(&fs::read_to_string(path)?);
        let setting = |key: &str| env::var(key).ok().or_else(|| values.get(key).cloned());

        let gps_port = setting("GPS_PORT")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_GPS_PORT));
        let svin_duration_s = setting("SURVEY_MIN_DURATION_SECS")
            .unwrap_or_else(|| DEFAULT_SVIN_DURATION_S.to_string())
            .parse::<u32>()
            .map_err(|error| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("SURVEY_MIN_DURATION_SECS is invalid: {error}"),
                )
            })?;
        let svin_accuracy_limit_m = setting("SURVEY_ACCURACY_LIMIT_M")
            .unwrap_or_else(|| DEFAULT_SVIN_ACC_LIMIT_M.to_string())
            .parse::<f32>()
            .map_err(|error| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("SURVEY_ACCURACY_LIMIT_M is invalid: {error}"),
                )
            })?;

        if !(1..=86_400).contains(&svin_duration_s) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SURVEY_MIN_DURATION_SECS must be between 1 and 86400",
            )
            .into());
        }
        if !svin_accuracy_limit_m.is_finite() || svin_accuracy_limit_m < 0.0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SURVEY_ACCURACY_LIMIT_M must be a finite, non-negative number",
            )
            .into());
        }

        Ok(Self {
            gps_port,
            svin_duration_s,
            svin_accuracy_limit_m,
        })
    }
}

fn main() -> AppResult<()> {
    let (ntrip_config, env_path) = NtripSourceConfig::from_dotenv()?;
    println!("Loaded NTRIP configuration from {}", env_path.display());
    let base_config = BaseConfig::from_dotenv(&env_path)?;

    let mut gps = setup_and_open(&base_config)?;
    wait_for_survey(&mut gps, &base_config);

    println!(
        "Survey complete; publishing RTCM3 to {}:{}/{}",
        ntrip_config.host(),
        ntrip_config.port(),
        ntrip_config.mountpoint()
    );
    let mut ntrip = NtripSource::connect(ntrip_config)?;
    discard_stale_rtcm(&mut gps);
    run(&mut gps, &mut ntrip)
}

/// Open and initialize the LC29H.
fn setup_and_open(config: &BaseConfig) -> AppResult<BaseGPS> {
    println!("Opening GPS UART: {}", config.gps_port.display());
    let mut gps = BaseGPS::open_port(config.gps_port.clone())?;

    println!("Starting GPS reader thread");
    let _reader = gps.start();
    thread::sleep(STARTUP_SETTLE);

    configure_gps(
        &mut gps,
        config.svin_duration_s,
        config.svin_accuracy_limit_m,
    );
    Ok(gps)
}

/// Configure the receiver using PinPointer's proven base-station sequence.
/// Individual command failures are non-fatal because the receiver may already
/// contain the requested settings from a previous run.
fn configure_gps(gps: &mut BaseGPS, svin_duration_s: u32, svin_accuracy_limit_m: f32) {
    match gps.verno(CMD_TIMEOUT) {
        Ok(version) => println!(
            "LC29H version: {} (built {} {})",
            version.version, version.build_date, version.build_time
        ),
        Err(error) => eprintln!("Warning: failed to get LC29H version: {error:?}"),
    }

    println!(
        "Configuring survey-in: mode={} min_dur={}s acc_limit={:.1}m",
        SVIN_MODE, svin_duration_s, svin_accuracy_limit_m
    );
    warn_on_command_error(
        "configure survey-in",
        gps.cfg_svin_write(
            PQTMCfgSvin {
                mode: SVIN_MODE,
                min_dur: svin_duration_s,
                acc_limit_m: svin_accuracy_limit_m,
                ecef_x: 0.0,
                ecef_y: 0.0,
                ecef_z: 0.0,
            },
            CMD_TIMEOUT,
        ),
    );

    warn_on_command_error("save survey parameters", gps.save_par(CMD_TIMEOUT));

    println!("Enabling RTCM3 MSM4 output (PAIR432)");
    warn_on_command_error(
        "enable RTCM3 MSM4 output",
        gps.pair_set_rtcm_mode(
            PairRTCMSetOutputMode {
                mode: RtcmMode::Rtcm3Msm4,
            },
            CMD_TIMEOUT,
        ),
    );

    println!("Enabling $PQTMSVINSTATUS messages at 1 Hz");
    warn_on_command_error(
        "enable survey status output",
        gps.cfg_msgrate_write(
            PQTMCfgMsgRate {
                msg_name: PQTMMsgName::SvinStatus,
                rate: 1,
                msg_ver: 1,
            },
            CMD_TIMEOUT,
        ),
    );

    for sentence_type in [
        PQTMMsgName::RMC,
        PQTMMsgName::GGA,
        PQTMMsgName::GSV,
        PQTMMsgName::GSA,
        PQTMMsgName::VTG,
    ] {
        let description = format!("enable {sentence_type:?} output");
        warn_on_command_error(
            &description,
            gps.cfg_msgrate_write(
                PQTMCfgMsgRate {
                    msg_name: sentence_type,
                    rate: 1,
                    msg_ver: 1,
                },
                CMD_TIMEOUT,
            ),
        );
    }

    warn_on_command_error(
        "increase NMEA decimal precision",
        gps.cfg_nmea_dp_write(
            PQTMCfgNmeaDp {
                utc_dp: 3,
                pos_dp: 8,
                alt_dp: 3,
                dop_dp: 2,
                spd_dp: 3,
                cog_dp: 2,
            },
            CMD_TIMEOUT,
        ),
    );

    warn_on_command_error("save PQTM parameters", gps.save_par(CMD_TIMEOUT));
    warn_on_command_error(
        "save PAIR settings to NVRAM",
        gps.pair_nvram_save_setting(CMD_TIMEOUT),
    );
}

fn wait_for_survey(gps: &mut BaseGPS, config: &BaseConfig) {
    println!(
        "Waiting for survey-in ({}s / {:.1}m)",
        config.svin_duration_s, config.svin_accuracy_limit_m
    );

    loop {
        while let Some(message) = gps.try_get_gps_data() {
            if let WireMessage::PQTMMessage(PQTMResponse::SvinStatus(status)) = message {
                println!(
                    "SVIN: valid={} observations={} duration={}s accuracy={:.2}m ECEF=({:.3},{:.3},{:.3})",
                    status.valid,
                    status.observations,
                    status.config_duration,
                    status.mean_acc,
                    status.mean_x,
                    status.mean_y,
                    status.mean_z
                );
                if status.valid == 2 {
                    return;
                }
            }
        }

        // Do not retain output produced by a previously configured base while
        // this run's fresh survey is still in progress.
        while gps.try_get_rtcm_data().is_some() {}
        thread::sleep(GPS_POLL_SLEEP);
    }
}

fn run(gps: &mut BaseGPS, ntrip: &mut NtripSource) -> AppResult<()> {
    loop {
        while let Some(message) = gps.try_get_rtcm_data() {
            match ntrip.send_rtcm(&message.raw_data)? {
                SendOutcome::Sent => {}
                SendOutcome::Reconnected => {
                    discard_stale_rtcm(gps);
                    break;
                }
            }
        }

        while let Some(message) = gps.try_get_gps_data() {
            if let WireMessage::PQTMMessage(PQTMResponse::SvinStatus(status)) = message
                && status.valid != 2
            {
                eprintln!(
                    "Warning: base survey is no longer valid (valid={}, accuracy={:.2}m)",
                    status.valid, status.mean_acc
                );
            }
        }

        thread::sleep(GPS_POLL_SLEEP);
    }
}

fn discard_stale_rtcm(gps: &mut BaseGPS) {
    let mut dropped = 0_u64;
    while gps.try_get_rtcm_data().is_some() {
        dropped += 1;
    }
    if dropped > 0 {
        println!("Discarded {dropped} stale RTCM3 frames");
    }
}

fn warn_on_command_error<T>(
    action: &str,
    result: Result<T, rtk::protocol::response::ResponseError>,
) {
    match result {
        Ok(_) => println!("LC29H: {action} OK"),
        Err(error) => eprintln!("Warning: LC29H could not {action} (non-fatal): {error:?}"),
    }
}

fn parse_base_settings(contents: &str) -> HashMap<String, String> {
    contents
        .lines()
        .filter_map(|raw_line| {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                return None;
            }
            let (key, value) = line.split_once('=')?;
            if !matches!(
                key.trim(),
                "GPS_PORT" | "SURVEY_MIN_DURATION_SECS" | "SURVEY_ACCURACY_LIMIT_M"
            ) {
                return None;
            }
            let value = value.trim();
            let value = if value.len() >= 2
                && ((value.starts_with('"') && value.ends_with('"'))
                    || (value.starts_with('\'') && value.ends_with('\'')))
            {
                &value[1..value.len() - 1]
            } else {
                value
            };
            Some((key.trim().to_string(), value.to_string()))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_only_base_station_settings() {
        let values = parse_base_settings(
            "NTRIP_PASSWORD=secret\nGPS_PORT=/dev/test\nSURVEY_MIN_DURATION_SECS=300\nSURVEY_ACCURACY_LIMIT_M='1.5'\n",
        );

        assert_eq!(values.len(), 3);
        assert_eq!(values.get("GPS_PORT").unwrap(), "/dev/test");
        assert_eq!(values.get("SURVEY_MIN_DURATION_SECS").unwrap(), "300");
        assert_eq!(values.get("SURVEY_ACCURACY_LIMIT_M").unwrap(), "1.5");
    }
}
