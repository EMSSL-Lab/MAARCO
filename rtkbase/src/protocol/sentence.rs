use crate::protocol::commands::{PQTMCfgMsgRate, PQTMCfgSvin};
use crate::protocol::helpers::{StatusField, parse_status_and_rest, wrap_sentence};
use crate::protocol::response::{PQTMEpe, PQTMResponse, PQTMSvinStatus, PQTMVerNo, ParseError};

use super::commands::PQTMCommand;
use super::helpers::unwrap_sentence;

pub trait Serialize {
    fn to_sentence(&self) -> String;
}

pub trait Deserialize: Sized {
    type Error;
    fn from_sentence(s: &str) -> Result<Self, Self::Error>;
}

impl Serialize for PQTMCommand {
    fn to_sentence(&self) -> String {
        match self {
            PQTMCommand::CfgSvinWrite(cfg) => wrap_sentence(&cfg.to_fields()),
            PQTMCommand::CfgSvinRead => wrap_sentence("PQTMCFGSVIN,R"),
            PQTMCommand::SavePar => wrap_sentence("PQTMSAVEPAR"),
            PQTMCommand::RestorePar => wrap_sentence("PQTMRESTOREPAR"),
            PQTMCommand::Verno => wrap_sentence("PQTMVERNO"),
            PQTMCommand::CfgMsgRateWrite(cfg) => wrap_sentence(&cfg.to_fields()),
            PQTMCommand::CfgMsgRateRead(cfg_get) => wrap_sentence(&cfg_get.to_fields()),
        }
    }
}

impl Deserialize for PQTMResponse {
    type Error = ParseError;

    fn from_sentence(s: &str) -> Result<PQTMResponse, Self::Error> {
        let payload = unwrap_sentence(s)?;
        let mut parts = payload.split(",");
        let header = parts.next().ok_or(ParseError::NoSentence)?;

        match header {
            "PQTMSAVEPAR" => match parse_status_and_rest(parts)? {
                StatusField::Ok(_) => Ok(PQTMResponse::SaveParOk),
                StatusField::Err(e) => Ok(PQTMResponse::SaveParError(e)),
            },
            "PQTMRESTOREPAR" => match parse_status_and_rest(parts)? {
                StatusField::Ok(_) => Ok(PQTMResponse::RestoreParOk),
                StatusField::Err(e) => Ok(PQTMResponse::RestoreParError(e)),
            },
            "PQTMVERNO" => match parse_status_and_rest(parts)? {
                StatusField::Ok(mut rest) => {
                    Ok(PQTMResponse::Verno(PQTMVerNo::from_fields(&mut rest)?))
                }
                StatusField::Err(e) => Ok(PQTMResponse::VernoError(e)),
            },
            "PQTMCFGSVIN" => {
                match parse_status_and_rest(parts)? {
                    StatusField::Ok(mut rest) => {
                        if rest.clone().next().is_none() {
                            // Write response: OK only:
                            Ok(PQTMResponse::CfgSvinWriteOk)
                        } else {
                            // Read response:
                            Ok(PQTMResponse::CfgSvinReadOk(PQTMCfgSvin::from_fields(
                                &mut rest,
                            )?))
                        }
                    }
                    StatusField::Err(e) => Ok(PQTMResponse::CfgSvinError(e)),
                }
            }
            "PQTMCFGMSGRATE" => {
                match parse_status_and_rest(parts)? {
                    StatusField::Ok(mut rest) => {
                        if rest.clone().next().is_none() {
                            // Write response: OK only:
                            Ok(PQTMResponse::CfgMsgRateWriteOk)
                        } else {
                            // Read response:
                            Ok(PQTMResponse::CfgMsgRateReadOk(PQTMCfgMsgRate::from_fields(
                                &mut rest,
                            )?))
                        }
                    }
                    StatusField::Err(e) => Ok(PQTMResponse::CfgMsgRateError(e)),
                }
            }
            "PQTMEPE" => Ok(PQTMResponse::Epe(PQTMEpe::from_fields(&mut parts)?)),
            "PQTMSVINSTATUS" => Ok(PQTMResponse::SvinStatus(PQTMSvinStatus::from_fields(
                &mut parts,
            )?)),
            _ => Err(ParseError::ParsingError("Unknown sentence header")),
        }
    }
}
