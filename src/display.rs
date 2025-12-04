use crossterm::{
    cursor::MoveUp,
    queue,
    style::{Color, Print, ResetColor, SetForegroundColor},
    terminal::{Clear, ClearType},
};
use std::io::Write;

use nmea::{Nmea, sentences::FixType};

use crate::usb_serial::SensorData;

pub struct Display {
    prev_lines: u16,
    last_gps: Option<(Nmea, Option<String>)>,
    last_arduino: Option<SensorData>,
}

enum DisplayItem {
    Header(String, Color),
    Divider(String, Color),
    Data(String, String, Option<String>),
}

impl Display {
    pub fn new() -> Self {
        Self { 
            prev_lines: 0,
            last_gps: None,
            last_arduino: None,
        }
    }

    // Private helper to render a list of DisplayItems
    fn render<W: Write>(&mut self, stdout: &mut W) -> std::io::Result<()> {
        let mut items = Vec::new();

        // GPS Data
        if let Some((parser, gga_fix_quality)) = &self.last_gps {
            let sats = parser.satellites();
            let mut avg_snr = 0u32;
            let mut count = 0u32;
            for sat in &sats {
                if let Some(snr) = sat.snr() {
                    avg_snr += snr as u32;
                    count += 1;
                }
            }
            let avg_snr_value = if count > 0 { avg_snr / count } else { 0 };

            let fix_quality_str = match gga_fix_quality.as_deref() {
                Some("0") => "Invalid",
                Some("1") => "GPS Fix",
                Some("2") => "DGPS Fix",
                Some("3") => "PPS Fix",
                Some("4") => "Fixed RTK",
                Some("5") => "Float RTK",
                Some("6") => "Estimated (dead reckoning)",
                Some("7") => "Manual input mode",
                Some("8") => "Simulation mode",
                Some(other) => other,
                None => "N/A",
            };

            items.push(DisplayItem::Header("=== GPS DATA ===".to_string(), Color::Yellow));
            items.push(DisplayItem::Data(
                "Timestamp".to_string(),
                format!("{:?}", parser.fix_time.unwrap_or_default()),
                None,
            ));
            items.push(DisplayItem::Data(
                "Latitude".to_string(),
                format!("{:?}", parser.latitude.unwrap_or_default()),
                None,
            ));
            items.push(DisplayItem::Data(
                "Longitude".to_string(),
                format!("{:?}", parser.longitude.unwrap_or_default()),
                None,
            ));
            items.push(DisplayItem::Data(
                "Altitude".to_string(),
                format!("{:?}", parser.altitude.unwrap_or_default()),
                Some("m".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Fix Type".to_string(),
                fix_quality_str.to_string(),
                None,
            ));
            items.push(DisplayItem::Data(
                "Speed".to_string(),
                format!("{:?}", parser.speed_over_ground.unwrap_or_default()),
                Some("km/h".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Number of Satellites".to_string(),
                format!("{:?}", parser.num_of_fix_satellites.unwrap_or_default()),
                None,
            ));
            items.push(DisplayItem::Data("HDOP".to_string(), format!("{:?}", parser.hdop.unwrap_or_default()), None));
            items.push(DisplayItem::Data("VDOP".to_string(), format!("{:?}", parser.vdop.unwrap_or_default()), None));
            items.push(DisplayItem::Data("PDOP".to_string(), format!("{:?}", parser.pdop.unwrap_or_default()), None));
            items.push(DisplayItem::Data(
                "Avg SNR".to_string(),
                format!("{:?}", avg_snr_value),
                Some("db-Hz".to_string()),
            ));
            items.push(DisplayItem::Divider("=================".to_string(), Color::Yellow));
        }

        // Arduino Data
        if let Some(arduino_data) = &self.last_arduino {
            let fmt_f32 = |val: Option<f32>| -> String {
                val.map(|v| format!("{:.2}", v)).unwrap_or_else(|| "N/A".to_string())
            };
            let fmt_u32 = |val: Option<u32>| -> String {
                val.map(|v| format!("{}", v)).unwrap_or_else(|| "N/A".to_string())
            };

            items.push(DisplayItem::Header("=== Arduino Sensor Data ===".to_string(), Color::Yellow));
            items.push(DisplayItem::Data(
                "Time".to_string(),
                fmt_u32(arduino_data.time_ms),
                Some("ms".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Voltage Left".to_string(),
                fmt_f32(arduino_data.voltage_left),
                Some("V".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Current Left".to_string(),
                fmt_f32(arduino_data.current_left_a),
                Some("A".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Voltage Right".to_string(),
                fmt_f32(arduino_data.voltage_right),
                Some("V".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Current Right".to_string(),
                fmt_f32(arduino_data.current_right_a),
                Some("A".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Motor Current Left".to_string(),
                fmt_f32(arduino_data.motor_current_left),
                Some("A".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Motor Current Right".to_string(),
                fmt_f32(arduino_data.motor_current_right),
                Some("A".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Euler X".to_string(),
                fmt_f32(arduino_data.euler_x),
                Some("°".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Euler Y".to_string(),
                fmt_f32(arduino_data.euler_y),
                Some("°".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Euler Z".to_string(),
                fmt_f32(arduino_data.euler_z),
                Some("°".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Acc Lin X".to_string(),
                fmt_f32(arduino_data.acc_lin_x),
                Some("m/s²".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Acc Lin Y".to_string(),
                fmt_f32(arduino_data.acc_lin_y),
                Some("m/s²".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Acc Lin Z".to_string(),
                fmt_f32(arduino_data.acc_lin_z),
                Some("m/s²".to_string()),
            ));
            items.push(DisplayItem::Data(
                "Sonar".to_string(),
                fmt_f32(arduino_data.sonar_mm),
                Some("mm".to_string()),
            ));
            items.push(DisplayItem::Data(
                "ToF".to_string(),
                fmt_f32(arduino_data.tof_mm),
                Some("mm".to_string()),
            ));
            items.push(DisplayItem::Data(
                "RPM Left".to_string(),
                fmt_f32(arduino_data.rpm_left),
                None,
            ));
            items.push(DisplayItem::Data(
                "RPM Right".to_string(),
                fmt_f32(arduino_data.rpm_right),
                None,
            ));
            items.push(DisplayItem::Data(
                "Rotations Left".to_string(),
                fmt_f32(arduino_data.rotations_left),
                None,
            ));
            items.push(DisplayItem::Data(
                "Rotations Right".to_string(),
                fmt_f32(arduino_data.rotations_right),
                None,
            ));
        }

        let max_len = items
            .iter()
            .filter_map(|item| {
                if let DisplayItem::Data(label, _, _) = item {
                    Some(label.len())
                } else {
                    None
                }
            })
            .max()
            .unwrap_or(0);

        let line_count = items.len() as u16;

        if self.prev_lines > 0 {
            queue!(
                stdout,
                MoveUp(self.prev_lines),
                Clear(ClearType::FromCursorDown)
            )?;
        }

        for item in items {
            match item {
                DisplayItem::Header(s, c) | DisplayItem::Divider(s, c) => {
                    queue!(
                        stdout,
                        SetForegroundColor(c),
                        Print(s),
                        ResetColor,
                        Print("\n")
                    )?;
                }
                DisplayItem::Data(label, value, opt_unit) => {
                    let padded = format!("{:<width$}: ", label, width = max_len);
                    queue!(stdout, Print(padded))?;
                    queue!(
                        stdout,
                        SetForegroundColor(Color::Green),
                        Print(value),
                        ResetColor
                    )?;
                    if let Some(unit) = opt_unit {
                        queue!(
                            stdout,
                            Print(" "),
                            SetForegroundColor(Color::Red),
                            Print(unit),
                            ResetColor
                        )?;
                    }
                    queue!(stdout, Print("\n"))?;
                }
            }
        }

        stdout.flush()?;

        self.prev_lines = line_count;

        Ok(())
    }

    pub fn update_gps<W: Write>(&mut self, stdout: &mut W, parser: &Nmea, gga_fix_quality: Option<String>) -> std::io::Result<()> {
        self.last_gps = Some((parser.clone(), gga_fix_quality));
        self.render(stdout)
    }

    pub fn update_arduino<W: Write>(
        &mut self,
        stdout: &mut W,
        arduino_data: &SensorData,
    ) -> std::io::Result<()> {
        self.last_arduino = Some(arduino_data.clone());
        self.render(stdout)
    }
}
