// use rppal::gpio::Gpio;
// use std::time::Duration;
// use std::error::Error;

// pub fn set_motor_pwm(pin_num: i32, pulse_width_us: i32) -> Result<(), Box<dyn Error>> {
//     let gpio = Gpio::new()?;
//     // Map the i32 pin to u16 for rppal
//     let mut pin = gpio.get(pin_num as u8)?.into_output();

//     let period = Duration::from_millis(20); // 50Hz
//     let pwm = Duration::from_micros(pulse_width_us as u64);

//     println!("Setting Pin {} to {}us", pin_num, pulse_width_us);
//     pin.set_pwm(period, pwm)?;

//     Ok(())
// }

use rppal::gpio::{Gpio, OutputPin};
use std::time::Duration;
use std::error::Error;

// This now returns the OutputPin so main can keep it alive
pub fn get_motor_pin(pin_num: u8) -> Result<OutputPin, Box<dyn Error>> {
    let gpio = Gpio::new()?;
    let pin = gpio.get(pin_num)?.into_output();
    Ok(pin)
}

pub fn update_pwm(pin: &mut OutputPin, pulse_width_us: u64) -> Result<(), Box<dyn Error>> {
    let period = Duration::from_millis(20); // 50Hz
    let pwm = Duration::from_micros(pulse_width_us);
    
    pin.set_pwm(period, pwm)?;
    Ok(())
}