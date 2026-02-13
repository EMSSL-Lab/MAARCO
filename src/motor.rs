use rppal::gpio::{Gpio, OutputPin};
use std::time::Duration;
use std::error::Error;

// In motor.rs - Ensure you have two pins ready
pub fn get_motor_pins(left_pin: u8, right_pin: u8) -> Result<(OutputPin, OutputPin), Box<dyn Error>> {
    let gpio = Gpio::new()?;
    let p_left = gpio.get(left_pin)?.into_output();
    let p_right = gpio.get(right_pin)?.into_output();
    Ok((p_left, p_right))
}

// pub fn update_pwm(pin: &mut OutputPin, pulse_width_us: u64) -> Result<(), Box<dyn Error>> {
//     let period = Duration::from_millis(20); // 50Hz
//     let pwm = Duration::from_micros(pulse_width_us);
    
//     pin.set_pwm(period, pwm)?;
//     Ok(())
// }

pub fn update_pwm(
    pin_l: &mut OutputPin, 
    pin_r: &mut OutputPin, 
    pulse_l: i64, 
    pulse_r: i64
) -> Result<(), Box<dyn Error>> {
    let period = Duration::from_millis(20); // 50Hz

    // We use .abs() so that your -1600 becomes a valid 1600us pulse
    let pwm_l = Duration::from_micros(pulse_l.abs() as u64);
    let pwm_r = Duration::from_micros(pulse_r.abs() as u64);

    pin_l.set_pwm(period, pwm_l)?;
    pin_r.set_pwm(period, pwm_r)?;

    Ok(())
}