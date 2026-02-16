# // use rppal::gpio::Gpio;
# // use std::thread;
# // use std::time::Duration;
# // use std::error::Error;

# // fn main() -> Result<(), Box<dyn Error>> {
# //     // 1. SETUP
# //     // Using Pin 18 (Standard for PWM on Pi)
# //     let gpio = Gpio::new()?;
# //     let mut pin = gpio.get(18)?.into_output();

# //     // 2. DEFINE VALUES
# //     // Period is 20ms (standard 50Hz frequency)
# //     let period = Duration::from_millis(20);
    
# //     // Define your specific command values
# //     let val_1750 = Duration::from_micros(1750); // Forward (approx 50%)
# //     let val_1500 = Duration::from_micros(1500); // Neutral / Stop

# //     println!("Starting Sequence...");

# //     // --- STEP 1: 1750 for 5 Seconds ---
# //     println!("Command: 1750 (Forward)");
# //     pin.set_pwm(period, val_1750)?;
# //     thread::sleep(Duration::from_secs(5));

# //     // --- STEP 2: 1500 for 5 Seconds ---
# //     println!("Command: 1500 (Neutral)");
# //     pin.set_pwm(period, val_1500)?;
# //     thread::sleep(Duration::from_secs(5));

# //     // --- STEP 3: 1750 for 5 Seconds ---
# //     println!("Command: 1750 (Forward)");
# //     pin.set_pwm(period, val_1750)?;
# //     thread::sleep(Duration::from_secs(5));

# //     // --- DONE ---
# //     println!("Sequence Complete. Setting to Neutral and exiting.");
    
# //     // Safety: Before the program quits, we set it back to 1500 (Neutral).
# //     // If we didn't do this, the motor might keep spinning or the ESC might beep 
# //     // indicating 'signal lost' when the program closes.
# //     pin.set_pwm(period, val_1500)?;

# //     Ok(())
# // }