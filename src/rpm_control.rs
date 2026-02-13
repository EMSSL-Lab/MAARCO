use std::time::Instant;


pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
}

pub struct MotorCommands {
    pub left_pwm_us: u64,   // Pulse width in microseconds
    // pub right_pwm_us: u64,  // Pulse width in microseconds
}

impl PDController{
    pub fn new(kp: f64, kd: f64) -> Self{
        Self {
            kp,
            kd,
            last_error: 0.0,
            last_time: Instant::now(),
        }
    }

pub fn rpm_calculate(&mut self, current_rpm: f64, target_rpm: f64 ) -> f64 {
let now = Instant::now();
//Calculate time passed in seconds
let dt = now.duration_since(self.last_time).as_secs_f64();

// // Handle angle wrap around
 let error = target_rpm - current_rpm;

//Derivative
let mut derivative = 0.0;
if dt > 0.0 {
    derivative = (error - self.last_error) / dt;
}

let output = (self.kp * error) + (self.kd * derivative);

// Update state
self.last_error = error;
self.last_time = now;

// Return function output
 output

}

pub fn compute_motor_commands(
    &mut self,
    current_rpm: f64,
    target_rpm: f64,
    base_speed_left: u64, // Now used as a constant
    // base_speed_right: u64, // Now used as a constant
) -> MotorCommands {
    let rpm_correction = self.rpm_calculate(current_rpm, target_rpm);

    // Left motor stays at the constant speed provided
    // let right_pwm = base_speed_right;

    // Right motor handles all the adjustment
    // If rpm_correction is positive (need to go faster), 
    // adding it makes the right motor faster.
    let left_pwm = (base_speed_left as f64 - rpm_correction).clamp(1000.0, 1500.0) as u64;

    MotorCommands {
        left_pwm_us: left_pwm as u64,
        // right_pwm_us: right_pwm as u64,
    }
}
}
