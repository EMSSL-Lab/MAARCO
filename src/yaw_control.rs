use std::time::Instant;


pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
}

pub struct MotorCommands {
    pub right_pwm_us: u64,  // Pulse width in microseconds
    pub left_pwm_us: u64,   // Left motor PWM (currently unused, set to base speed)
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

pub fn yaw_calculate(&mut self, current_yaw: f64, target_yaw: f64 ) -> f64 {
let now = Instant::now();
//Calculate time passed in seconds
let dt = now.duration_since(self.last_time).as_secs_f64();

// Handle angle wrap around
let mut error = target_yaw - current_yaw;
if error > 180.0 {
    error -= 360.0;
}
else if error < -180.0 {
    error += 360.0;
}

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
    current_yaw: f64,
    target_yaw: f64,
    base_speed_right: u64, // Now used as a constant
    base_speed_left: u64
) -> MotorCommands {
    let yaw_correction = self.yaw_calculate(current_yaw, target_yaw);

    // Right motor handles all the adjustment
    // If yaw_correction is positive (need to turn right), 
    // subtracting it makes the right motor slower.
    let right_pwm = (base_speed_right as f64 + yaw_correction).clamp(1500.0, 2000.0) as u64;
    let left_pwm = base_speed_left; // Left motor runs at constant speed
    MotorCommands {
        right_pwm_us: right_pwm as u64,
        left_pwm_us: left_pwm as u64,
        
    }
}
}
