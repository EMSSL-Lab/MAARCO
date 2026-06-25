use std::time::Instant;


pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
}

pub struct MotorCommands {
    pub right_pwm_us: u64,  // Pulse width in microseconds
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

pub fn reset(&mut self) {
    self.last_error = 0.0;
    self.last_time = Instant::now();
}


pub fn yaw_calculate(&mut self, current_yaw: f64, target_yaw: f64 ) -> f64 {
let now = Instant::now();
//Calculate time passed in seconds
let dt = now.duration_since(self.last_time).as_secs_f64();

// Handle angle wrap around
let mut error = target_yaw - current_yaw;
while error > 180.0 {
    error -= 360.0;
}
while error < -180.0 {
    error += 360.0;
}

//Derivative
let mut derivative = 0.0;
if dt > 0.0 {
    derivative = (error - self.last_error) / dt;
}

let output = -1.0*(self.kp * error) + -1.0*(self.kd * derivative);

// Update state
self.last_error = error;
self.last_time = now;

// Return function output
 output

}

pub fn compute_motor_commands(
    &mut self,
    current_yaw: f64, // From IMU 
    target_yaw: f64,  // User declared target yaw
    base_speed_right: u64, // User declared base pwm 
) -> MotorCommands {
    let yaw_correction = self.yaw_calculate(current_yaw, target_yaw);
    // If the base throttle is 1500, then right motor should not turn
    if base_speed_right == 1500 {
        return MotorCommands {
            right_pwm_us: 1500,
        };
    }

    let right_pwm = (base_speed_right as f64 + yaw_correction).clamp(1500.0, 2000.0) as u64;

    MotorCommands {
        right_pwm_us: right_pwm as u64,
    }
}
}
