use std::time::Instant;


pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
}

pub struct CrabMotorCommands {
    pub front_pwm_us: u64,
    pub rear_pwm_us: u64,
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

    pub fn update_gains(&mut self, new_kp: f64, new_kd: f64) {
        self.kp = new_kp;
        self.kd = new_kd;
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

pub fn compute_crab_commands(
        &mut self,
        current_yaw: f64,
        target_yaw: f64,
        base_speed: u64, // The speed at which you want to crab sideways
    ) -> CrabMotorCommands {
        let yaw_correction = self.yaw_calculate(current_yaw, target_yaw);

        // We apply the correction to the Rear motor to keep the Front as our anchor.
        // If yaw_correction is positive, it increases Rear speed relative to Front.
        // You may need to flip the sign (+/-) depending on your screw thread handiness.
        let front_pwm = (base_speed as f64).clamp(1000.0, 2000.0) as u64;

        let rear_pwm = (base_speed as f64 + yaw_correction).clamp(1000.0, 2000.0) as u64;

        CrabMotorCommands {
            front_pwm_us: front_pwm,
            rear_pwm_us: rear_pwm,
        }
    }
}
