use std::time::Instant;

pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
    base_throttle: f64,
}

pub struct MotorCommands {
    pub left_pwm: u64,
    pub base_throttle: u64,
}

impl PDController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            kp,
            kd,
            last_error: 0.0,
            last_time: Instant::now(),
            base_throttle: 0.0,
        }
    }

    pub fn reset_base_throttle(&mut self) {
        self.base_throttle = 0.0;
    }

    pub fn reset(&mut self) {
        self.last_error = 0.0;
        self.last_time = Instant::now();
    }

    /// Holds left motor at target_rpm using feedback from measured rpm_left.
    /// base_speed (1500–2000) is mirrored around 1500 to get the left baseline.
    /// Trim accumulates to compensate for persistent terrain slip.
    pub fn compute_motor_commands(
        &mut self,
        current_rpm: f64, // rpm of the left motor 
        target_rpm: f64,  // User prompted
        base_throttle: u64,  // Base throttle user prompted
    ) -> MotorCommands {
        let now = Instant::now();
        let dt = now.duration_since(self.last_time).as_secs_f64();
        self.last_time = now;

        // Positive error: too slow → need more power → lower PWM (faster reverse)
        let error = target_rpm - current_rpm;

        let derivative = if dt > 0.02 {  // only compute if at least 20ms has passed
        (error - self.last_error) / dt
        } else {
        0.0
        };
        self.last_error = error;

        let pd_output = (self.kp * error) + (self.kd * derivative);
        self.base_throttle += pd_output;
        self.base_throttle = self.base_throttle.clamp(1500.0,2000.0);

        // Mirror base_throttle around 1500 to get left motor baseline
       

        // right base throttle should be between 1500 and 2000
        let right_pwm: u64 = self.base_throttle as u64;
        
        // left base throttle should be mirrored, between 1000 and 1500
        let left_pwm = (3000.0 - self.base_throttle).clamp(1000.0, 1499.0) as u64;

        MotorCommands {
            left_pwm: left_pwm,
            base_throttle: right_pwm,
        }
    }
}
