use std::time::Instant;

pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
    trim: f64,
}

pub struct MotorCommands {
    pub left_pwm_us: u64,
}

impl PDController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            kp,
            kd,
            last_error: 0.0,
            last_time: Instant::now(),
            trim: 0.0,
        }
    }

    pub fn reset_trim(&mut self) {
        self.trim = 0.0;
    }

    /// Holds left motor at target_rpm using feedback from measured rpm_left.
    /// base_speed (1500–2000) is mirrored around 1500 to get the left baseline.
    /// Trim accumulates to compensate for persistent terrain slip.
    pub fn compute_motor_commands(
        &mut self,
        current_rpm: f64,
        target_rpm: f64,
        base_speed: u64,
    ) -> MotorCommands {
        let now = Instant::now();
        let dt = now.duration_since(self.last_time).as_secs_f64();
        self.last_time = now;

        // Positive error: too slow → need more power → lower PWM (faster reverse)
        let error = target_rpm - current_rpm;

        let derivative = if dt > 0.0 {
            (error - self.last_error) / dt
        } else {
            0.0
        };
        self.last_error = error;

        let pd_output = (self.kp * error) + (self.kd * derivative);
        self.trim += pd_output * dt;
        self.trim = self.trim.clamp(-200.0, 200.0);

        // Mirror base_speed around 1500 to get left motor baseline
        let offset = base_speed as f64 - 1500.0;
        let mirrored_base = 1500.0 - offset;

        let left_pwm = (mirrored_base - self.trim)
            .clamp(1000.0, 1499.0) as u64;

        MotorCommands {
            left_pwm_us: left_pwm,
        }
    }
}
