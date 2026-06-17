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
            base_throttle: 1500.0, // Start at neutral throttle
        }
    }

    pub fn reset_base_throttle(&mut self) {
        self.base_throttle = 1500.0;
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
    rpm_l: f64,
    target_rpm: f64,
) -> MotorCommands {
    let now = Instant::now();
    let dt = now.duration_since(self.last_time).as_secs_f64();
    self.last_time = now;

    // Explicit zero-RPM short-circuit before PD math runs
    if target_rpm <= 0.0 {
    self.base_throttle = 1500.0; // drives left_pwm to 1499 → neutral
    self.last_error = 0.0;
    return MotorCommands {
        left_pwm: 1500,       // true neutral for left
        base_throttle: 1500,  // true neutral for right
        };
    }


    let error = target_rpm - rpm_l;

    let derivative = if dt > 0.02 {
        (error - self.last_error) / dt
    } else {
        0.0
    };
    self.last_error = error;

    let pd_output = (self.kp * error) + (self.kd * derivative);

    // Determine if we need to reflect the pd_output around 1500 or not   
    let next_throttle = self.base_throttle + pd_output;
    if next_throttle >= 1500.0 && next_throttle <= 2000.0 {
        self.base_throttle = next_throttle.clamp(1500.0, 2000.0);
    } else {
        self.base_throttle = (next_throttle + 2.0*(1500.0 - next_throttle)).clamp(1500.0,2000.0); // Reflect about 1500 pwm value
    }

    let right_pwm: u64 = self.base_throttle as u64;
    let left_pwm = (3000.0 - self.base_throttle).clamp(1000.0, 1500.0) as u64;

    MotorCommands {
        left_pwm,
        base_throttle: right_pwm,
    }
}
}
