use std::time::Instant;

// --- PHYSICAL SCREW CONSTANTS ---
// Measure these in the same units (e.g., millimeters)
const SCREW_PITCH: f64 = 50.0;    // The distance between thread peaks
const SCREW_DIAMETER: f64 = 100.0; // The outer diameter of the screw
// ---------------------------------

pub struct PDControllerCrab {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
    creep_factor: f64, 
}

/// We only return the Right PWM here because, per your design, 
/// the Left PWM is handled by your separate RPM Controller.
pub struct CrabMotorCommands {
    pub right_pwm_us: u64,
}

impl PDControllerCrab {
    /// Constructor now automatically calculates creep based on hard-coded constants
    pub fn new(kp: f64, kd: f64) -> Self {
        // Calculate the creep factor based on the screw geometry
        // Creep Factor = Pitch / (pi * Diameter)
        let creep_factor = SCREW_PITCH / (std::f64::consts::PI * SCREW_DIAMETER);
        
        Self {
            kp,
            kd,
            last_error: 0.0,
            last_time: Instant::now(),
            creep_factor,
        }
    }

    /// Identical PD calculation logic to maintain consistency.
    pub fn calculate_correction(&mut self, current_yaw: f64, target_yaw: f64) -> f64 {
        let now = Instant::now();
        let dt = now.duration_since(self.last_time).as_secs_f64();

        let mut error = target_yaw - current_yaw;
        
        // Wrap around logic for 0-360 degree range
        if error > 180.0 { error -= 360.0; }
        else if error < -180.0 { error += 360.0; }

        let mut derivative = 0.0;
        if dt > 0.0 {
            derivative = (error - self.last_error) / dt;
        }

        let output = (self.kp * error) + (self.kd * derivative);

        self.last_error = error;
        self.last_time = now;

        output
    }

    /// Computes commands specifically for Crab Motion with creep compensation.
    /// Note: This ONLY calculates the Right motor PWM.
    pub fn compute_crab_commands(
        &mut self,
        current_yaw: f64,
        target_yaw: f64,
        base_speed_right: u64, // Comes from Left Motor RPM Controller
    ) -> CrabMotorCommands {
        let yaw_correction = self.calculate_correction(current_yaw, target_yaw);

        // 1. Calculate the compensation effort to cancel out longitudinal creep.
        // We use the hard-coded creep_factor here.
        let compensation = base_speed_right as f64 * self.creep_factor;

        // 2. Combine Yaw correction with Creep compensation.                
        // Right PWM = Base + Yaw Adjustment - Physical Creep Adjustment
        let right_pwm = (base_speed_right as f64 + yaw_correction - compensation)
            .clamp(1000.0, 2000.0) as u64;

        CrabMotorCommands {
            right_pwm_us: right_pwm,
        }
    }
}