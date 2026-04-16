use nalgebra::{Vector3, UnitQuaternion};

pub struct EKF {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub q: UnitQuaternion<f64>,
    pub b_a: Vector3<f64>,
    pub b_g: Vector3<f64>,
}

impl EKF {
    pub fn new() -> Self {
        Self {
            p: Vector3::zeros(),
            v: Vector3::zeros(),
            q: UnitQuaternion::identity(),
            b_a: Vector3::zeros(),
            b_g: Vector3::zeros(),
        }
    }

    pub fn predict(&mut self, accel: Vector3<f64>, gyro: Vector3<f64>, dt: f64) {
        if dt <= 0.0 { return; }

        // 1. Gyro Bias correction
        let w = gyro - self.b_g;
        
        // 2. Update Orientation
        let dq = UnitQuaternion::from_scaled_axis(w * dt);
        self.q = self.q * dq;

        // 3. Rotate Accelerometer to World Frame (Subtracting Accel Bias)
        let rot = self.q.to_rotation_matrix();
        let acc_world_no_g = rot * (accel - self.b_a); 
        
        // 4. Subtract Gravity (assuming Z is UP, g = 9.81)
        let g = Vector3::new(0.0, 0.0, 9.81); 
        let acc_final = acc_world_no_g - g;

        // 5. Integrate Position and Velocity
        self.p += self.v * dt + 0.5 * acc_final * dt * dt;
        self.v += acc_final * dt;
    }

    pub fn update_gps(&mut self, gps_pos: Vector3<f64>, dt_gps: f64, alpha_p: f64, alpha_v: f64) 
    {
        let pos_error = gps_pos - self.p;
        
        // Apply position correction
        self.p += pos_error * alpha_p;

        // Apply velocity correction (nudge)
        if dt_gps > 0.0 {
            self.v += (pos_error / dt_gps) * alpha_v;
        }
    }
}