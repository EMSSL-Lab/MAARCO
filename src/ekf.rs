use nalgebra::{Matrix3, UnitQuaternion, Vector3, SMatrix, SVector};

// Define aliases for the 9-dimensional state [pos, vel, ori_error]
type Matrix9 = SMatrix<f64, 9, 9>;
type Vector9 = SVector<f64, 9>;

pub struct EKF {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub q: UnitQuaternion<f64>,
    pub b_a: Vector3<f64>,
    pub b_g: Vector3<f64>,

    pub p_cov: Matrix9,
    pub q_proc: Matrix9,

    // ADD THESE FIELDS HERE:
    pub q_pos: f64,
    pub q_vel: f64,
    pub q_ori: f64,
    pub r_rtk_fixed: f64,
    pub r_rtk_float: f64,

    // Add this to maintain filter state
    pub filtered_accel: Vector3<f64>,
    pub cutoff_freq: f64, // Cutoff frequency in Hz (e.g., 5.0)
    // pub lpf_alpha: f64, // Smoothing factor: 0.0 (total lag) to 1.0 (no filter)
}

impl EKF {

    // Inside impl EKF in src/ekf.rs
    pub fn get_yaw_degrees(&self) -> f64 {
        // 1. Convert Quaternion to Euler angles (Roll, Pitch, Yaw)
        let euler = self.q.euler_angles();
        
        // 2. The third value is Yaw (in radians)
        let yaw_rad = euler.2;
        
        // 3. Convert to degrees and normalize to 0-360 if preferred
        let mut deg = yaw_rad.to_degrees();
        
        // Optional: Standardize turtle-friendly rotation
        if deg < 0.0 {
            deg += 360.0;
        }
        
        deg
    }
    
    pub fn new(q_pos: f64, q_vel: f64, q_ori: f64, r_fix: f64,    // <--- This must be named 'r_fix'
        r_float: f64, cutoff_freq: f64) -> Self {
        let p_cov = Matrix9::identity() * 1.0;
        
        let mut q_proc = Matrix9::identity();
        q_proc.fixed_view_mut::<3, 3>(0, 0).copy_from(&(Matrix3::identity() * q_pos));
        q_proc.fixed_view_mut::<3, 3>(3, 3).copy_from(&(Matrix3::identity() * q_vel));
        q_proc.fixed_view_mut::<3, 3>(6, 6).copy_from(&(Matrix3::identity() * q_ori));

        Self {
            p: Vector3::zeros(),
            v: Vector3::zeros(),
            q: UnitQuaternion::identity(),
            b_a: Vector3::zeros(),
            b_g: Vector3::zeros(),
            p_cov,
            q_proc,

            // ADD THESE FIELDS HERE:
            q_pos,
            q_vel,
            q_ori,
            r_rtk_fixed : r_fix,
            r_rtk_float : r_float,
            // Initialize filter
            filtered_accel: Vector3::zeros(),
            cutoff_freq, // Tuning: lower = smoother/more lag, higher = noisier/faster
        }
    }

    pub fn update_tuning(&mut self, q_pos: f64, q_vel: f64, q_ori: f64, r_fix: f64, r_float: f64) {
        self.q_pos = q_pos;
        self.q_vel = q_vel;
        self.q_ori = q_ori;
        self.r_rtk_fixed = r_fix;
        self.r_rtk_float = r_float;
        // Optionally update your Process Noise matrix (Q) here if it's pre-calculated
    }
    
    pub fn predict(&mut self, raw_accel: Vector3<f64>, gyro: Vector3<f64>, dt: f64) {
        if dt <= 0.0 { return; }

        // 1. CALCULATE ALPHA BASED ON DT
        // alpha = dt / (tau + dt) where tau = 1 / (2 * pi * f_c)
        let tau = 1.0 / (2.0 * std::f64::consts::PI * self.cutoff_freq);
        let alpha = dt / (tau + dt);
        
        // Clamp alpha to [0, 1] just in case dt is huge
        let alpha = alpha.clamp(0.0, 1.0);

        // 2. APPLY FILTER
        if self.filtered_accel.norm() == 0.0 {
            self.filtered_accel = raw_accel;
        } else {
            self.filtered_accel = (self.filtered_accel * (1.0 - alpha)) + (raw_accel * alpha);
        }

        let w = gyro - self.b_g;
        let dq = UnitQuaternion::from_scaled_axis(w * dt);
        self.q = self.q * dq;
        let rot = self.q.to_rotation_matrix();

        // let acc_corrected = accel - self.b_a;
        let acc_corrected = self.filtered_accel - self.b_a;

        // Gravity subtraction (adjust if your IMU Z-axis is inverted)
        let acc_world = (rot * acc_corrected) - Vector3::new(0.0, 0.0, 9.81);
        
        self.p += self.v * dt + 0.5 * acc_world * dt * dt;
        self.v += acc_world * dt;

        let mut f = Matrix9::identity();
        f.fixed_view_mut::<3, 3>(0, 3).copy_from(&(Matrix3::identity() * dt));

        let skew_accel = Matrix3::new(
            0.0, -acc_corrected.z, acc_corrected.y,
            acc_corrected.z, 0.0, -acc_corrected.x,
            -acc_corrected.y, acc_corrected.x, 0.0
        );
        f.fixed_view_mut::<3, 3>(3, 6).copy_from(&(rot.matrix() * -skew_accel * dt));

        self.p_cov = (f * self.p_cov * f.transpose()) + (self.q_proc * dt);
    }

    pub fn update_gps(&mut self, gps_pos: Vector3<f64>, r_value: f64) {
        let mut h = SMatrix::<f64, 3, 9>::zeros();
        h.fixed_view_mut::<3, 3>(0, 0).copy_from(&Matrix3::identity());

        let y = gps_pos - self.p;
        let r = Matrix3::identity() * r_value;
        let s = (h * self.p_cov * h.transpose()) + r;

        if let Some(s_inv) = s.try_inverse() {
            let k = self.p_cov * h.transpose() * s_inv;
            let update: Vector9 = k * y; 
            
            self.p += update.fixed_rows::<3>(0);
            self.v += update.fixed_rows::<3>(3);
            
            let q_corr = UnitQuaternion::from_scaled_axis(update.fixed_rows::<3>(6));
            self.q = self.q * q_corr;

            self.p_cov = (Matrix9::identity() - (k * h)) * self.p_cov;
        }
    }
}