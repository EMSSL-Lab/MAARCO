use nalgebra::{Matrix4, Vector4, Matrix3x4, Vector3, Matrix3, Vector2};

pub struct RoverState {
    pub x_m: f64,
    pub y_m: f64,
    pub velocity_ms: f64,
    pub yaw_rad: f64,
}

pub struct DistanceTracker {
    pub start_lat: Option<f64>,
    pub start_lon: Option<f64>,
    pub state: Vector4<f64>,         // [x, y, v, yaw]
    pub covariance: Matrix4<f64>,    // P
    process_noise: Matrix4<f64>, // Q (Trust in physics)
    measurement_noise: Matrix3<f64>, // R (Trust in GPS)
}

impl DistanceTracker {
    pub fn new() -> Self {
        Self {
            start_lat: None,
            start_lon: None,
            state: Vector4::zeros(),
            covariance: Matrix4::identity() * 0.5,
            // Q: We set the velocity noise high (2.0) because your accel is noisy
            process_noise: Matrix4::from_diagonal(&Vector4::new(0.01, 0.01, 2.0, 0.01)),
            // process_noise: Matrix4::from_diagonal(&Vector4::new(0.05, 0.05, 2.0, 0.01)),
            // R: We set GPS position noise high (5.0) to ignore the "bouncing"
            // Trust GPS significantly more because of RTK
            measurement_noise: Matrix3::from_diagonal(&Vector3::new(0.05, 0.05, 0.1)),
            // measurement_noise: Matrix3::from_diagonal(&Vector3::new(5.0, 5.0, 0.5)),
        }
    }

    pub fn set_origin(&mut self, lat: f64, lon: f64) {
        self.start_lat = Some(lat);
        self.start_lon = Some(lon);
        self.state = Vector4::zeros();
        self.covariance = Matrix4::identity() * 0.1;
    }

    /// PREDICT Step (Call @ 10Hz from Arduino data)
    pub fn predict_imu(&mut self, raw_accel_y: f64, yaw_deg: f64, dt: f64) {
        // 1. Deadband: Ignore noise if the robot isn't really moving
        let accel = if raw_accel_y.abs() < 0.15 { 0.0 } else { raw_accel_y };
        
        let yaw_rad = yaw_deg.to_radians();
        let v = self.state[2];

        // Kinematic equations
        self.state[0] += v * yaw_rad.cos() * dt;
        self.state[1] += v * yaw_rad.sin() * dt;
        self.state[2] += accel * dt;
        self.state[3] = yaw_rad;

        // Jacobian of motion model
        let mut f = Matrix4::identity();
        f[(0, 2)] = yaw_rad.cos() * dt;
        f[(1, 2)] = yaw_rad.sin() * dt;

        self.covariance = f * self.covariance * f.transpose() + self.process_noise;
    }

    /// UPDATE Step (Call @ 1Hz from GPS data)
    pub fn update_gps(&mut self, lat: f64, lon: f64, gps_v_ms: f64, fix_str: &str) -> RoverState {
    // pub fn update_gps(&mut self, lat: f64, lon: f64, gps_v_ms: f64) -> RoverState {
        let s_lat = self.start_lat.unwrap_or(lat);
        let s_lon = self.start_lon.unwrap_or(lon);
        let lon_scale = s_lat.to_radians().cos();

        // Measurement Z: [GPS_X, GPS_Y, GPS_Velocity]
        let z = Vector3::new(
            (lon - s_lon) * 111319.9 * lon_scale,
            (lat - s_lat) * 111319.9,
            gps_v_ms
        );

        // H Matrix: Mapping state [x, y, v, yaw] to measurement [x, y, v]
        let h = Matrix3x4::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        );

        // Mapping your display strings to Measurement Uncertainty (R)
        let r_val = match fix_str {
            "Fixed RTK" => 0.0001, // 1cm - Extreme trust
            "Float RTK" => 0.01,   // 10cm - High trust
            "DGPS Fix"  => 0.5,    // Sub-meter trust
            "GPS Fix"   => 2.0,    // Standard meter-level trust
            _           => 5.0,    // Invalid or poor signal
        };
        self.measurement_noise = Matrix3::from_diagonal(&Vector3::new(r_val, r_val, 0.1));

        // Kalman math
        let s = h * self.covariance * h.transpose() + self.measurement_noise;
        let k = self.covariance * h.transpose() * s.try_inverse().unwrap();
        let y = z - (h * self.state);
        
        self.state += k * y;
        self.covariance = (Matrix4::identity() - k * h) * self.covariance;

        RoverState {
            x_m: self.state[0],
            y_m: self.state[1],
            velocity_ms: self.state[2],
            yaw_rad: self.state[3],
        }
    }
}