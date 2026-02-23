// use std::time::Instant;

// pub struct PDController {
//     pub kp: f64,
//     pub kd: f64,
//     last_error: f64,
//     last_time: Instant,
//     start_lat: Option<f64>,
//     start_lon: Option<f64>,
// }

// pub struct DistanceCommands {
//     pub base_speed: u64,
//     pub arrived: bool,
// }

// impl PDController {
//     pub fn new(kp: f64, kd: f64) -> Self {
//         Self {
//             kp,
//             kd,
//             last_error: 0.0,
//             last_time: Instant::now(),
//             start_lat: None,
//             start_lon: None,
//         }
//     }

//     /// Calculates distance in meters between two GPS coordinates using Haversine
//     fn calculate_haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
//         let r = 6371000.0; // Earth radius in meters
//         let d_lat = (lat2 - lat1).to_radians();
//         let d_lon = (lon2 - lon1).to_radians();
//         let a = (d_lat / 2.0).sin().powi(2)
//             + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
//         let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
//         r * c
//     }

//     pub fn distance_calculate(&mut self, current_lat: f64, current_lon: f64, target_dist: f64) -> (f64, f64) {
//         let now = Instant::now();
//         let dt = now.duration_since(self.last_time).as_secs_f64();

//         // Set starting point on the first valid GPS fix
//         if self.start_lat.is_none() {
//             self.start_lat = Some(current_lat);
//             self.start_lon = Some(current_lon);
//         }

//         let dist_traveled = Self::calculate_haversine(
//             self.start_lat.unwrap(),
//             self.start_lon.unwrap(),
//             current_lat,
//             current_lon,
//         );

//         let error = target_dist - dist_traveled;

//         let mut derivative = 0.0;
//         if dt > 0.0 {
//             derivative = (error - self.last_error) / dt;
//         }

//         let output = (self.kp * error) + (self.kd * derivative);

//         self.last_error = error;
//         self.last_time = now;

//         (output, dist_traveled)
//     }

//     pub fn compute_motor_commands(
//         &mut self,
//         current_lat: f64,
//         current_lon: f64,
//         target_dist: f64,
//         base_throttle: u64,
//     ) -> DistanceCommands {
//         let (correction, dist_traveled) = self.distance_calculate(current_lat, current_lon, target_dist);

//         // If error is very small, we've arrived
//         if (target_dist - dist_traveled).abs() < 0.2 {
//             return DistanceCommands { base_speed: 1500, arrived: true };
//         }

//         // Apply correction to the base throttle
//         let final_speed = (base_throttle as f64 + correction).clamp(1500.0, 2000.0) as u64;

//         DistanceCommands {
//             base_speed: final_speed,
//             arrived: false,
//         }
//     }
// }

use std::time::Instant;

const SCREW_PITCH_M: f64 = 0.05;
const SCREW_EFFICIENCY: f64 = 0.75;
const METERS_PER_ROTATION: f64 = SCREW_PITCH_M * SCREW_EFFICIENCY;
const ARRIVAL_THRESHOLD_M: f64 = 0.2;
const GPS_TIMEOUT_SEC: f64 = 3.0; // Safety: stop if GPS hangs too long

pub struct DistanceController {
    start_lat: Option<f64>,
    start_lon: Option<f64>,
    dr_lat: Option<f64>,
    dr_lon: Option<f64>,
    dr_north_m: f64,
    dr_east_m: f64,
    
    // Benchmark specific fields
    pub total_odo_m: f64,      // Accumulated screw distance
    last_imu_time: Instant,
    last_gps_time: Instant,
}

impl DistanceController {
    pub fn new() -> Self {
        Self {
            start_lat: None,
            start_lon: None,
            dr_lat: None,
            dr_lon: None,
            dr_north_m: 0.0,
            dr_east_m: 0.0,
            total_odo_m: 0.0,
            last_imu_time: Instant::now(),
            last_gps_time: Instant::now(),
        }
    }

    pub fn update_gps(&mut self, lat: f64, lon: f64) {
        let now = Instant::now();
        if self.start_lat.is_none() {
            self.start_lat = Some(lat);  // assign lat and lon coords
            self.start_lon = Some(lon);
            println!("[BENCHMARK] Origin Locked.");
        }

        // Calculate the "Snap Error" for your logs before resetting
        let prev_est_dist = self.distance_from_start_euclidean();
        
        self.dr_lat = Some(lat);    // dead reckoning for gps
        self.dr_lon = Some(lon);
        self.dr_north_m = 0.0;      // dead reckoning for imu
        self.dr_east_m = 0.0;
        self.last_gps_time = now;
        
        // This log helps you see how much your IMU drifted over the 1s gap
        println!("[BENCHMARK] GPS Sync. Current Odo: {:.2}m", self.total_odo_m);
    }

    pub fn update_imu(&mut self, sample: ImuSample) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_imu_time).as_secs_f64();
        self.last_imu_time = now;

        // Safety 1: Don't move if we haven't initialized GPS
        if self.dr_lat.is_none() { return; }

        // Safety 2: Relaxed dt for 1Hz GPS. 
        // We only return if the gap is massive (e.g., system freeze)
        if dt <= 0.0 || dt > 2.0 { return; }
        
        // calculate distance moved each dt
        let rotations_per_sec = (sample.rpm_left + sample.rpm_right) / 2.0 / 60.0;
        let delta_dist = rotations_per_sec * METERS_PER_ROTATION * dt;

        // Track raw physical movement
        self.total_odo_m += delta_dist;

        // Track directional movement (Dead Reckoning)
        let heading_rad = sample.heading_deg.to_radians();
        self.dr_north_m += delta_dist * heading_rad.cos();
        self.dr_east_m  += delta_dist * heading_rad.sin();
    }

    /// Benchmark-friendly distance: uses pure displacement since the last GPS "snap"
    /// added to the distance already covered by the GPS.
    fn distance_from_start_euclidean(&self) -> f64 {
        let (s_lat, s_lon) = match (self.start_lat, self.start_lon) {
            (Some(la), Some(lo)) => (la, lo),
            _ => return 0.0,
        };
        let (c_lat, c_lon) = match (self.dr_lat, self.dr_lon) {
            (Some(la), Some(lo)) => (la, lo),
            _ => return 0.0,
        };

        // 1. Distance between start point and last known GPS point
        let base_gps_dist = haversine(s_lat, s_lon, c_lat, c_lon);

        // 2. Local distance added by IMU since that last GPS point
        let local_dr_dist = (self.dr_north_m.powi(2) + self.dr_east_m.powi(2)).sqrt();

        base_gps_dist + local_dr_dist
    }

    pub fn check(&self, target_dist: f64) -> DistanceOutput {
        // SAFETY: If GPS stops updating for 3 seconds, stop the rover for safety.
        let gps_age = Instant::now().duration_since(self.last_gps_time).as_secs_f64();
        if gps_age > GPS_TIMEOUT_SEC {
            return DistanceOutput { arrived: true, dist_traveled_m: self.total_odo_m };
        }

        // For straight-line benchmarks, total_odo_m is usually "cleaner",
        // but distance_from_start_euclidean is more "honest" about final position.
        let current_dist = self.distance_from_start_euclidean();

        DistanceOutput {
            arrived: current_dist >= target_dist - ARRIVAL_THRESHOLD_M,
            dist_traveled_m: current_dist,
        }
    }
}