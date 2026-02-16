use std::time::Instant;

pub struct PDController {
    pub kp: f64,
    pub kd: f64,
    last_error: f64,
    last_time: Instant,
    start_lat: Option<f64>,
    start_lon: Option<f64>,
}

pub struct DistanceCommands {
    pub base_speed: u64,
    pub arrived: bool,
}

impl PDController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            kp,
            kd,
            last_error: 0.0,
            last_time: Instant::now(),
            start_lat: None,
            start_lon: None,
        }
    }

    /// Calculates distance in meters between two GPS coordinates using Haversine
    fn calculate_haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        let r = 6371000.0; // Earth radius in meters
        let d_lat = (lat2 - lat1).to_radians();
        let d_lon = (lon2 - lon1).to_radians();
        let a = (d_lat / 2.0).sin().powi(2)
            + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        r * c
    }

    pub fn distance_calculate(&mut self, current_lat: f64, current_lon: f64, target_dist: f64) -> (f64, f64) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_time).as_secs_f64();

        // Set starting point on the first valid GPS fix
        if self.start_lat.is_none() {
            self.start_lat = Some(current_lat);
            self.start_lon = Some(current_lon);
        }

        let dist_traveled = Self::calculate_haversine(
            self.start_lat.unwrap(),
            self.start_lon.unwrap(),
            current_lat,
            current_lon,
        );

        let error = target_dist - dist_traveled;

        let mut derivative = 0.0;
        if dt > 0.0 {
            derivative = (error - self.last_error) / dt;
        }

        let output = (self.kp * error) + (self.kd * derivative);

        self.last_error = error;
        self.last_time = now;

        (output, dist_traveled)
    }

    pub fn compute_motor_commands(
        &mut self,
        current_lat: f64,
        current_lon: f64,
        target_dist: f64,
        base_throttle: u64,
    ) -> DistanceCommands {
        let (correction, dist_traveled) = self.distance_calculate(current_lat, current_lon, target_dist);

        // If error is very small, we've arrived
        if (target_dist - dist_traveled).abs() < 0.2 {
            return DistanceCommands { base_speed: 1500, arrived: true };
        }

        // Apply correction to the base throttle
        let final_speed = (base_throttle as f64 + correction).clamp(1500.0, 2000.0) as u64;

        DistanceCommands {
            base_speed: final_speed,
            arrived: false,
        }
    }
}