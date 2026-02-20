use std::time::Instant;

const SCREW_PITCH_M: f64 = 0.05;
const SCREW_EFFICIENCY: f64 = 0.75;
const METERS_PER_ROTATION: f64 = SCREW_PITCH_M * SCREW_EFFICIENCY;
const ARRIVAL_THRESHOLD_M: f64 = 0.2;

pub struct DistanceController {
    dr_lat: Option<f64>,
    dr_lon: Option<f64>,
    dr_north_m: f64,
    dr_east_m: f64,
    start_lat: Option<f64>,
    start_lon: Option<f64>,
    last_imu_time: Instant,
}

pub struct ImuSample {
    pub heading_deg: f64,
    pub rpm_left: f64,
    pub rpm_right: f64,
}

pub struct DistanceOutput {
    pub arrived: bool,
    pub dist_traveled_m: f64,
}

impl DistanceController {
    pub fn new() -> Self {
        Self {
            dr_lat: None,
            dr_lon: None,
            dr_north_m: 0.0,
            dr_east_m: 0.0,
            start_lat: None,
            start_lon: None,
            last_imu_time: Instant::now(),
        }
    }

    /// Called at 1 Hz when a new GPS epoch is confirmed.
    /// Snaps dead-reckoning origin to the fresh fix, zeroing accumulated drift.
    pub fn update_gps(&mut self, lat: f64, lon: f64) {
        if self.start_lat.is_none() {
            self.start_lat = Some(lat);
            self.start_lon = Some(lon);
            println!("[DistCtrl] Start position locked: ({:.7}, {:.7})", lat, lon);
        }
        self.dr_lat = Some(lat);
        self.dr_lon = Some(lon);
        self.dr_north_m = 0.0;
        self.dr_east_m = 0.0;
    }

    /// Called at 10 Hz on every Arduino sensor packet.
    /// Integrates screw RPM + heading into dead-reckoned position.
    pub fn update_imu(&mut self, sample: ImuSample) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_imu_time).as_secs_f64();
        self.last_imu_time = now;

        if self.dr_lat.is_none() || dt <= 0.0 || dt > 1.0 {
            return; // end function if no lat data or unreasonable dt 
        }

        let avg_rpm = (sample.rpm_left + sample.rpm_right) / 2.0;
        let rotations_per_sec = avg_rpm / 60.0;
        let forward_displacement = rotations_per_sec * METERS_PER_ROTATION * dt; // in meters

        // Trig to convert heading vector into north/east components. Heading is 0° at north, increasing clockwise.
        let heading_rad = sample.heading_deg.to_radians();
        self.dr_north_m += forward_displacement * heading_rad.cos();
        self.dr_east_m  += forward_displacement * heading_rad.sin();
    }

    fn estimated_position(&self) -> Option<(f64, f64)> {
        let base_lat = self.dr_lat?;
        let base_lon = self.dr_lon?;
        let lat_m_per_deg = 111_111.0_f64;
        let lon_m_per_deg = 111_111.0 * base_lat.to_radians().cos();

        // convert north/east displacements from meters back into degrees and add to base position
        Some((
            base_lat + self.dr_north_m / lat_m_per_deg,
            base_lon + self.dr_east_m / lon_m_per_deg,
        ))
    }

    // Returns the displacement from the start position in meters, using the Haversine function to 
    // calculate the distance between the start position (degrees) and the current estimated position (degrees)
    fn distance_from_start(&self) -> f64 {
        let (start_lat, start_lon) = match (self.start_lat, self.start_lon) {
            (Some(la), Some(lo)) => (la, lo),
            _ => return 0.0,
        };
        let (est_lat, est_lon) = match self.estimated_position() {
            Some(pos) => pos,
            None => return 0.0,
        };

        // Calculate the distane between the start position (in degrees) and the current estimated position (in degrees) using the Haversine formula
        // The Haversine function output is the distance in meters between the two positions
        haversine(start_lat, start_lon, est_lat, est_lon)
    }

    /// Checks whether the rover has traveled target_dist meters.
    /// No motor logic here — just position tracking and arrival detection.
    pub fn check(&self, target_dist: f64) -> DistanceOutput {
        let dist_traveled = self.distance_from_start();
        DistanceOutput {
            arrived: dist_traveled >= target_dist - ARRIVAL_THRESHOLD_M,
            dist_traveled_m: dist_traveled,
        }
    }
}

// Haversine formula to calculate the great-circle distance between two points on the Earth given their latitudes and longitudes in degrees.
// Returns distance in meters.
fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = 6_371_000.0_f64;
    let d_lat = (lat2 - lat1).to_radians();
    let d_lon = (lon2 - lon1).to_radians();
    let a = (d_lat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
    r * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
}