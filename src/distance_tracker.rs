// This script calculates the distance traveled using GPS and IMU data

// distance_tracker.rs
//
// Sensor-fused distance tracker for a screw-drive rover.
//
// Update cadence:
//   update_gps()  — called at ~1 Hz when a new GPS epoch arrives
//   update_imu()  — called at ~10 Hz on every Arduino sensor packet
//
// Fusion strategy (between GPS pulses):
//   1. RPM odometry   : avg screw RPM → meters traveled in dt
//   2. Kinematics     : v*dt + ½*a*dt²  (a = |acc_lin| from IMU)
//   3. Weighted average of the two → added to running total
//
// On each GPS pulse:
//   • Total distance is *reset* to haversine(start → current fix)   (drift correction)
//   • Velocity seed is reset to GPS-reported speed (km/h → m/s)

use std::time::Instant;

// ── Screw-drive constants ──────────────────────────────────────────────────────
const SCREW_PITCH_M: f64 = 0.067;        // meters per revolution (physical pitch)
const SCREW_EFFICIENCY: f64 = 0.75;     // terrain efficiency factor
const METERS_PER_ROTATION: f64 = SCREW_PITCH_M * SCREW_EFFICIENCY;

// ── Sensor-fusion weights (must sum to 1.0) ───────────────────────────────────
const RPM_WEIGHT: f64 = 0.6;
const KINEMATICS_WEIGHT: f64 = 0.4;

// ── Arrival threshold ─────────────────────────────────────────────────────────
const ARRIVAL_THRESHOLD_M: f64 = 0.2;

// ─────────────────────────────────────────────────────────────────────────────

pub struct DistanceTracker {
    // Start position (set once on first GPS fix, never changes)
    start_lat: Option<f64>,
    start_lon: Option<f64>,

    // Running distance estimate (meters from start).
    // Reset to haversine value on each GPS pulse.
    pub distance_traveled_m: f64,

    // Velocity used in kinematic equations (m/s).
    // Seeded from GPS speed; integrated forward between pulses.
    velocity_ms: f64,

    // Timestamp of the last update_imu() call (for dt calculation)
    last_imu_time: Option<Instant>,
}

/// Returned by update_imu() on every 10 Hz tick.
pub struct DistanceOutput {
    /// True when distance_traveled_m >= target - ARRIVAL_THRESHOLD_M
    pub arrived: bool,
    /// Current fused distance estimate from start (meters)
    pub dist_traveled_m: f64,
}

impl DistanceTracker {
    pub fn new() -> Self {
        Self {
            start_lat: None,
            start_lon: None,
            distance_traveled_m: 0.0,
            velocity_ms: 0.0,
            last_imu_time: None,
        }
    }

// New target reset function
// Called when the rover is re-tasked after arrival
pub fn reset_for_new_target(&mut self) {
    self.start_lat = None;
    self.start_lon = None;
    self.distance_traveled_m = 0.0;
    self.velocity_ms = 0.0;
    self.last_imu_time = None;
    println!("[DistTrack] Resetting for new target.");
}



    // ── GPS update (called ~1 Hz) ─────────────────────────────────────────────
    //
    // On the very first call: locks the start position and seeds velocity.
    // On subsequent calls:
    //   • Resets distance_traveled_m to haversine(start → this fix)
    //   • Resets velocity_ms to GPS-reported speed (converted km/h → m/s)
    pub fn update_gps(&mut self, lat: f64, lon: f64, speed_kmh: f64) {
        // Lock start position once
        if self.start_lat.is_none() {
            self.start_lat = Some(lat);
            self.start_lon = Some(lon);
            self.distance_traveled_m = 0.0;
            self.velocity_ms = speed_kmh / 3.6;
            println!(
                "[DistTrack] Start locked: ({:.7}, {:.7}), v0 = {:.3} m/s",
                lat,
                lon,
                self.velocity_ms
            );
        } else {
            // Reset fused total to authoritative GPS haversine distance
            let start_lat = self.start_lat.unwrap();
            let start_lon = self.start_lon.unwrap();
            self.distance_traveled_m = haversine(start_lat, start_lon, lat, lon);

            // Re-seed velocity from GPS (drift correction), also convert speed to m/s
            self.velocity_ms = speed_kmh / 3.6;

            println!(
                "[DistTrack] GPS reset → dist = {:.3} m, v = {:.3} m/s",
                self.distance_traveled_m, self.velocity_ms
            );
        }
        
    }

    // ── IMU & RPM update (called ~10 Hz) ─────────────────────────────────────
    //
    // Computes Δdistance via two methods, takes a weighted average,
    // and accumulates into distance_traveled_m.
    // Also integrates velocity forward for the next kinematic step.
    //
    // Returns DistanceOutput so main.rs can check arrival and act.
    pub fn update_imu(
        &mut self,
        rpm_left: f64,
        rpm_right: f64,
        acc_x: f64,
        acc_y: f64,
        acc_z: f64,
        target_dist_m: f64,
    ) -> DistanceOutput {
        // ── Compute dt ──────────────────────────────────────────────────────
        let now = Instant::now();
        let dt = match self.last_imu_time {
            Some(t) => {
                let elapsed = now.duration_since(t).as_secs_f64();
                // Sanity-clamp: ignore dt > 1 s (e.g. first tick after long pause)
                if elapsed <= 0.0 || elapsed > 1.0 {
                    self.last_imu_time = Some(now);
                    return self.output(target_dist_m);
                }
                elapsed
            }
            None => {
                // Very first IMU tick — no dt yet, just record time
                self.last_imu_time = Some(now);
                return self.output(target_dist_m);
            }
        };
        self.last_imu_time = Some(now);

        // Don't integrate until we have a GPS start fix
        if self.start_lat.is_none() {
            return self.output(target_dist_m);
        }

        // ── 1. RPM odometry ──────────────────────────────────────────────────
        //   average screw RPM → rotations/s → meters in dt
        let avg_rpm = (rpm_left + rpm_right) / 2.0;
        let delta_rpm = (avg_rpm / 60.0) * METERS_PER_ROTATION * dt;

        // ── 2. Kinematics (constant-acceleration) ────────────────────────────
        //   acceleration in the y is the forward facing direction of the robot 
        let accel = acc_y;

        let delta_kin = self.velocity_ms * dt + 0.5 * accel * dt * dt;
        
        // ── 3. Sensor fusion — weighted average ──────────────────────────────
        let delta_fused = RPM_WEIGHT * delta_rpm + KINEMATICS_WEIGHT * delta_kin;

        // ── 4. Accumulate ─────────────────────────────────────────────────────
        self.distance_traveled_m += delta_fused; 

        // ── 5. Integrate velocity for next step ──────────────────────────────
        
        self.velocity_ms = (self.velocity_ms + accel * dt);

        self.output(target_dist_m)
    }

    // ── Internal helper ───────────────────────────────────────────────────────
    fn output(&self, target_dist_m: f64) -> DistanceOutput {
        DistanceOutput {
            arrived: self.distance_traveled_m >= target_dist_m - ARRIVAL_THRESHOLD_M,
            dist_traveled_m: self.distance_traveled_m,
        }
    }
}


// ── Haversine formula ─────────────────────────────────────────────────────────
// Returns the great-circle distance between two WGS-84 coordinates in meters.
fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0; // Earth radius in meters
    let d_lat = (lat2 - lat1).to_radians();
    let d_lon = (lon2 - lon1).to_radians();
    let a = (d_lat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
    R * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
}