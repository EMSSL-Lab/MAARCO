// distance_tracker.rs

pub struct RoverState {
    pub x_m: f64,          // East-West meters from origin
    pub y_m: f64,          // North-South meters from origin
    pub velocity_ms: f64,
}

pub struct DistanceTracker {
    pub start_lat: Option<f64>,
    pub start_lon: Option<f64>,
}

impl DistanceTracker {
    pub fn new() -> Self {
        Self {
            start_lat: None,
            start_lon: None,
        }
    }

    /// Resets the origin. Use this when Python sends "SET_ORIGIN"
    pub fn set_origin(&mut self, lat: f64, lon: f64) {
        self.start_lat = Some(lat);
        self.start_lon = Some(lon);
        println!("[GPS] Origin Set: {:.7}, {:.7}", lat, lon);
    }

    pub fn update_gps(&mut self, lat: f64, lon: f64, speed_kmh: f64) -> Option<RoverState> {
        // We can't calculate XY until we have an origin
        let s_lat = self.start_lat?;
        let s_lon = self.start_lon?;

        // 1. Calculate Y (Northing)
        // 1 degree of latitude is constant: ~111,320 meters
        let y_m = (lat - s_lat) * 111_319.9;

        // 2. Calculate X (Easting)
        // 1 degree of longitude shrinks as you move toward the poles
        let lon_scale = s_lat.to_radians().cos();
        let x_m = (lon - s_lon) * 111_319.9 * lon_scale;

        Some(RoverState {
            x_m,
            y_m,
            velocity_ms: speed_kmh / 3.6,
        })
    }
}



// ── Haversine formula ─────────────────────────────────────────────────────────
// fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
//     const R: f64 = 6_371_000.0; 
//     let d_lat = (lat2 - lat1).to_radians();
//     let d_lon = (lon2 - lon1).to_radians();
//     let a = (d_lat / 2.0).sin().powi(2)
//         + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
//     R * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
// }