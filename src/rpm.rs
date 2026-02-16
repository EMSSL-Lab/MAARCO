// // rpm.rs
// pub struct RpmController {
//     pub kp: f32,
//     pub kd: f32,
//     last_error: f32,
// }

// impl RpmController {
//     pub fn new(kp: f32, kd: f32) -> Self {
//         Self { kp, kd, last_error: 0.0 }
//     }

//     pub fn calculate_correction(&mut self, target_rpm: f32, current_rpm: f32) -> f32 {
//         let error = target_rpm - current_rpm;
//         let p_term = self.kp * error;
//         let d_term = self.kd * (error - self.last_error);
        
//         self.last_error = error;
//         p_term + d_term
//     }
// }

// // ... inside the loop ...

// // if let Some(sensor_data) = arduino_serial_data.unwrap() {
    
// //     // --- YAW CONTROL ---
// //     if let Some(euler_x) = sensor_data.euler_x {
// //         let target_yaw = 0.0;
// //         let correction = yaw_ctrl.calculate_correction(target_yaw, euler_x);
        
// //         // We map the correction (assuming a max correction of +/- 180) 
// //         // to our PWM range around the 1500 midpoint.
// //         let pulse = map_range(correction, -180.0, 180.0, 1000, 2000) as u64;
// //         let safe_pulse = pulse.clamp(1000, 2000);
        
// //         let _ = motor::update_pwm(&mut yaw_pin, safe_pulse);
// //     }

// //     // --- RPM CONTROL ---
// //     if let Some(current_rpm) = sensor_data.rpm {
// //         let target_rpm = 100.0;
// //         let correction = rpm_ctrl.calculate_correction(target_rpm, current_rpm);
        
// //         // Here we map the RPM correction (assuming max error of +/- 100 RPM)
// //         let pulse = map_range(correction, -100.0, 100.0, 1000, 2000) as u64;
// //         let safe_pulse = pulse.clamp(1000, 2000);
        
// //         let _ = motor::update_pwm(&mut rpm_pin, safe_pulse);
// //     }
// // }