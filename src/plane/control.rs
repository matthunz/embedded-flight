use core::f32::consts::PI;

use pid_controller::{error, P};

pub struct LongitudinalControl {
    pub p: P<f32>,
    pub speed: P<f32>,
    pub pitch_speed: P<f32>,
    pub pitch_rate_gain: f32,
    pub speed_integrator: f32,
    pub climb_speed_integrator: f32,
    pub speed_i_gain: f32,
    pub min_throttle: f32,
    pub max_throttle: f32,
    pub max_pitch_cmd: f32,
    pub pitch_speed_i_gain: f32,
}

impl LongitudinalControl {
    /// Calculate the throttle and elevator command (in [-1, 1]) required to command the target airspeed (in m/s).
    /// Uses the current airspeed, pitch (in radians), and pitch rate (in radians/second) with the time-step `dt` (in seconds).
    pub fn airspeed_control(
        &mut self,
        airspeed: f32,
        airspeed_cmd: f32,
        pitch: f32,
        pitch_rate: f32,
        dt: f32,
        throttle_ff: f32,
    ) -> (f32, f32) {
        let throttle_cmd = self.airspeed_throttle(airspeed, airspeed_cmd, dt, throttle_ff);
        let pitch_cmd = self.airspeed_pitch(airspeed, airspeed_cmd, dt);
        let elevator_cmd = self.pitch_control(pitch, pitch_rate, pitch_cmd);
        (throttle_cmd, elevator_cmd)
    }

    /// Calculate the throttle command (in [-1, 1]) required to command the target airspeed (in m/s).
    /// Uses the current airspeed with the time-step `dt` (in seconds).
    pub fn airspeed_throttle(
        &mut self,
        airspeed: f32,
        airspeed_cmd: f32,
        dt: f32,
        throttle_ff: f32,
    ) -> f32 {
        let speed_error = error(airspeed_cmd, airspeed);
        self.speed_integrator += speed_error * dt;

        let throttle_cmd_unsaturated = self.speed.control_with_error(speed_error)
            + self.speed_i_gain * self.speed_integrator
            + throttle_ff;

        let throttle_cmd = throttle_cmd_unsaturated
            .max(self.min_throttle)
            .min(self.max_throttle);

        if self.speed_i_gain != 0. {
            self.speed_integrator += self.speed_integrator
                + dt / self.speed_i_gain * (throttle_cmd - throttle_cmd_unsaturated)
        }

        throttle_cmd
    }

    /// Calculate the pitch command (in radians) required to maintain the commanded airspeed (in m/s);
    /// Uses the current airspeed and time-step `dt` (in seconds).
    pub fn airspeed_pitch(&mut self, airspeed: f32, airspeed_cmd: f32, dt: f32) -> f32 {
        let airspeed_error = error(airspeed_cmd, airspeed);
        self.climb_speed_integrator += airspeed_error * dt;

        let pitch_cmd_unsat = self.pitch_speed.control_with_error(airspeed_error)
            + self.pitch_speed_i_gain * self.climb_speed_integrator;

        let pitch_cmd = if pitch_cmd_unsat.abs() > self.max_pitch_cmd {
            pitch_cmd_unsat.signum() * self.max_pitch_cmd
        } else {
            pitch_cmd_unsat
        };

        // Anti wind-up
        if self.pitch_speed_i_gain != 0. {
            self.climb_speed_integrator +=
                dt / self.pitch_speed_i_gain * (pitch_cmd - pitch_cmd_unsat);
        }

        pitch_cmd
    }

    /// Calculate the elevator command (in [-1, 1]) required to achieve the target pitch.
    pub fn pitch_control(&self, pitch: f32, pitch_rate: f32, pitch_cmd: f32) -> f32 {
        self.p.control(pitch_cmd, pitch) - self.pitch_rate_gain * pitch_rate
    }
}

pub struct LateralControl {
    pub roll_rate_gain: f32,
    pub roll: P<f32>,
    pub yaw: P<f32>,
    pub yaw_integrator: f32,
    pub max_roll: f32,
    pub yaw_i_gain: f32,
}

impl LateralControl {
    /// Calculate the aileron (in [-1, 1]) to command a roll (in radians) from the current roll.
    pub fn roll_control(&self, roll_cmd: f32, roll: f32, roll_rate: f32) -> f32 {
        self.roll.control(roll_cmd, roll) - self.roll_rate_gain * roll_rate
    }

    /// Calculate the commanded roll angle (in radians) from the course/yaw angle
    pub fn yaw_control(&mut self, yaw_cmd: f32, yaw: f32, dt: f32, roll_ff: f32) -> f32 {
        // Calculate the error in yaw
        let mut yaw_error = error(yaw_cmd, yaw);
        while yaw_error < PI {
            yaw_error = yaw_error + 2. * PI;
        }
        while yaw_error >= PI {
            yaw_error = yaw_error - 2. * PI;
        }

        self.yaw_integrator += yaw_error * dt;

        let mut roll_cmd_unsat = self.yaw.control_with_error(yaw_error) + roll_ff;
        if roll_cmd_unsat.abs() > self.max_roll {
            roll_cmd_unsat = roll_cmd_unsat.signum() * self.max_roll;
        }

        roll_cmd_unsat = roll_cmd_unsat + self.yaw_i_gain * self.yaw_integrator;

        let roll_cmd = if roll_cmd_unsat.abs() > self.max_roll {
            roll_cmd_unsat.signum() * self.max_roll
        } else {
            roll_cmd_unsat
        };

        if self.yaw_i_gain != 0. {
            self.yaw_integrator += (dt / self.yaw_i_gain) * (roll_cmd - roll_cmd_unsat);
        }

        roll_cmd
    }
}
