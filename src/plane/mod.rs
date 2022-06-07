use core::f32::consts::PI;

use pid_controller::{error, P};

pub struct LateralControl {
    pub roll_rate_gain: f32,
    pub p: P<f32>,
    pub yaw: P<f32>,
    pub yaw_integrator: f32,
    pub max_roll: f32,
    pub yaw_i_gain: f32,
}

impl LateralControl {
    /// Calculate the aileron (in [-1, 1]) to command a roll (in radians) from the current roll.
    pub fn roll_attitude_hold_loop(&self, roll_cmd: f32, roll: f32, roll_rate: f32) -> f32 {
        self.p.control(roll_cmd, roll) - self.roll_rate_gain * roll_rate
    }

    /// Calculate the commanded roll angle from the course/yaw angle
    pub fn yaw_hold_loop(&mut self, yaw_cmd: f32, yaw: f32, dt: f32, roll_ff: f32) -> f32 {
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
