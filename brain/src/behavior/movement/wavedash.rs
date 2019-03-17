use crate::strategy::{Action, Behavior, Context, Priority};

use common::{
    ext::{ExtendPhysics, ExtendRotation3},
    rl,
};

use nameof::name_of_type;
use std::f32::consts::PI;

const JUMP_INPUT_TIME: f32 = 2.0 / rl::PHYSICS_TICK_FREQ;
const PITCH_ADJUSTMENT_TIME: f32 = 8.0 / rl::PHYSICS_TICK_FREQ;
const DODGE_WAIT_TIME: f32 = 98.0 / rl::PHYSICS_TICK_FREQ;
// FIXME: This is just a guess. Complete stabilization doesn't happen until
//        around 48 ticks have passed. We are stable enough to move sooner than
//        that.
const FOLLOW_THROUGH_TIME: f32 = 12.0 / rl::PHYSICS_TICK_FREQ;

enum Phase {
    Jump,
    Adjust,
    Wait,
    Dodge,
    FollowThrough,
    Finished,
}

pub struct Wavedash {
    phase_start_time: Option<f32>,
    starting_pitch: Option<f32>,
    phase: Phase,
    throttle: f32,
}

impl Wavedash {
    pub fn new() -> Self {
        Wavedash {
            phase_start_time: None,
            starting_pitch: None,
            phase: Phase::Jump,
            throttle: 1.0, // Force on for now, inherit throttle from parent?
        }
    }

    // This should be close to accurate. The exact time varies, because
    // the time spent adjusting pitch and how long we wait before dodging
    // are based on physics conditions.
    pub fn estimated_duration() -> f32 {
        JUMP_INPUT_TIME
            + PITCH_ADJUSTMENT_TIME
            + DODGE_WAIT_TIME
            + JUMP_INPUT_TIME     // Dodge
            + FOLLOW_THROUGH_TIME // Handbrake slide to retain speed and stabalize
    }
}

impl Behavior for Wavedash {
    fn name(&self) -> &str {
        name_of_type!(Wavedash)
    }

    fn priority(&self) -> Priority {
        Priority::Force
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let phase_start_time = self
            .phase_start_time
            .get_or_insert(ctx.packet.GameInfo.TimeSeconds);
        let starting_pitch = *self
            .starting_pitch
            .get_or_insert(ctx.me().Physics.rot().pitch());
        let current_pitch = ctx.me().Physics.rot().pitch();
        let pitch_delta = current_pitch - starting_pitch;
        let elapsed = ctx.packet.GameInfo.TimeSeconds - *phase_start_time;

        match self.phase {
            Phase::Jump => {
                if elapsed >= JUMP_INPUT_TIME {
                    self.phase = Phase::Adjust;
                    *phase_start_time = ctx.packet.GameInfo.TimeSeconds;

                    return self.execute_old(ctx);
                }

                if !ctx.me().OnGround {
                    ctx.eeg.log(self.name(), "wheels not on ground");
                    return Action::Abort;
                }

                Action::Yield(common::halfway_house::PlayerInput {
                    Jump: true,
                    Pitch: 1.0,
                    Throttle: self.throttle,
                    ..Default::default()
                })
            }
            Phase::Adjust => {
                // This is about what the pitch change is in 8 physics ticks.
                if pitch_delta >= PI / 360.0 {
                    self.phase = Phase::Wait;
                    *phase_start_time = ctx.packet.GameInfo.TimeSeconds;

                    return self.execute_old(ctx);
                }

                if elapsed > PITCH_ADJUSTMENT_TIME * 2.0 {
                    ctx.eeg
                        .log(self.name(), "spent too much time leaning back, aborting");
                    return Action::Abort;
                }

                Action::Yield(common::halfway_house::PlayerInput {
                    Pitch: 1.0,
                    Throttle: self.throttle,
                    ..Default::default()
                })
            }
            Phase::Wait => {
                if ctx.me().Physics.loc().z <= 39.0 && ctx.me().Physics.vel().z < 0.0 {
                    self.phase = Phase::Dodge;
                    *phase_start_time = ctx.packet.GameInfo.TimeSeconds;

                    return self.execute_old(ctx);
                }

                if elapsed > DODGE_WAIT_TIME * 1.25 {
                    ctx.eeg.log(self.name(), "took too long to land, aborting");
                    return Action::Abort;
                }

                if ctx.me().DoubleJumped {
                    ctx.eeg.log(self.name(), "already double jumped");
                    return Action::Abort;
                }

                Action::Yield(common::halfway_house::PlayerInput {
                    Throttle: self.throttle,
                    ..Default::default()
                })
            }
            Phase::Dodge => {
                if elapsed >= JUMP_INPUT_TIME {
                    self.phase = Phase::FollowThrough;
                    *phase_start_time = ctx.packet.GameInfo.TimeSeconds;

                    return self.execute_old(ctx);
                }

                if ctx.me().OnGround {
                    ctx.eeg.log(self.name(), "goomba stomped?");
                    return Action::Abort;
                }

                Action::Yield(common::halfway_house::PlayerInput {
                    Pitch: -1.0,
                    Jump: true,
                    Handbrake: true,
                    Throttle: self.throttle,
                    ..Default::default()
                })
            }
            Phase::FollowThrough => {
                if elapsed >= FOLLOW_THROUGH_TIME {
                    self.phase = Phase::Finished;
                    return self.execute_old(ctx);
                }

                Action::Yield(common::halfway_house::PlayerInput {
                    Handbrake: true,
                    ..Default::default()
                })
            }
            Phase::Finished => Action::Return,
        }
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::{
            higher_order::Chain,
            movement::{Land, Wavedash},
        },
        integration_tests::{TestRunner, TestScenario},
        strategy::Priority,
    };
    use common::prelude::*;
    use nalgebra::{Point3, Vector3};
    use vec_box::vec_box;

    #[test]
    fn wavedash_to_supersonic() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(0.0, -3900.0, 17.01),
                car_vel: Vector3::new(0.0, 1800.0, 0.0),
                ball_loc: Point3::new(2000.0, 0.0, 92.74),
                ..Default::default()
            })
            .behavior(Chain::new(Priority::Idle, vec_box![
                Land::new(),
                Wavedash::new(),
            ]))
            .run_for_millis(1250);
        let packet = test.sniff_packet();
        let vel = packet.GameCars[0].Physics.vel().norm();
        println!("vel = {}", vel);
        assert!(vel >= 2200.0)
    }
}
