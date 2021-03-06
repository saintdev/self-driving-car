use crate::strategy::{Action, Context};
use common::prelude::*;
use nalgebra::Point3;
use nameof::name_of_type;

const ERROR_THRESHOLD: f32 = 50.0;

/// Track the ball's trajectory vs. our prediction, and if they differ by too
/// much, abort.
pub struct SameBallTrajectory {
    prediction: Option<Prediction>,
}

struct Prediction {
    t: f32,
    loc: Point3<f32>,
}

impl SameBallTrajectory {
    pub fn new() -> SameBallTrajectory {
        SameBallTrajectory { prediction: None }
    }

    pub fn execute_old(&mut self, ctx: &mut Context<'_>) -> Option<Action> {
        if self.eval_vel_changed(ctx) {
            Some(Action::Abort)
        } else {
            self.update_snapshot(ctx);
            None
        }
    }

    fn update_snapshot(&mut self, ctx: &mut Context<'_>) {
        let frame = ctx.scenario.ball_prediction().at_time_or_last(0.1);
        self.prediction = Some(Prediction {
            t: ctx.packet.GameInfo.TimeSeconds + frame.t,
            loc: frame.loc,
        });
    }

    fn eval_vel_changed(&mut self, ctx: &mut Context<'_>) -> bool {
        let prediction = some_or_else!(self.prediction.as_ref(), {
            return false;
        });
        let rel_time = prediction.t - ctx.packet.GameInfo.TimeSeconds;
        let frame = match ctx.scenario.ball_prediction().at_time(rel_time) {
            Some(f) => f,
            None => {
                log::warn!("game time not in prediction range");
                ctx.scenario.ball_prediction().start()
            }
        };

        let error = (prediction.loc - frame.loc).to_2d().norm();
        if error >= ERROR_THRESHOLD {
            ctx.eeg.log(
                name_of_type!(SameBallTrajectory),
                format!("perturbance detected with error {:.2}", error),
            );
            true
        } else {
            false
        }
    }
}
