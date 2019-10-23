#[cfg(test)]
use crate::strategy::Behavior;
use crate::{
    eeg::{color, Drawable, EEG},
    helpers::ball::{BallPredictor, FrameworkBallPrediction},
    strategy::{infer_game_mode, Context, Dropshot, Game, Runner, Scenario, Soccar},
    utils::FPSCounter,
};
use common::{prelude::*, ControllerInput, ExtendDuration};
use nalgebra::{clamp, Point3};
use nameof::name_of_type;
use std::time::Instant;

#[cfg(target_family = "windows")]
use crate::helpers::ball::ChipBallPrediction;

pub struct Brain<'a> {
    runner: Runner,
    ball_predictor: Box<dyn BallPredictor + 'a>,
    player_index: Option<i32>,
    fps_counter: FPSCounter,
    /// This is not automated or enforced in any way, it's just a convenient
    /// memory slot for optional use in behaviors.
    last_quick_chat: f32,
}

impl<'a> Brain<'a> {
    fn new(runner: Runner, ball_predictor: impl BallPredictor + 'a) -> Self {
        Self {
            runner,
            ball_predictor: Box::new(ball_predictor),
            player_index: None,
            fps_counter: FPSCounter::new(),
            last_quick_chat: 0.0,
        }
    }

    // This is just here so it's exported from the crate since I'm lazy
    pub fn infer_game_mode(field_info: rlbot::flat::FieldInfo<'_>) -> rlbot::GameMode {
        infer_game_mode(field_info)
    }

    #[cfg(target_family = "windows")]
    pub fn soccar(_rlbot: &'a rlbot::RLBot) -> Self {
        Self::new(Runner::new(Soccar::new()), ChipBallPrediction::new())
    }

    #[cfg(target_family = "unix")]
    pub fn soccar(rlbot: &'a rlbot::RLBot) -> Self {
        Self::new(
            Runner::new(Soccar::new()),
            FrameworkBallPrediction::new(rlbot),
        )
    }

    pub fn dropshot(rlbot: &'a rlbot::RLBot) -> Self {
        Self::new(
            Runner::new(Dropshot::new()),
            FrameworkBallPrediction::new(rlbot),
        )
    }

    pub fn hoops(rlbot: &'a rlbot::RLBot) -> Self {
        Self::new(
            Runner::new(Soccar::new()),
            FrameworkBallPrediction::new(rlbot),
        )
    }

    #[cfg(test)]
    #[cfg(target_family = "windows")]
    pub fn with_behavior(_rlbot: &'a rlbot::RLBot, behavior: impl Behavior + 'static) -> Self {
        Self::new(Runner::with_current(behavior), ChipBallPrediction::new())
    }

    #[cfg(test)]
    #[cfg(target_family = "unix")]
    pub fn with_behavior(rlbot: &'a rlbot::RLBot, behavior: impl Behavior + 'static) -> Self {
        Self::new(
            Runner::with_current(behavior),
            FrameworkBallPrediction::new(rlbot),
        )
    }

    #[cfg(test)]
    pub fn set_behavior(&mut self, behavior: impl Behavior + 'static, eeg: &mut EEG) {
        eeg.log(name_of_type!(Brain<'_>), format!("! {}", behavior.name()));
        self.runner = Runner::with_current(behavior);
    }

    pub fn set_player_index(&mut self, player_index: i32) {
        self.player_index = Some(player_index);
    }

    pub fn tick(
        &mut self,
        field_info: rlbot::flat::FieldInfo<'_>,
        packet: &common::halfway_house::LiveDataPacket,
        eeg: &mut EEG,
    ) -> common::halfway_house::PlayerInput {
        self.fps_counter.tick(packet.GameInfo.TimeSeconds);

        eeg.print_time("game_time", packet.GameInfo.TimeSeconds);
        eeg.print_value("fps", format_fps(self.fps_counter.fps()));
        eeg.print_value("ball loc", packet.GameBall.Physics.loc());
        eeg.print_value("ball vel", packet.GameBall.Physics.vel());
        eeg.print_value("p1 loc", packet.GameCars[0].Physics.loc());
        eeg.print_value("p1 vel", Point3::from(packet.GameCars[0].Physics.vel()));
        eeg.draw(Drawable::print("-----------------------", color::GREEN));

        let mut result = self.determine_controls(field_info, packet, eeg);

        result.Throttle = clamp(result.Throttle, -1.0, 1.0);
        result.Steer = clamp(result.Steer, -1.0, 1.0);
        result.Pitch = clamp(result.Pitch, -1.0, 1.0);
        result.Yaw = clamp(result.Yaw, -1.0, 1.0);
        result.Roll = clamp(result.Roll, -1.0, 1.0);

        eeg.draw(Drawable::print("-----------------------", color::GREEN));
        eeg.print_value("throttle", ControllerInput(result.Throttle));
        eeg.print_value("steer", ControllerInput(result.Steer));
        eeg.print_value("pitch", ControllerInput(result.Pitch));
        eeg.print_value("yaw", ControllerInput(result.Yaw));
        eeg.print_value("roll", ControllerInput(result.Roll));
        eeg.print_value("jump", result.Jump);
        eeg.print_value("boost", result.Boost);
        eeg.print_value("handbrake", result.Handbrake);

        result
    }

    fn determine_controls(
        &mut self,
        field_info: rlbot::flat::FieldInfo<'_>,
        packet: &common::halfway_house::LiveDataPacket,
        eeg: &mut EEG,
    ) -> common::halfway_house::PlayerInput {
        let start = Instant::now();

        let game = Game::new(field_info, packet, self.player_index.unwrap() as usize);
        let scenario = Scenario::new(&game, &*self.ball_predictor, packet);
        let mut ctx = Context::new(&game, packet, &scenario, eeg, &mut self.last_quick_chat);

        ctx.eeg.print_time("possession", ctx.scenario.possession());

        let result = self.runner.execute_old(&mut ctx);

        let stop = Instant::now();
        let duration = stop - start;
        let calc_ms = duration.as_millis_polyfill();
        // RL's physics runs at 120Hz, which leaves us ~8ms to make a decision.
        if calc_ms >= 8 {
            ctx.eeg.log(
                name_of_type!(Brain<'_>),
                format!("slow frame took {}ms", calc_ms),
            );
        }

        result
    }
}

fn format_fps(fps: Option<usize>) -> String {
    fps.map(|x| format!("{:.0}", x))
        .unwrap_or_else(|| "...".to_string())
}
