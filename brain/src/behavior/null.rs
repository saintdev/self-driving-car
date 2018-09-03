use behavior::behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use rlbot;

pub struct NullBehavior;

impl NullBehavior {
    pub fn new() -> NullBehavior {
        NullBehavior
    }
}

impl Behavior for NullBehavior {
    fn name(&self) -> &'static str {
        "NullBehavior"
    }

    fn execute(&mut self, _packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        eeg.draw(Drawable::print(self.name(), color::YELLOW));
        Action::Yield(rlbot::PlayerInput::default())
    }
}