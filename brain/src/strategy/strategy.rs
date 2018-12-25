use crate::strategy::{Behavior, Context};

pub trait Strategy: Send {
    fn baseline(&mut self, ctx: &mut Context) -> Box<Behavior>;
    fn interrupt(&mut self, ctx: &mut Context, current: &Behavior) -> Option<Box<Behavior>>;
}
