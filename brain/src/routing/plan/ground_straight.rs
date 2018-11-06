use common::{prelude::*, rl, PrettyPrint};
use nalgebra::Point2;
use ordered_float::NotNan;
use routing::{
    models::{
        CarState, CarState2D, PlanningContext, PlanningDump, RoutePlan, RoutePlanError,
        RoutePlanner, SegmentPlan,
    },
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{Chain, ForwardDodge, Straight, StraightMode},
};
use simulate::{Car1D, CarForwardDodge, CarForwardDodge1D};

#[derive(Clone, new)]
pub struct GroundStraightPlanner {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for GroundStraightPlanner {
    fn name(&self) -> &'static str {
        stringify!(GroundStraightPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log(self, format!("target_loc = {}", self.target_loc.pretty()));

        assert!(!self.target_loc.x.is_nan());
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: self.target_loc,
            },
        );

        let simple =
            StraightSimple::new(self.target_loc, self.target_time, self.end_chop, self.mode);
        let with_dodge =
            StraightWithDodge::new(self.target_loc, self.target_time, self.end_chop, self.mode);

        let planners = [&simple as &RoutePlanner, &with_dodge];
        let plans = planners.iter().map(|p| p.plan(ctx, dump));
        let plans = at_least_one_ok(plans)?;
        Ok(fastest(plans.into_iter()))
    }
}

fn at_least_one_ok<T, E>(results: impl Iterator<Item = Result<T, E>>) -> Result<Vec<T>, E> {
    let mut oks: Vec<T> = Vec::new();
    let mut error = None;
    for result in results {
        match result {
            Ok(x) => oks.push(x),
            Err(e) => error = Some(e),
        }
    }
    if oks.is_empty() {
        Err(error.unwrap())
    } else {
        Ok(oks)
    }
}

fn fastest(steps: impl Iterator<Item = RoutePlan>) -> RoutePlan {
    steps
        .min_by_key(|s| NotNan::new(s.segment.duration()).unwrap())
        .unwrap()
}

#[derive(Clone, new)]
struct StraightSimple {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for StraightSimple {
    fn name(&self) -> &'static str {
        stringify!(StraightSimple)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        _dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: self.target_loc,
            },
        );
        guard!(
            ctx.start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let segment = Straight::new(
            CarState2D {
                loc: ctx.start.loc.to_2d(),
                rot: ctx.start.rot.to_2d(),
                vel: ctx.start.vel.to_2d(),
                boost: ctx.start.boost,
            },
            self.target_loc,
            self.end_chop,
            self.mode,
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}

/// Calculate a ground interception of the ball with a single dodge.
#[derive(Clone, new)]
struct StraightWithDodge {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for StraightWithDodge {
    fn name(&self) -> &'static str {
        stringify!(StraightWithDodge)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        _dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(
            ctx.start,
            IsSkidding,
            RoutePlanError::MustNotBeSkidding {
                recover_target_loc: self.target_loc,
            },
        );
        guard!(
            ctx.start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let dodges = StraightDodgeCalculator::new(
            ctx.start.clone(),
            self.target_loc,
            self.target_time,
            self.end_chop,
        )
        .collect();
        let dodge = dodges
            .into_iter()
            .min_by_key(|d| NotNan::new(d.score).unwrap())
            .ok_or(RoutePlanError::MovingTooFast)?;

        let before = Straight::new(
            CarState2D {
                loc: ctx.start.loc.to_2d(),
                rot: ctx.start.rot.to_2d(),
                vel: ctx.start.vel.to_2d(),
                boost: ctx.start.boost,
            },
            ctx.start.loc.to_2d()
                + (self.target_loc - ctx.start.loc.to_2d()).normalize() * dodge.approach_distance,
            0.0,
            StraightMode::Asap,
        );
        let dodge = ForwardDodge::new(before.end(), dodge.dodge);
        let dodge_end = dodge.end();
        let after = Straight::new(
            CarState2D {
                loc: dodge_end.loc.to_2d(),
                rot: dodge_end.rot.to_2d(),
                vel: dodge_end.vel.to_2d(),
                boost: dodge_end.boost,
            },
            self.target_loc,
            self.end_chop,
            self.mode,
        );
        let segment = Chain::new(vec![Box::new(before), Box::new(dodge), Box::new(after)]);
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}

/// Calculate motions consisting of straight, then dodge, then straight again.
#[derive(new)]
struct StraightDodgeCalculator {
    start: CarState,
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    end_chop: f32,
}

impl StraightDodgeCalculator {
    pub fn collect(&self) -> Vec<StraightDodge> {
        // Performance knob
        const GRANULARITY: f32 = 0.1;

        let mut car = Car1D::new(self.start.vel.to_2d().norm()).with_boost(self.start.boost);
        let mut result = Vec::new();

        loop {
            if let Some(target_time) = self.target_time {
                if car.time() >= target_time {
                    break;
                }
            }

            car.multi_step(GRANULARITY, rl::PHYSICS_DT, 1.0, true);
            match self.evaluate(&car) {
                Some(dodge) => result.push(dodge),
                None => break,
            }
        }

        result
    }

    fn evaluate(&self, approach: &Car1D) -> Option<StraightDodge> {
        let dodge = CarForwardDodge::calc_1d(approach.speed());

        // `end_chop` is "dead time" that the caller requested we leave available for
        // the subsequent maneuver. Coasting on landing is the most conservative case.
        let mut dodge_end = Car1D::new(dodge.end_speed).with_boost(approach.boost());
        dodge_end.multi_step(self.end_chop, rl::PHYSICS_DT, 0.0, false);
        let dodge_end = dodge_end;

        // Now we know where this dodge would take us. Let's check if it meets the
        // requirements:

        // Check if we can even complete the dodge by the target time.
        let total_time = approach.time() + dodge.duration() + dodge_end.time();
        if let Some(target_time) = self.target_time {
            if total_time > target_time {
                return None;
            }
        }

        // Check that we don't land past the target.
        let target_traveled = (self.target_loc - self.start.loc.to_2d()).norm();
        let total_dist =
            approach.distance_traveled() + dodge.end_dist + dodge_end.distance_traveled();
        if total_dist >= target_traveled {
            return None;
        }

        // Now simulate coasting after the landing. If we pass the target before the
        // target time, dodging made us go too fast.
        if let Some(target_time) = self.target_time {
            let mut coast = Car1D::new(dodge.end_speed).with_boost(approach.boost());
            coast.multi_step(target_time - total_time, rl::PHYSICS_DT, 0.0, false);
            if total_dist + coast.distance_traveled() > target_traveled {
                return None;
            }
        }

        // To calculate the "best" dodge, they all need to be compared on equal terms.
        // The equal term I choose is the minimum time needed to reach the target.
        let mut blitz = Car1D::new(dodge.end_speed).with_boost(approach.boost());
        while total_dist + blitz.distance_traveled() < target_traveled {
            blitz.step(rl::PHYSICS_DT, 1.0, true);
        }
        let score = total_time + blitz.time();

        Some(StraightDodge {
            approach_distance: approach.distance_traveled(),
            dodge,
            score,
        })
    }
}

struct StraightDodge {
    approach_distance: f32,
    dodge: CarForwardDodge1D,
    score: f32,
}
