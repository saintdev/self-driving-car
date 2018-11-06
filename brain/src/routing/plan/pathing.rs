use common::{physics, prelude::*, rl};
use nalgebra::Point2;
use routing::{
    models::{CarState, RoutePlanner},
    plan::{
        ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner,
        higher_order::ChainedPlanner,
    },
    segments::StraightMode,
};

/// Calculate whether driving straight would intersect the goal wall. If so,
/// return the route we should follow to get outside the goal.
pub fn avoid_smacking_goal_wall(start: &CarState) -> Option<Box<RoutePlanner>> {
    match avoid_smacking_goal_wall_waypoint(start) {
        None => None,
        Some(waypoint) => Some(ChainedPlanner::chain(vec![
            Box::new(TurnPlanner::new(waypoint, None)),
            Box::new(GroundStraightPlanner::new(
                waypoint,
                0.0,
                0.0,
                StraightMode::Asap,
            )),
        ])),
    }
}

/// Calculate whether driving straight would intersect the goal wall. If so,
/// return the waypoint we should drive to first to avoid embarrassing
/// ourselves.
pub fn avoid_smacking_goal_wall_waypoint(start: &CarState) -> Option<Point2<f32>> {
    let margin = 125.0;
    if start.loc.y.abs() < rl::FIELD_MAX_Y {
        return None;
    }
    let goal_y = rl::FIELD_MAX_Y * start.loc.y.signum();
    let ray = physics::car_forward_axis_2d(start.rot.to_2d());
    let toi = (goal_y - start.loc.y) / ray.y;
    let cross_x = start.loc.x + toi * ray.x;
    if cross_x.abs() >= rl::GOALPOST_X {
        Some(Point2::new(
            (rl::GOALPOST_X - margin) * cross_x.signum(),
            (rl::FIELD_MAX_Y - margin) * start.loc.y.signum(),
        ))
    } else {
        None
    }
}