use crate::{
    routing::{
        behavior::FollowRoute,
        models::{
            PlanningContext, PlanningDump, RoutePlan, RoutePlanError, RoutePlanner, SegmentPlan,
        },
    },
    strategy::Behavior,
};
use derive_new::new;

pub fn segment_plan_tester(plan: impl SegmentPlan + Clone + 'static) -> impl Behavior {
    FollowRoute::new(CookedPlanner::new(plan))
}

#[derive(new, Clone)]
struct CookedPlanner<P>
where
    P: SegmentPlan + Clone + 'static,
{
    plan: P,
}

impl<P> RoutePlanner for CookedPlanner<P>
where
    P: SegmentPlan + Clone + 'static,
{
    fn name(&self) -> &'static str {
        stringify!(CookedPlanner)
    }

    fn plan(
        &self,
        _ctx: &PlanningContext<'_, '_>,
        _dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        Ok(RoutePlan {
            segment: Box::new(self.plan.clone()),
            next: None,
        })
    }
}
