use crate::{
    integration_tests::helpers::{TestRunner, TestScenario},
    strategy::Runner,
};
use common::prelude::*;
use nalgebra::{Rotation3, Vector3};

#[test]
#[ignore]
// We came in too hot, we should be able to smack it to the side.
fn todo() {
    let test = TestRunner::start0(TestScenario {
        ball_loc: Vector3::new(-1004.2267, -1863.0571, 93.15),
        ball_vel: Vector3::new(1196.1945, -1186.7386, 0.0),
        car_loc: Vector3::new(1692.9968, -2508.7695, 17.01),
        car_rot: Rotation3::from_unreal_angles(-0.009779127, -2.0910075, 0.0),
        car_vel: Vector3::new(-896.0074, -1726.876, 8.375226),
        ..Default::default()
    });
    test.set_behavior(Runner::soccar());
    unimplemented!();
}

#[allow(deprecated)]
mod template {
    use crate::{
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::Runner,
    };
    use nalgebra::Vector3;

    #[test]
    #[ignore]
    fn scenario_template() {
        let _test = TestRunner::new()
            .scenario(TestScenario {
                enemy_loc: Vector3::new(6000.0, 6000.0, 0.0),
                ..TestScenario::from_recorded_row("../logs/play.csv", 10.0)
            })
            .starting_boost(25.0)
            .behavior(Runner::soccar())
            .run_for_millis(7000);
        unimplemented!();
    }

    #[test]
    #[ignore]
    fn recording_template() {
        let _test = TestRunner::new()
            .preview_recording("../logs/play.csv", 100.0, 2.0, 5.0)
            .starting_boost(25.0)
            .behavior(Runner::soccar())
            .run_for_millis(7000);
        unimplemented!();
    }
}
