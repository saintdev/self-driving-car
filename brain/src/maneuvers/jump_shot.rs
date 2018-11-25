#[cfg(test)]
mod integration_tests {
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore(note = "The great bankruptcy of 2018")]
    fn far_corner_falling() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Vector3::new(2790.6147, 4609.1733, 101.71101),
                ball_vel: Vector3::new(-1887.2709, 1104.1567, -66.64522),
                car_loc: Vector3::new(3066.908, 4276.6646, 20.0),
                car_rot: Rotation3::from_unreal_angles(0.0, 3.43, 0.0),
                car_vel: Vector3::new(-1351.3594, 345.7535, 108.78551),
                ..Default::default()
            })
            .run_for_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn sideways_jump() {
        let _test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Vector3::new(3882.8804, 1246.1459, 145.22563),
                ball_vel: Vector3::new(-48.063526, 172.48882, -818.2703),
                car_loc: Vector3::new(3656.8044, -12.230183, 16.709408),
                car_rot: Rotation3::from_unreal_angles(-0.0069029136, 2.6237783, -0.002396845),
                car_vel: Vector3::new(-855.45123, 459.4182, 14.042989),
                ..Default::default()
            })
            .run_for_millis(5000);

        unimplemented!();
    }

    #[test]
    #[ignore(note = "The great bankruptcy of 2018")]
    fn correct_mispredicted_bounce() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Vector3::new(-3350.3652, 2287.6494, 537.7215),
                ball_vel: Vector3::new(583.83765, 331.18698, -33.879772),
                car_loc: Vector3::new(-3420.127, 75.629135, 17.02),
                car_rot: Rotation3::from_unreal_angles(-0.009491506, 0.52864814, -0.0001917476),
                car_vel: Vector3::new(1083.5627, 572.17487, 8.241),
                ..Default::default()
            })
            .run_for_millis(3000);

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "[JumpShot] Air"));
        });
        assert!(test.has_scored());
    }
}
