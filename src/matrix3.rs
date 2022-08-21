#[allow(unused_imports)]
use num_traits::real::Real;

#[cfg(test)]
mod test_mat {
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn angle_axis() {
        // https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
        let a = na::geometry::Rotation3::<f32>::from_axis_angle(
            &na::Vector3::<f32>::x_axis(),
            0.25 * core::f32::consts::PI,
        );
        let b = na::geometry::Rotation3::<f32>::from_axis_angle(
            &na::Vector3::<f32>::y_axis(),
            0.5 * core::f32::consts::PI,
        );
        let c = na::geometry::Rotation3::<f32>::from_axis_angle(
            &na::Vector3::<f32>::z_axis(),
            0.33 * core::f32::consts::PI,
        );
        let m = a * b * c;

        let expect = [[0.0, 0.0, 1.0], [0.969, -0.249, 0.0], [0.249, 0.969, 0.0]];
        for i in 0..m.matrix().as_ref().len() {
            for j in 0..m.matrix().as_ref().iter().len() {
                assert_approx_eq!(m[(i, j)], expect[i][j], 0.001);
            }
        }
    }
}
