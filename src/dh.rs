pub trait DH<T: na::RealField> {
    fn from_dh_param(a: T, alfa: T, d: T, theta: T) -> na::Isometry3<T>;
}

impl<T: na::RealField> DH<T> for na::geometry::Isometry3<T> {
    fn from_dh_param(a: T, alfa: T, d: T, theta: T) -> Self {
        let ta = na::Isometry3::new(
            na::Vector3::new(a, T::zero(), T::zero()),
            na::Vector3::zeros(),
        );
        let td = na::Isometry3::new(
            na::Vector3::new(T::zero(), T::zero(), d),
            na::Vector3::zeros(),
        );
        let talfa = na::Isometry3::new(na::Vector3::zeros(), na::Vector3::x() * alfa);
        let ttheta = na::Isometry3::new(na::Vector3::zeros(), na::Vector3::z() * theta);

        ta * talfa * td * ttheta
    }
}

#[cfg(test)]
mod dh_param_tests {
    use crate::dh::DH;
    use assert_approx_eq::assert_approx_eq;
    use core::f32::consts::PI;

    #[test]
    fn dh_mat() {
        let t = na::Isometry3::from_dh_param(0.0, 0.0, 0.0, PI / 6.0);
        let c = 3.0_f32.sqrt() / 2.0;
        let s = 1.0 / 2.0;

        let expect = [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        for i in 0..t.to_matrix().as_ref().len() {
            for j in 0..t.to_matrix().as_ref().iter().len() {
                assert_approx_eq!(t.to_matrix()[(i, j)], expect[i][j], 0.001);
            }
        }
    }
    #[test]
    fn dh_mat2() {
        let t = na::Isometry3::from_dh_param(0.0, PI / 2.0, 3.0, PI / 6.0);
        let c = 3.0_f32.sqrt() / 2.0;
        let s = 1.0 / 2.0;

        let expect = [
            [c, -s, 0.0, 0.0],
            [0.0, 0.0, -1.0, -3.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        for i in 0..t.to_matrix().as_ref().len() {
            for j in 0..t.to_matrix().as_ref().iter().len() {
                assert_approx_eq!(t.to_matrix()[(i, j)], expect[i][j], 0.001);
            }
        }
    }
}
