use core::ops::{Add, AddAssign, Mul, Neg};
#[allow(unused_imports)]
use num_traits::real::Real;
// use core::f32::consts::PI;

use crate::matrix::Matrix;
use crate::vector3::Vector3;

pub trait Matrix3<T> {
    fn from_angle_axis(angle: T, axis: Matrix<T, 3, 1>) -> Matrix<T, 3, 3>;
}

impl<T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T> + num_traits::Float>
    Matrix3<T> for Matrix<T, 3, 3>
{
    fn from_angle_axis(angle: T, axis: Matrix<T, 3, 1>) -> Matrix<T, 3, 3> {
        Matrix::<T, 3, 3>::identity() * angle.cos()
            + axis * axis.transpose() * (<T as From<f32>>::from(1.0) - angle.cos())
            + axis.as_skew_symmetric_matrix() * angle.sin()
    }
}

#[cfg(test)]
mod test_mat {
    use crate::matrix::Matrix;
    use crate::matrix3::Matrix3;
    use crate::vector3::Vector3;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn angle_axis() {
        // https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
        let a = Matrix::<f32, 3, 3>::from_angle_axis(
            0.25 * core::f32::consts::PI,
            Matrix::<f32, 3, 1>::unit_x(),
        );
        let b = Matrix::<f32, 3, 3>::from_angle_axis(
            0.5 * core::f32::consts::PI,
            Matrix::<f32, 3, 1>::unit_y(),
        );
        let c = Matrix::<f32, 3, 3>::from_angle_axis(
            0.33 * core::f32::consts::PI,
            Matrix::<f32, 3, 1>::unit_z(),
        );
        let m = a * b * c;

        let expect = [[0.0, 0.0, 1.0], [0.969, -0.249, 0.0], [0.249, 0.969, 0.0]];
        for i in 0..m.as_ref().len() {
            for j in 0..m.as_ref().iter().len() {
                assert_approx_eq!(m.as_ref()[i][j], expect[i][j], 0.001);
            }
        }
    }
}
