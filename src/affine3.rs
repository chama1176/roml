use core::ops::{Add, AddAssign, Mul, Neg};
#[allow(unused_imports)]
use num_traits::real::Real;

use crate::matrix::Matrix;
use crate::vector3::Vector3;
use crate::matrix3::Matrix3;

pub trait Affine3<T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T> + num_traits::Float> {
    fn rotation(&self) -> [[T; 3]; 3];
    fn translation(&self) -> [[T; 1]; 3];
    fn from_dh_param(a: T, alfa: T, d: T, theta: T) -> Matrix<T, 4, 4>;

    fn mat_trans_x(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
    fn mat_rot_x(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        let r = Matrix::<T, 3, 3>::from_angle_axis(d, Matrix::<T, 3, 1>::unit_x());
        for i in 0..3 {
            for j in 0..3 {
                t.as_mut()[i][j] = r.as_ref()[i][j];
            }
        }
        t
    }
    fn mat_trans_z(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        t.as_mut()[2][3] = d;
        t
    }
    fn mat_rot_z(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        let r = Matrix::<T, 3, 3>::from_angle_axis(d, Matrix::<T, 3, 1>::unit_z());
        for i in 0..3 {
            for j in 0..3 {
                t.as_mut()[i][j] = r.as_ref()[i][j];
            }
        }
        t
    }
}

impl<T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T> + num_traits::Float>
    Affine3<T> for Matrix<T, 4, 4>
{
    fn rotation(&self) -> [[T; 3]; 3] {
        [[self.as_ref()[0][0], self.as_ref()[0][1], self.as_ref()[0][2]],
        [self.as_ref()[1][0], self.as_ref()[1][1], self.as_ref()[1][2]],
        [self.as_ref()[2][0], self.as_ref()[2][1], self.as_ref()[2][2]],]
    }

    fn translation(&self) -> [[T; 1]; 3] {
        [[self.as_ref()[0][3]],
        [self.as_ref()[0][3]],
        [self.as_ref()[0][3]]]
    }

    fn from_dh_param(a: T, alfa: T, d: T, theta: T) -> Matrix<T, 4, 4> {
        Self::mat_trans_x(a) * Self::mat_rot_x(alfa) * Self::mat_trans_z(d) * Self::mat_rot_z(theta)
    }
}

#[cfg(test)]
mod dh_param_tests {
    use crate::affine3::Affine3;
    use crate::Matrix;
    use core::f32::consts::PI as PI;
    use assert_approx_eq::assert_approx_eq;


    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
    #[test]
    fn dh_mat() {
        let t = Matrix::<f32, 4, 4>::from_dh_param(0.0, 0.0, 0.0, PI/6.0);
        let c = 3.0_f32.sqrt() / 2.0;
        let s = 1.0 / 2.0;
        assert_eq!(
            *t.as_ref(),
            [
                [c, -s, 0.0, 0.0],
                [s, c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ]
        );
    }
    #[test]
    fn dh_mat2() {
        let t = Matrix::<f32, 4, 4>::from_dh_param(0.0, PI/2.0, 3.0, PI/6.0);
        let c = 3.0_f32.sqrt() / 2.0;
        let s = 1.0 / 2.0;

        let expect =             [
            [c, -s, 0.0, 0.0],
            [0.0, 0.0, -1.0, -3.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ];

        for i in 0..t.as_ref().len() {
            for j in 0..t.as_ref().iter().len() {
                assert_approx_eq!(t.as_ref()[i][j], expect[i][j]);
            }
        }
    }

}
