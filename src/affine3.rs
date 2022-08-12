use core::ops::{Add, AddAssign, Mul, Neg};
#[allow(unused_imports)]
use num_traits::real::Real;

use crate::matrix::Matrix;

pub trait Affine3<T: From<f32> + Copy> {
    fn from_dh_param(a: T, alfa: T, d: T, theta: T) -> Matrix<T, 4, 4>;
    fn mat_trans_x(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
    fn mat_rot_x(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
    fn mat_trans_z(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        t.as_mut()[2][3] = d;
        t
    }
    fn mat_rot_z(d: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
}

// impl Affine3<T> {
//     fn mat_trans_x(d: f32) -> Matrix<f32, 4, 4> {
//         let mut t = Matrix::<f32, 4, 4>::identity();
//         t.as_mut()[0][3] = d;
//         t
//     }
//     fn mat_rot_x(d: f32) -> Matrix<f32, 4, 4> {
//         let mut t = Matrix::<f32, 4, 4>::identity();
//         t.as_mut()[0][3] = d;
//         t
//     }
//     fn mat_trans_z(d: f32) -> Matrix<f32, 4, 4> {
//         let mut t = Matrix::<f32, 4, 4>::identity();
//         t.as_mut()[2][3] = d;
//         t
//     }
//     fn mat_rot_z(d: f32) -> Matrix<f32, 4, 4> {
//         let mut t = Matrix::<f32, 4, 4>::identity();
//         t.as_mut()[0][3] = d;
//         t
//     }

// }

impl<T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T> + num_traits::Float>
    Affine3<T> for Matrix<T, 4, 4>
{
    fn from_dh_param(a: T, alfa: T, d: T, theta: T) -> Matrix<T, 4, 4> {
        let mut t = Matrix::<T, 4, 4>::new();
        t = t * Self::mat_trans_x(a);

        t
    }
}

#[cfg(test)]
mod dh_param_tests {
    use crate::affine3::Affine3;
    use crate::Matrix;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
    #[test]
    fn init_transform_mat() {
        let mut result = Matrix::<f32, 4, 4>::from_dh_param(1.0, 2.0, 3.0, 4.0);
        result.as_mut()[0][0] = 1.0;
        assert_eq!(
            *result.as_ref(),
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0]
            ]
        );
    }
}
