#![no_std]
//! This crate is for robotic math.
//!
#![allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use core::result::Result;
use core::time::Duration;
use num_traits::real::Real;

pub mod matrix;
pub use matrix::Matrix;
pub mod matrix3;
pub mod vector3;

// use heapless::Vec;

// pub trait Interface {
//     fn write_byte(&mut self, data: u8);
//     fn read_byte(&mut self) -> Option<u8>;
// }
// pub trait Clock {
//     fn get_current_time(&self) -> Duration;
// }

#[allow(dead_code)]
pub struct Quaternion {
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
}

impl Quaternion {
    pub fn new() -> Self {
        Self {
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
        }
    }
    pub fn x(self) -> f32 {
        self.q0
    }
}

pub struct DH {}

impl DH {
    pub fn transform_mat(a: f32, alfa: f32, d: f32, theta: f32) -> Matrix<f32, 4, 4> {
        let mut t = Matrix::<f32, 4, 4>::new();
        t = t * DH::mat_trans_x(a);

        t
    }
    fn mat_trans_x(d: f32) -> Matrix<f32, 4, 4> {
        let mut t = Matrix::<f32, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
    fn mat_rot_x(d: f32) -> Matrix<f32, 4, 4> {
        let mut t = Matrix::<f32, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
    fn mat_trans_z(d: f32) -> Matrix<f32, 4, 4> {
        let mut t = Matrix::<f32, 4, 4>::identity();
        t.as_mut()[2][3] = d;
        t
    }
    fn mat_rot_z(d: f32) -> Matrix<f32, 4, 4> {
        let mut t = Matrix::<f32, 4, 4>::identity();
        t.as_mut()[0][3] = d;
        t
    }
}

#[cfg(test)]
mod tests {
    use crate::Quaternion;

    #[test]
    fn init_quaternion() {
        let q = Quaternion::new();
        assert_eq!(q.q0, 1.0);
        assert_eq!(q.q1, 0.0);
        assert_eq!(q.q2, 0.0);
        assert_eq!(q.q3, 0.0);
    }

    #[test]
    fn quaternion() {
        let q = Quaternion::new();
        assert_eq!(q.x(), 1.0);
    }

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}

#[cfg(test)]
mod dh_param_tests {
    use crate::Matrix;
    use crate::DH;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
    #[test]
    fn init_transform_mat() {
        let mut result = DH::transform_mat(1.0, 2.0, 3.0, 4.0);
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
