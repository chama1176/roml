#![no_std]
//! This crate is for robotic math.
//!
#![allow(unused_imports)]
use core::result::Result;
use core::time::Duration;
use core::ops::{Add, AddAssign, Mul};
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


pub struct Matrix<T, const ROWS: usize, const COLS: usize> {
    data: [[T; COLS]; ROWS],
}

impl<T: From<f32>+Copy, const ROWS: usize, const COLS: usize> Matrix<T, ROWS, COLS>{
    pub fn new() -> Self {
        Self {
            data: [[T::from(0.0); COLS]; ROWS]
        }
    }
}

impl<T: From<f32>+Copy+Add<Output=T>+AddAssign+Mul<Output=T>, const ROWS: usize, const COLS: usize, const MULCOLS: usize> Mul<Matrix<T, COLS, MULCOLS>> for Matrix<T, ROWS, COLS> {
    // The multiplication of rational numbers is a closed operation.
    type Output = Matrix<T, ROWS, MULCOLS>;

    fn mul(self, rhs: Matrix<T, COLS, MULCOLS>) -> Self::Output {
        let mut result = Matrix::<T, ROWS, MULCOLS>::new();
        for i in 0..ROWS{
            for j in 0.. MULCOLS{
                for k in 0..COLS {
                    result.data[i][j] += self.data[i][k] * rhs.data[k][j];
                }
            }
        }

        result
    }
}


#[cfg(test)]
mod test_mat {
    use crate::Matrix;

    #[test]
    fn init_mat() {
        let m = Matrix::<f32, 2, 3>::new();
        assert_eq!(m.data[0][0], 0.0);
        assert_eq!(m.data[0][1], 0.0);
        assert_eq!(m.data[0][2], 0.0);
        assert_eq!(m.data[1][0], 0.0);
        assert_eq!(m.data[1][1], 0.0);
        assert_eq!(m.data[1][2], 0.0);
    }

    #[test]
    fn mul_mat() {
        let mut a = Matrix::<f32, 2, 3>::new();
        a.data[0][0] = 1.0;
        a.data[0][1] = 2.0;
        a.data[0][2] = 3.0;
        a.data[1][0] = 4.0;
        a.data[1][1] = 5.0;
        a.data[1][2] = 6.0;

        let mut b = Matrix::<f32, 3, 2>::new();
        b.data = [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]];

        let c = a*b;
        assert_eq!(c.data[0][0], 22.0);
        assert_eq!(c.data[0][1], 28.0);
        assert_eq!(c.data[1][0], 49.0);
        assert_eq!(c.data[1][1], 64.0);
    }


}
