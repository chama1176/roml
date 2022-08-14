#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use num_traits::real::Real;

use crate::matrix::Matrix;

pub trait Quaternion<T> {
    fn new() ->  Matrix<T, 4, 1>;
    fn x(&self) -> &T;
    fn y(&self) -> &T;
    fn z(&self) -> &T;
    fn w(&self) -> &T;
    fn q0(&self) -> &T;
    fn q1(&self) -> &T;
    fn q2(&self) -> &T;
    fn q3(&self) -> &T;
}

impl<T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T> + Neg<Output = T>> Quaternion<T> for Matrix<T, 4, 1> {
    fn new() -> Matrix<T, 4, 1> {
        Matrix::<T, 4, 1>::from([[T::from(1.0)],[T::from(0.0)],[T::from(0.0)],[T::from(0.0)]])
    }
    fn w(&self) -> &T {
        &self.as_ref()[0][0]
    }
    fn x(&self) -> &T {
        &self.as_ref()[1][0]
    }
    fn y(&self) -> &T {
        &self.as_ref()[2][0]
    }
    fn z(&self) -> &T {
        &self.as_ref()[3][0]
    }
    fn q0(&self) -> &T {
        &self.as_ref()[0][0]
    }
    fn q1(&self) -> &T {
        &self.as_ref()[1][0]
    }
    fn q2(&self) -> &T {
        &self.as_ref()[2][0]
    }
    fn q3(&self) -> &T {
        &self.as_ref()[3][0]
    }
}

#[cfg(test)]
mod tests {
    use crate::matrix::Matrix;
    use crate::quaternion::Quaternion;

    #[test]
    fn init_quaternion() {
        let q = <Matrix<f32, 4, 1> as Quaternion<f32>>::new();
        assert_eq!(*q.q0(), 1.0);
        assert_eq!(*q.q1(), 0.0);
        assert_eq!(*q.q2(), 0.0);
        assert_eq!(*q.q3(), 0.0);
    }

    #[test]
    fn quaternion() {
        let q = <Matrix<f32, 4, 1> as Quaternion<f32>>::new();
        assert_eq!(*q.w(), 1.0);
    }

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
