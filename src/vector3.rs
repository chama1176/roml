#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use num_traits::real::Real;

use crate::matrix::Matrix;

pub trait Vector3<T> {
    fn unit_x() -> Matrix<T, 3, 1>;
    fn unit_y() -> Matrix<T, 3, 1>;
    fn unit_z() -> Matrix<T, 3, 1>;
    fn as_mut_x(&mut self) -> &mut T;
    fn as_mut_y(&mut self) -> &mut T;
    fn as_mut_z(&mut self) -> &mut T;
    fn x(&self) -> &T;
    fn y(&self) -> &T;
    fn z(&self) -> &T;
    fn as_skew_symmetric_mat(&self) -> Matrix<T, 3, 3>;
}

impl<'a, T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T> + Neg<Output = T>>
    Vector3<T> for Matrix<T, 3, 1>
{
    fn unit_x() -> Matrix<T, 3, 1> {
        Matrix::<T, 3, 1>::from([[T::from(1.0)], [T::from(0.0)], [T::from(0.0)]])
    }
    fn unit_y() -> Matrix<T, 3, 1> {
        Matrix::<T, 3, 1>::from([[T::from(0.0)], [T::from(1.0)], [T::from(0.0)]])
    }
    fn unit_z() -> Matrix<T, 3, 1> {
        Matrix::<T, 3, 1>::from([[T::from(0.0)], [T::from(0.0)], [T::from(1.0)]])
    }
    fn as_mut_x(&mut self) -> &mut T {
        &mut self.as_mut()[0][0]
    }
    fn as_mut_y(&mut self) -> &mut T {
        &mut self.as_mut()[1][0]
    }
    fn as_mut_z(&mut self) -> &mut T {
        &mut self.as_mut()[2][0]
    }
    fn x(&self) -> &T {
        &self.as_ref()[0][0]
    }
    fn y(&self) -> &T {
        &self.as_ref()[1][0]
    }
    fn z(&self) -> &T {
        &self.as_ref()[2][0]
    }
    fn as_skew_symmetric_mat(&self) -> Matrix<T, 3, 3> {
        Matrix::<T, 3, 3>::from([
            [T::from(0.0), -*self.z(), *self.y()],
            [*self.z(), T::from(0.0), -*self.x()],
            [-*self.y(), *self.x(), T::from(0.0)],
        ])
    }
}

#[cfg(test)]
mod test_mat {

    #[test]
    fn init_mat() {
        let v = na::Vector3::<f32>::x();
        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 0.0);
        assert_eq!(v[2], 0.0);
        assert_eq!(v[(0,0)], 1.0);
        assert_eq!(v[(1,0)], 0.0);
        assert_eq!(v[(2,0)], 0.0);
    }

    #[test]
    fn skew_symmetric_mat() {
        let v = na::Vector3::<f32>::new(1.0, 2.0, 3.0);
        let sksm = v.cross_matrix();
        assert_eq!(sksm[(0,0)],  0.0);
        assert_eq!(sksm[(0,1)], -3.0);
        assert_eq!(sksm[(0,2)],  2.0);
        assert_eq!(sksm[(1,0)],  3.0);
        assert_eq!(sksm[(1,1)],  0.0);
        assert_eq!(sksm[(1,2)], -1.0);
        assert_eq!(sksm[(2,0)], -2.0);
        assert_eq!(sksm[(2,1)],  1.0);
        assert_eq!(sksm[(2,2)],  0.0);
    }
}
