use core::ops::{Add, AddAssign, Mul, Neg};
use core::result::Result;
use core::time::Duration;

#[derive(Debug, Copy, Clone)]
pub struct Matrix<T, const ROWS: usize, const COLS: usize> {
    data: [[T; COLS]; ROWS],
}

impl<'a, T: From<f32> + Copy, const ROWS: usize, const COLS: usize> Matrix<T, ROWS, COLS> {
    pub fn new() -> Self {
        Self {
            data: [[T::from(0.0); COLS]; ROWS],
        }
    }
    pub fn from(data: [[T; COLS]; ROWS]) -> Self {
        Self { data: data }
    }
    pub fn identity() -> Self {
        assert_eq!(ROWS, COLS);
        let mut data = [[T::from(0.0); COLS]; ROWS];
        for i in 0..COLS {
            data[i][i] = T::from(1.0);
        }
        Self { data: data }
    }

    pub fn transpose(&self) -> Matrix<T, COLS, ROWS> {
        let mut result = Matrix::<T, COLS, ROWS>::new();
        for i in 0..ROWS {
            for j in 0..COLS {
                result.data[j][i] = self.data[i][j];
            }
        }
        result
    }

    pub fn as_mut(&mut self) -> &mut [[T; COLS]; ROWS] {
        &mut self.data
    }

    pub fn as_ref(&self) -> &[[T; COLS]; ROWS] {
        &self.data
    }
}

impl<
        T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T>,
        const ROWS: usize,
        const COLS: usize,
        const MULCOLS: usize,
    > Mul<Matrix<T, COLS, MULCOLS>> for Matrix<T, ROWS, COLS>
{
    // The multiplication of rational numbers is a closed operation.
    type Output = Matrix<T, ROWS, MULCOLS>;

    fn mul(self, rhs: Matrix<T, COLS, MULCOLS>) -> Self::Output {
        let mut result = Matrix::<T, ROWS, MULCOLS>::new();
        for i in 0..ROWS {
            for j in 0..MULCOLS {
                for k in 0..COLS {
                    result.data[i][j] += self.data[i][k] * rhs.data[k][j];
                }
            }
        }

        result
    }
}

impl<
        T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T>,
        const ROWS: usize,
        const COLS: usize,
    > Mul<T> for Matrix<T, ROWS, COLS>
{
    // The multiplication of rational numbers is a closed operation.
    type Output = Matrix<T, ROWS, COLS>;

    fn mul(self, rhs: T) -> Self::Output {
        let mut result = Matrix::<T, ROWS, COLS>::new();
        for i in 0..ROWS {
            for j in 0..COLS {
                result.data[i][j] = self.data[i][j] * rhs;
            }
        }

        result
    }
}

// impl<
//         T: From<f32> + Copy + Add<Output = T> + AddAssign + Mul<Output = T>,
//         const ROWS: usize,
//         const COLS: usize,
//     > Mul<Matrix<T, ROWS, COLS>> for f32
// {
//     // The multiplication of rational numbers is a closed operation.
//     type Output = Matrix<T, ROWS, COLS>;

//     fn mul(self, rhs: Matrix<T, ROWS, COLS>) -> Self::Output {
//         let mut result = Matrix::<T, ROWS, COLS>::new();
//         for i in 0..ROWS {
//             for j in 0..COLS {
//                 result.data[i][j] = T::from(self) * rhs.data[i][j];
//             }
//         }

//         result
//     }
// }

impl<T: From<f32> + Copy + Add<Output = T> + AddAssign, const ROWS: usize, const COLS: usize>
    Add<Matrix<T, ROWS, COLS>> for Matrix<T, ROWS, COLS>
{
    // The multiplication of rational numbers is a closed operation.
    type Output = Matrix<T, ROWS, COLS>;

    fn add(self, rhs: Matrix<T, ROWS, COLS>) -> Self::Output {
        let mut result = Matrix::<T, ROWS, COLS>::new();
        for i in 0..ROWS {
            for j in 0..COLS {
                result.data[i][j] = self.data[i][j] + rhs.data[i][j];
            }
        }
        result
    }
}

#[cfg(test)]
mod test_mat {
    use crate::matrix::Matrix;

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
    fn identity_mat() {
        let m = Matrix::<f32, 2, 2>::identity();
        assert_eq!(m.data[0][0], 1.0);
        assert_eq!(m.data[0][1], 0.0);
        assert_eq!(m.data[1][0], 0.0);
        assert_eq!(m.data[1][1], 1.0);
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

        let c = a * b;
        assert_eq!(c.data[0][0], 22.0);
        assert_eq!(c.data[0][1], 28.0);
        assert_eq!(c.data[1][0], 49.0);
        assert_eq!(c.data[1][1], 64.0);
    }

    #[test]
    fn mul_scaler_mat() {
        let a: f32 = 2.0;

        // let mut b = Matrix::<f32, 2, 3>::new();
        // b.data = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        // let c = a * b;
        // assert_eq!(c.data, [[2.0, 4.0, 6.0], [8.0, 10.0, 12.0]]);

        let mut b = Matrix::<f32, 2, 3>::new();
        b.data = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        let c = b * a;
        assert_eq!(c.data, [[2.0, 4.0, 6.0], [8.0, 10.0, 12.0]]);
    }

    #[test]
    fn add_mat() {
        let mut a = Matrix::<f32, 2, 3>::new();
        a.data = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        let mut b = Matrix::<f32, 2, 3>::new();
        b.data = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];

        let c = a + b;
        assert_eq!(c.data, [[2.0, 4.0, 6.0], [8.0, 10.0, 12.0]]);
    }

    #[test]
    fn transpose_mat() {
        let mut a = Matrix::<f32, 2, 3>::new();
        a.data = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];

        let c = a.transpose();
        assert_eq!(c.data, [[1.0, 4.0], [2.0, 5.0], [3.0, 6.0]]);
    }

    #[test]
    fn transform_position() {
        let mut pos_a = Matrix::<f32, 3, 1>::new();
        pos_a.data = [[4.0], [5.0], [6.0]];

        let e = Matrix::<f32, 3, 3>::identity();
        let pos = e * pos_a;
        assert_eq!(pos.data, [[4.0], [5.0], [6.0]]);
    }
}
