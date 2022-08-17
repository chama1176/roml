
#[cfg(test)]
mod test_mat {

    #[test]
    fn init_mat() {
        let m = na::Matrix2x3::<f32>::zeros();        
        assert_eq!(m[(0,0)], 0.0);
        assert_eq!(m[(0,1)], 0.0);
        assert_eq!(m[(0,2)], 0.0);
        assert_eq!(m[(1,0)], 0.0);
        assert_eq!(m[(1,1)], 0.0);
        assert_eq!(m[(1,2)], 0.0);
    }

    #[test]
    fn identity_mat() {
        let m = na::Matrix2::<f32>::identity();
        assert_eq!(m[(0,0)], 1.0);
        assert_eq!(m[(0,1)], 0.0);
        assert_eq!(m[(1,0)], 0.0);
        assert_eq!(m[(1,1)], 1.0);
    }

    #[test]
    fn mul_mat() {
        let mut a = na::Matrix2x3::<f32>::zeros();
        a[(0,0)] = 1.0;
        a[(0,1)] = 2.0;
        a[(0,2)] = 3.0;
        a[(1,0)] = 4.0;
        a[(1,1)] = 5.0;
        a[(1,2)] = 6.0;

        let b = na::Matrix3x2::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        let c = a * b;
        assert_eq!(c[(0,0)], 22.0);
        assert_eq!(c[(0,1)], 28.0);
        assert_eq!(c[(1,0)], 49.0);
        assert_eq!(c[(1,1)], 64.0);
    }

    #[test]
    fn mul_scaler_mat() {
        let a: f32 = 2.0;
        let b = na::Matrix2x3::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let c = b * a;
        assert_eq!(c[(0,0)], 2.0);
        assert_eq!(c[(0,1)], 4.0);
        assert_eq!(c[(0,2)], 6.0);
        assert_eq!(c[(1,0)], 8.0);
        assert_eq!(c[(1,1)], 10.0);
        assert_eq!(c[(1,2)], 12.0);
        let d = a * b;
        assert_eq!(d[(0,0)], 2.0);
        assert_eq!(d[(0,1)], 4.0);
        assert_eq!(d[(0,2)], 6.0);
        assert_eq!(d[(1,0)], 8.0);
        assert_eq!(d[(1,1)], 10.0);
        assert_eq!(d[(1,2)], 12.0);
    }

    #[test]
    fn add_mat() {
        let a = na::Matrix2x3::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let b = na::Matrix2x3::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let c = a + b;
        assert_eq!(c[(0,0)], 2.0);
        assert_eq!(c[(0,1)], 4.0);
        assert_eq!(c[(0,2)], 6.0);
        assert_eq!(c[(1,0)], 8.0);
        assert_eq!(c[(1,1)], 10.0);
        assert_eq!(c[(1,2)], 12.0);
    }

    #[test]
    fn transpose_mat() {
        let a = na::Matrix2x3::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let c = a.transpose();
        assert_eq!(c[(0,0)], 1.0);
        assert_eq!(c[(0,1)], 4.0);
        assert_eq!(c[(1,0)], 2.0);
        assert_eq!(c[(1,1)], 5.0);
        assert_eq!(c[(2,0)], 3.0);
        assert_eq!(c[(2,1)], 6.0);
    }

    #[test]
    fn transform_position() {
        let pos_a = na::Matrix3x1::<f32>::new(4.0, 5.0, 6.0);
        let e = na::Matrix3::<f32>::identity();
        let pos = e * pos_a;
        assert_eq!(pos[(0,0)], 4.0);
        assert_eq!(pos[(1,0)], 5.0);
        assert_eq!(pos[(2,0)], 6.0);
    }
}
