#[cfg(test)]
mod test_mat {

    #[test]
    fn init_mat() {
        let v = na::Vector3::<f32>::x();
        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 0.0);
        assert_eq!(v[2], 0.0);
        assert_eq!(v[(0, 0)], 1.0);
        assert_eq!(v[(1, 0)], 0.0);
        assert_eq!(v[(2, 0)], 0.0);
    }

    #[test]
    fn skew_symmetric_mat() {
        let v = na::Vector3::<f32>::new(1.0, 2.0, 3.0);
        let sksm = v.cross_matrix();
        assert_eq!(sksm[(0, 0)], 0.0);
        assert_eq!(sksm[(0, 1)], -3.0);
        assert_eq!(sksm[(0, 2)], 2.0);
        assert_eq!(sksm[(1, 0)], 3.0);
        assert_eq!(sksm[(1, 1)], 0.0);
        assert_eq!(sksm[(1, 2)], -1.0);
        assert_eq!(sksm[(2, 0)], -2.0);
        assert_eq!(sksm[(2, 1)], 1.0);
        assert_eq!(sksm[(2, 2)], 0.0);
    }
}
