
#[cfg(test)]
mod tests {

    #[test]
    fn init_quaternion() {
        let q = na::Quaternion::<f32>::identity();
        assert_eq!(q[0], 0.0);
        assert_eq!(q[1], 0.0);
        assert_eq!(q[2], 0.0);
        assert_eq!(q[3], 1.0);
    }

}
