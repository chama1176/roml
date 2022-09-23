#[allow(unused_imports)]

pub trait PIDController<T> {
    fn new() -> Self;
    fn update(&mut self, r: T, y: T) -> T;
}
/// Robot Kinematics and Dynamics
pub struct PID<T> {
    pub kp: T,
    pub ki: T,
    pub kd: T,
    last_u: T,
    e_1: T,
    e_2: T,
    dt: T,
}

impl<T: na::RealField> PIDController<T> for PID<T> {
    fn new() -> Self {
        Self {
            kp: T::zero(),
            ki: T::zero(),
            kd: T::zero(),
            last_u: T::zero(),
            e_1: T::zero(),
            e_2: T::zero(),
            dt: T::one(),
        }
    }

    /// r: Target value
    /// y: Present value
    fn update(&mut self, r: T, y: T) -> T {
        // P.77あたりの式
        let mut u = self.last_u.clone();
        let e = r - y;
        u += self.kp.clone() * (e.clone() - self.e_1.clone());
        u += self.ki.clone() * self.dt.clone() * e.clone();
        u += self.kd.clone()
            * (e.clone() - T::from_f32(2.0).unwrap() * self.e_1.clone() + self.e_2.clone())
            / self.dt.clone();

        self.e_2 = self.e_1.clone();
        self.e_1 = e;

        u
    }
}

#[cfg(test)]
mod test_ik {
    use crate::pid_controller::PIDController;
    use approx::assert_relative_eq;

    #[test]
    fn velocity_type_pid() {
        assert_relative_eq!(1.0 + 1.0, 2.0, epsilon = 1.0e-6);
    }
}
