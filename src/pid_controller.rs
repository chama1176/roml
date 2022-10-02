#[allow(unused_imports)]

pub trait PIDController<T> {
    fn new() -> Self;
    fn update(&mut self, r: T, y: T) -> T;
}

#[derive(Debug, Clone, Copy)]
pub struct PID<T> {
    pub kp: T,
    pub ki: T,
    pub kd: T,
    pub i_cramp: T,
    last_e: T,
    e_sum: T,
    dt: T,
}

impl<T: na::RealField> PIDController<T> for PID<T> {
    fn new() -> Self {
        Self {
            kp: T::zero(),
            ki: T::zero(),
            kd: T::zero(),
            i_cramp: T::from_f32(1.).unwrap(),
            last_e: T::zero(),
            e_sum: T::zero(),
            dt: T::one(),
        }
    }

    /// r: Target value
    /// y: Present value
    fn update(&mut self, r: T, y: T) -> T {
        // P.77あたりの式
        let mut u = T::zero();
        let e = r - y;
        self.e_sum += e.clone();
        u += self.kp.clone() * e.clone();
        let iv = self.ki.clone() * self.e_sum.clone();
        if self.i_cramp < iv {
            u += self.i_cramp.clone();
        } else if iv < -self.i_cramp.clone() {
            u += -self.i_cramp.clone();
        } else {
            u += iv;
        }
        u += self.kd.clone() * (e.clone() - self.last_e.clone());

        self.last_e = e;

        u
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PIDVelForm<T> {
    pub kp: T,
    pub ki: T,
    pub kd: T,
    last_u: T,
    e_1: T,
    e_2: T,
    dt: T,
}

impl<T: na::RealField> PIDController<T> for PIDVelForm<T> {
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
        self.last_u = u.clone();

        u
    }
}

#[cfg(test)]
mod test_ik {
    use crate::pid_controller::*;
    use approx::assert_relative_eq;

    #[test]
    fn velocity_type_pid() {
        let mut pid = PID::<f32>::new();
        pid.kp = 10.;
        let u = pid.update(5., 3.);
        assert_relative_eq!(u, 20.0, epsilon = 1.0e-6);
        let u = pid.update(5., 3.);
        assert_relative_eq!(u, 20.0, epsilon = 1.0e-6);
    }
}
