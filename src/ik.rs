#[allow(unused_imports)]

/// Robot Kinematics and Dynamics
pub struct IK4dTriangle<T> {
    pub a: T,
    pub b: T,
    pub ref_theta: T,
}

impl<T: na::RealField> IK4dTriangle<T> {
    pub fn new() -> Self {
        Self {
            a: T::zero(),
            b: T::zero(),
            ref_theta: T::zero(),
        }
    }

    pub fn solve(&self, vc: &na::Vector3<T>) -> [T; 4] {
        let mut ans = [T::zero(), T::zero(), T::zero(), T::zero()];
        let c_squared =
            vc.x.clone() * vc.x.clone() + vc.y.clone() * vc.y.clone() + vc.z.clone() * vc.z.clone();
        let c = c_squared.sqrt();
        // cosine formula
        ans[3] = ((self.a.clone() * self.a.clone() + self.b.clone() * self.b.clone() - c.clone() * c.clone())
            / (T::from_f32(2.0).unwrap() * self.a.clone() * self.b.clone()))
        .acos();

        let b_theta = ((self.a.clone() * self.a.clone() + c.clone() * c.clone() - self.b.clone()*self.b.clone())
        / (T::from_f32(2.0).unwrap() * self.a.clone() * c.clone())).acos();
        let angle_cz = (vc.z.clone() / c.clone()).acos();   // angle between vc and z axis

        // let rot_cz = vc.cross(&na::Vector3::z_axis());

        let r_z = self.a.clone() * (angle_cz.clone() - b_theta.clone()).cos();
        let ratio = (self.a.clone() *(angle_cz.clone() - b_theta.clone()).sin()) /(c.clone() * angle_cz.sin());
        let r_x = vc.x.clone() * ratio.clone();
        let r_y = vc.y.clone() * ratio;
        let r = na::Vector3::<T>::new(r_x, r_y, r_z);

        let rot_ref_theta = na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(vc.clone()), self.ref_theta.clone());
        let r_dot = rot_ref_theta.transform_vector(&r);
        ans[0] = r_dot.y.clone().atan2(r_dot.x.clone());
        ans[1] = (r_dot.z.clone() / c.clone()).acos();
        ans[2] = self.ref_theta.clone();

        ans
    }
}

#[cfg(test)]
mod test_ik {
    use crate::ik::IK4dTriangle;
    use approx::assert_relative_eq;

    #[test]
    fn ik_4dof_triangle() {
        let mut ik = IK4dTriangle::<f32>::new();
        ik.a = 3.0;
        ik.b = 3.0;
        ik.ref_theta = 0.0;
        let ans = ik.solve(&na::Vector3::new(3.0, 0.0, 0.0));

        assert_eq!(ans[0], 0.0);
        assert_eq!(ans[1], 0.0);
        assert_eq!(ans[2], 0.0);
        assert_relative_eq!(ans[3], core::f32::consts::PI / 3.0, epsilon = 1.0e-6);
    }
}
