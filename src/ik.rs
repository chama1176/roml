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
        ans[3] = T::pi()
            - ((self.a.clone() * self.a.clone() + self.b.clone() * self.b.clone()
                - c.clone() * c.clone())
                / (T::from_f32(2.0).unwrap() * self.a.clone() * self.b.clone()))
            .acos();

        // angle between vc and va
        let b_theta = ((self.a.clone() * self.a.clone() + c.clone() * c.clone()
            - self.b.clone() * self.b.clone())
            / (T::from_f32(2.0).unwrap() * self.a.clone() * c.clone()))
        .acos();

        // rotation from axis vertial to z axis and vc👺
        let rot_cz = na::UnitQuaternion::from_axis_angle(
            &na::Unit::new_normalize(vc.cross(&na::Vector3::z_axis())),
            b_theta.clone(),
        );
        let va_dot = rot_cz.transform_vector(&vc) * self.a.clone() / c.clone();
        let vb_dot = vc.clone() - va_dot.clone();

        // rotation along with vector c
        let rot_ref_theta = na::UnitQuaternion::from_axis_angle(
            &na::Unit::new_normalize(vc.clone()),
            self.ref_theta.clone(),
        );
        let va = rot_ref_theta.transform_vector(&va_dot);
        let vb = rot_ref_theta.transform_vector(&vb_dot);
        ans[0] = va.y.clone().atan2(va.x.clone());
        ans[1] = (va.z.clone() / self.a.clone()).acos();

        // let rot_ans0 = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), ans[0].clone());
        // let rot_ans1 = na::UnitQuaternion::from_axis_angle(
        //     &na::Unit::new_normalize(vc.cross(&na::Vector3::z_axis())),
        //     ans[1].clone(),
        // );
        // let vb_dot_dot = rot_ans1.transform_vector(&rot_ans0.transform_vector(&vb_dot));
        // 👺
        let rot_ans3 = na::UnitQuaternion::from_axis_angle(
            &na::Unit::new_normalize(na::Vector3::z_axis().cross(&va)),
            ans[3].clone(),
        );
        let vb_dot_dot = rot_ans3.transform_vector(&va) * self.b.clone() / self.a.clone();

        // vector a component of b 👺
        let va_tmp = va.clone() * va.dot(&vb) / self.a.clone() / self.a.clone();
        let va_tmp2 = va.clone() * va.dot(&vb_dot_dot) / self.a.clone() / self.a.clone();
        // assert_eq!(va_tmp, va_tmp2);

        let vb_tmp = vb_dot_dot.clone() - va_tmp.clone();
        let vb_tmp2 = vb.clone() - va_tmp.clone();

        ans[2] = na::UnitQuaternion::rotation_between(&vb_tmp, &vb_tmp2)
            .unwrap()
            .angle();

        ans
        // 👺原点と方向の定義も必要
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
        assert_relative_eq!(ans[0], 0.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[1], core::f32::consts::PI / 6.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[2], 0.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[3], core::f32::consts::PI * 2.0 / 3.0, epsilon = 1.0e-6);
        ik.a = 3.0;
        ik.b = 3.0;
        ik.ref_theta = 0.0;
        let ans = ik.solve(&na::Vector3::new(
            3.0 / (2.0 as f32).sqrt(),
            3.0 / (2.0 as f32).sqrt(),
            0.0,
        ));
        assert_relative_eq!(ans[0], core::f32::consts::PI / 4.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[1], core::f32::consts::PI / 6.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[2], 0.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[3], core::f32::consts::PI * 2.0 / 3.0, epsilon = 1.0e-6);
        ik.a = 3.0;
        ik.b = 3.0;
        ik.ref_theta = core::f32::consts::PI / 2.0;
        let ans = ik.solve(&na::Vector3::new(3.0, 0.0, 0.0));
        assert_relative_eq!(ans[0], -core::f32::consts::PI / 3.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[1], core::f32::consts::PI / 2.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[2], core::f32::consts::PI / 2.0, epsilon = 1.0e-6);
        assert_relative_eq!(ans[3], core::f32::consts::PI * 2.0 / 3.0, epsilon = 1.0e-6);
    }
}
