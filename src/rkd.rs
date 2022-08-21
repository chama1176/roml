#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::Vec;

use crate::link::Link;

/// Robot Kinematics and Dynamics
pub struct Rkd<T> {
    pub links: Vec<Link<T>, 256>,
}

impl<T: na::RealField> Rkd<T> {
    fn new() -> Self {
        Self { links: Vec::new() }
    }
    fn update_kinematic_relationship(&mut self) {
        self.links[0].r_quat = na::UnitQuaternion::identity();
        self.links[0].p = na::Vector3::<T>::zeros();
        self.links[0].w_vec = na::Vector3::zeros();
        self.links[0].dpdt = na::Vector3::<T>::zeros();
        self.links[0].dwdt_vec = na::Vector3::zeros();
        self.links[0].ddpddt = na::Vector3::zeros();
        self.links[0].s = self.links[0].com.clone();

        for i in 1..self.links.len() {
            let pi = self.links[i].parent as usize;
            self.links[i].r_quat = na::UnitQuaternion::new_normalize(
                self.links[pi].r_quat.as_ref()
                    * na::UnitQuaternion::from_axis_angle(
                        &self.links[i].a,
                        self.links[i].q.clone(),
                    )
                    .as_ref(),
            );
            self.links[i].p =
                self.links[pi].p.clone() + self.links[pi].r_quat.transform_vector(&self.links[i].b);

            self.links[i].w_vec = self.links[pi].w_vec.clone()
                + self.links[pi]
                    .r_quat
                    .transform_vector(&(self.links[i].a.as_ref() * self.links[i].dqdt.clone()));
            self.links[i].dpdt = self.links[pi].dpdt.clone()
                + self.links[pi].w_vec.cross(
                    &self.links[pi]
                        .r_quat
                        .transform_vector(&self.links[i].b.clone()),
                );

            self.links[i].dwdt_vec = self.links[pi].dwdt_vec.clone()
                + self.links[pi]
                    .r_quat
                    .transform_vector(&(self.links[i].a.as_ref() * self.links[i].ddqddt.clone()))
                + self.links[pi].w_vec.cross(
                    &self.links[pi]
                        .r_quat
                        .transform_vector(&(self.links[i].a.as_ref() * self.links[i].dqdt.clone())),
                );
            self.links[i].ddpddt = self.links[pi].ddpddt.clone()
                + self.links[pi].dwdt_vec.cross(
                    &self.links[pi]
                        .r_quat
                        .transform_vector(&self.links[i].b.clone()),
                )
                + self.links[pi].dwdt_vec.cross(
                    &self.links[pi].dwdt_vec.cross(
                        &self.links[pi]
                            .r_quat
                            .transform_vector(&self.links[i].b.clone()),
                    ),
                );

            self.links[i].s = self.links[pi].s.clone()
                + self.links[i].r_quat.transform_vector(&self.links[i].com);

            // self.links[i].dpdt = self.links[pi].dpdt.clone_owned()
        }
    }
}

#[cfg(test)]
mod test_rkd {
    use crate::link::Link;
    use crate::rkd::Rkd;

    #[test]
    fn init() {
        let mut rkd = Rkd::<f32>::new();
        let l0 = Link::new();
        l0.r_quat;
        l0.b;
        let l1 = Link::new();
        let l2 = Link::new();
        rkd.links.push(l0).unwrap();
        rkd.links.push(l1).unwrap();
        rkd.links.push(l2).unwrap();

        rkd.update_kinematic_relationship();
    }
}
