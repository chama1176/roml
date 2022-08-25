#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::{Deque, Vec};

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
        // Currently id 0 must be root link.
        self.links[0].r_quat = na::UnitQuaternion::identity();
        self.links[0].p = na::Vector3::<T>::zeros();
        self.links[0].w_vec = na::Vector3::zeros();
        self.links[0].dwdt_vec = na::Vector3::zeros();
        self.links[0].ddpddt = na::Vector3::zeros();
        self.links[0].ddsddt = na::Vector3::zeros();

        let mut ids = Deque::<u8, 256>::new();
        for id in self.links[0].children.clone() {
            ids.push_back(id).unwrap();
        }

        // DFS
        while ids.len() > 0 {
            let i = ids.pop_back().unwrap() as usize;
            for id in self.links[i].children.clone() {
                ids.push_back(id).unwrap();
            }

            let pi = self.links[i].parent as usize;
            let pr = na::UnitQuaternion::from_axis_angle(&self.links[i].a, self.links[i].q.clone()); // lotation from parent to i

            self.links[i].r_quat = self.links[pi].r_quat.clone() * pr.clone();
            self.links[i].p =
                self.links[pi].p.clone() + self.links[pi].r_quat.transform_vector(&self.links[i].b);

            self.links[i].w_vec = pr.inverse().transform_vector(&self.links[pi].w_vec)
                + self.links[i].a.clone().into_inner() * self.links[i].dqdt.clone();
            self.links[i].dwdt_vec = pr.inverse().transform_vector(&self.links[pi].dwdt_vec)
                + self.links[i].a.clone().into_inner() * self.links[i].ddqddt.clone()
                + pr.inverse()
                    .transform_vector(&self.links[pi].w_vec)
                    .cross(&(self.links[i].a.clone().into_inner() * self.links[i].dqdt.clone()));

            self.links[i].ddpddt = pr.inverse().transform_vector(
                &(self.links[pi].ddpddt.clone()
                    + self.links[pi].dwdt_vec.cross(&self.links[i].b.clone())
                    + self.links[pi]
                        .w_vec
                        .cross(&self.links[pi].w_vec.cross(&self.links[i].b.clone()))),
            );
            self.links[i].ddsddt = self.links[i].ddpddt.clone()
                + self.links[i].dwdt_vec.cross(&self.links[i].com)
                + self.links[i]
                    .w_vec
                    .cross(&self.links[i].w_vec.cross(&self.links[i].com));
        }
    }

    fn update_equation_of_motion(&mut self) {
        let mut f_hat = Vec::<na::Vector3<T>, 256>::new();
        f_hat
            .resize(self.links.len(), na::Vector3::zeros())
            .unwrap();
        let mut n_hat = Vec::<na::Vector3<T>, 256>::new();
        n_hat
            .resize(self.links.len(), na::Vector3::zeros())
            .unwrap();

        for i in 0..self.links.len() {
            f_hat[i] = self.links[i].ddsddt.clone() * self.links[i].mass.clone();
            n_hat[i] = self.links[i].inertia_mat.clone() * self.links[i].dwdt_vec.clone()
                + self.links[i]
                    .w_vec
                    .cross(&(self.links[i].inertia_mat.clone() * self.links[i].w_vec.clone()));
        }

        let mut ids = Deque::<u8, 256>::new();
        let mut seen = Vec::<bool, 256>::new();
        seen.resize(self.links.len(), false).unwrap();
        ids.push_back(0).unwrap();
        for id in self.links[0].children.clone() {
            ids.push_back(id).unwrap();
        }
        seen[0] = true;

        let mut f = Vec::<na::Vector3<T>, 256>::new();
        f.resize(self.links.len(), na::Vector3::zeros()).unwrap();
        let mut n = Vec::<na::Vector3<T>, 256>::new();
        n.resize(self.links.len(), na::Vector3::zeros()).unwrap();

        // DFS return order
        while ids.len() > 0 {
            let i = *ids.back().unwrap() as usize;
            let mut flag = true;
            for c in self.links[i].children.clone() {
                if seen[c as usize] == false {
                    ids.push_back(c).unwrap();
                    flag = false;
                }
            }
            seen[i] = true;
            if flag == false {
                continue;
            }
            f[i] = f_hat[i].clone();
            n[i] = n_hat[i].clone() + self.links[i].com.cross(&f_hat[i]);
            for c in self.links[i].children.clone() {
                let r = na::UnitQuaternion::from_axis_angle(
                    &self.links[c as usize].a,
                    self.links[c as usize].q.clone(),
                ); // lotation from i to c
                let tmp = r.inverse().transform_vector(&f[c as usize]); // 👺逆かも
                f[i] += tmp;
                let tmp = r.inverse().transform_vector(&n[c as usize])
                    + self.links[c as usize].b.cross(&r.inverse().transform_vector(&f[c as usize]));
            }
            ids.pop_back().unwrap();
        }
        let mut t = Vec::<na::Vector3<T>, 256>::new();
        t.resize(self.links.len(), na::Vector3::zeros()).unwrap();
        // t =
    }
}

#[cfg(test)]
mod test_rkd {
    use crate::link::Link;
    use crate::rkd::Rkd;

    #[test]
    fn init() {
        let mut rkd = Rkd::<f32>::new();
        let mut l0 = Link::new();
        l0.parent = 0;
        rkd.links.push(l0).unwrap();
        let mut l1 = Link::new();
        l1.parent = 0;
        l1.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l1).unwrap();
        let mut l2 = Link::new();
        l2.parent = 0;
        l2.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l2).unwrap();
        let mut l3 = Link::new();
        l3.parent = 1;
        l3.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l3).unwrap();
        let mut l4 = Link::new();
        l4.parent = 1;
        l4.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l4).unwrap();

        for i in 0..rkd.links.len() {
            rkd.links[i].id = i as u8;
            let pid = rkd.links[i].parent;
            let id = rkd.links[i].id;
            if pid != id {
                rkd.links[pid as usize].children.push(id).unwrap();
            }
        }
        assert_eq!(*rkd.links[0].children, [1, 2]);
        assert_eq!(*rkd.links[1].children, [3, 4]);
        assert_eq!(*rkd.links[2].children, []);
        assert_eq!(*rkd.links[3].children, []);
        assert_eq!(*rkd.links[4].children, []);

        rkd.update_kinematic_relationship();
        assert_eq!(rkd.links[0].p, na::Vector3::zeros());
        assert_eq!(rkd.links[1].p, na::Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(rkd.links[2].p, na::Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(rkd.links[3].p, na::Vector3::new(0.0, 0.0, 2.0));
        assert_eq!(rkd.links[4].p, na::Vector3::new(0.0, 0.0, 2.0));

        rkd.update_equation_of_motion();
    }

    #[test]
    fn lane2d() {
        let mut rkd = Rkd::<f32>::new();
        let mut l0 = Link::new();
        l0.parent = 0;
        rkd.links.push(l0).unwrap();
        let mut l1 = Link::new();
        l1.parent = 0;
        l1.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l1).unwrap();
        let mut l2 = Link::new();
        l2.parent = 0;
        l2.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l2).unwrap();
        let mut l3 = Link::new();
        l3.parent = 1;
        l3.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l3).unwrap();
        let mut l4 = Link::new();
        l4.parent = 1;
        l4.b = na::Vector3::new(0.0, 0.0, 1.0);
        rkd.links.push(l4).unwrap();

        for i in 0..rkd.links.len() {
            rkd.links[i].id = i as u8;
            let pid = rkd.links[i].parent;
            let id = rkd.links[i].id;
            if pid != id {
                rkd.links[pid as usize].children.push(id).unwrap();
            }
        }
        assert_eq!(*rkd.links[0].children, [1, 2]);
        assert_eq!(*rkd.links[1].children, [3, 4]);
        assert_eq!(*rkd.links[2].children, []);
        assert_eq!(*rkd.links[3].children, []);
        assert_eq!(*rkd.links[4].children, []);

        rkd.update_kinematic_relationship();
        assert_eq!(rkd.links[0].p, na::Vector3::zeros());
        assert_eq!(rkd.links[1].p, na::Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(rkd.links[2].p, na::Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(rkd.links[3].p, na::Vector3::new(0.0, 0.0, 2.0));
        assert_eq!(rkd.links[4].p, na::Vector3::new(0.0, 0.0, 2.0));

        rkd.update_equation_of_motion();
    }


}
