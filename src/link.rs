#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::Vec;

/// 親リンクからつながる位置（回転軸）を原点とする
#[derive(Debug)]
pub struct Link<T> {
    pub id: u8,
    pub parent: u8,
    pub children: Vec<u8, 256>,
    pub mass: T,
    pub com: na::Vector3<T>,         // Centor of Mass, 自リンク原点中心
    pub inertia_mat: na::Matrix3<T>, // 自リンク原点まわり
    pub q: T,
    pub dqdt: T,
    pub ddqddt: T,
    pub q_min: T,
    pub q_max: T,
    pub p: na::Vector3<T>,             // position in world coordinate
    pub r_quat: na::UnitQuaternion<T>, // rotation quat in world coordinate

    pub ddpddt: na::Vector3<T>,      // position in local coordinate
    pub ddsddt: na::Vector3<T>,      // com acceleration in local coordinate
    pub w_vec: na::Vector3<T>,       // rotation vec in local coordinate
    pub dwdt_vec: na::Vector3<T>,    // rotation vec in local coordinate
    pub a: na::Unit<na::Vector3<T>>, // joint axis vec relative to parent link
    pub b: na::Vector3<T>,           // joint position relative to parent link
}

impl<T: na::RealField> Link<T> {
    pub fn new() -> Self {
        Self {
            id: 0,
            parent: 0,
            children: Vec::new(),
            mass: T::zero(),
            com: na::Vector3::<T>::zeros(),
            inertia_mat: na::Matrix3::zeros(),
            q: T::zero(),
            dqdt: T::zero(),
            ddqddt: T::zero(),
            q_min: T::zero(),
            q_max: T::zero(),
            p: na::Vector3::<T>::zeros(),
            r_quat: na::UnitQuaternion::identity(),

            ddpddt: na::Vector3::<T>::zeros(),
            ddsddt: na::Vector3::<T>::zeros(),
            w_vec: na::Vector3::zeros(),
            dwdt_vec: na::Vector3::zeros(),
            a: na::Vector3::x_axis(),
            b: na::Vector3::<T>::zeros(),
        }
    }
}

#[cfg(test)]
mod test_link {
    use crate::link::Link;

    #[test]
    fn init() {
        let l = Link::<f32>::new();
        assert_eq!(l.parent, 0);
    }
}
