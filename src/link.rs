#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::Vec;

/// 親リンクからつながる位置（回転軸）を原点とする
pub struct Link<T> {
    id: u8,
    parent: u8,
    children: Vec<u8, 10>,
    mass: f32,
    com: na::Vector3<T>,     // Centor of Mass
    inertia_mat: na::Matrix3<T>, // 自リンク原点まわり
    q: f32,
    dqdt: f32,
    ddqddt: f32,
    q_min: f32,
    q_max: f32,
    p: na::Vector3<T>,   // position in world coordinate
    r: na::Matrix3<T>,   // rotation matrix in world coordinate
    a: na::Unit<na::Vector3<T>>,   // joint axis vec relative to parent link
    b: na::Vector3<T>,   // joint position relative to parent link   

}

impl<T: na::RealField> Link<T> {
    fn new() -> Self {
        Self{
            id: 0,
            parent: 0,
            children: Vec::new(),
            mass: 0.0,
            com: na::Vector3::<T>::zeros(),
            inertia_mat: na::Matrix3::zeros(),
            q: 0.0,
            dqdt: 0.0,
            ddqddt: 0.0,
            q_min: 0.0,
            q_max: 0.0,
            p: na::Vector3::<T>::zeros(),
            r: na::Matrix3::zeros(),
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
        assert_eq!(l.id, 0);
    }
}
