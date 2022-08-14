#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::Vec;

use crate::matrix::Matrix;
use crate::matrix3::Matrix3;
use crate::vector3::Vector3;

/// 親リンクからつながる位置（回転軸）を原点とする
pub struct Link {
    id: u8,
    children: Vec<u8, 10>,
    mass: f32,
    com: Matrix<f32, 3, 1>,     // Centor of Mass
    inertia_mat: Matrix<f32, 3, 3>, // 自リンク原点まわり
    q: f32,
    dqdt: f32,
    ddqddt: f32,
    p: Matrix<f32, 3, 1>,   // position in world coordinate
    r: Matrix<f32, 3, 3>,   // rotation matrix in world coordinate
    a: Matrix<f32, 3, 1>,   // joint axis vec relative to parent link
    b: Matrix<f32, 3, 1>,   // joint position relative to parent link   

}

impl Link {
    fn new() -> Self {
        Self{
            id: 0,
            children: Vec::new(),
            mass: 0.0,
            com: Matrix::<f32, 3, 1>::new(),
            inertia_mat: Matrix::<f32, 3, 3>::new(),
            q: 0.0,
            dqdt: 0.0,
            ddqddt: 0.0,
            p: Matrix::<f32, 3, 1>::new(),
            r: Matrix::<f32, 3, 3>::new(),
            a: Matrix::<f32, 3, 1>::new(),
            b: Matrix::<f32, 3, 1>::new(),
        }
    }
}


#[cfg(test)]
mod test_link {
    use crate::matrix::Matrix;
    use crate::matrix3::Matrix3;
    use crate::vector3::Vector3;
    use crate::link::Link;

    #[test]
    fn init() {
        let l = Link::new();
        assert_eq!(l.id, 0);
    }
}
