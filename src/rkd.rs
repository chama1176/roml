#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::Vec;

use crate::matrix::Matrix;
use crate::matrix3::Matrix3;
use crate::vector3::Vector3;
use crate::link::Link;

/// Robot Kinematics and Dynamics
pub struct Rkd {
    links: Vec<Link, 256>,
}

impl Rkd {
    fn new() -> Self {
        Self{
            links: Vec::new(),
        }
    }
}


#[cfg(test)]
mod test_rkd {
    use crate::matrix::Matrix;
    use crate::matrix3::Matrix3;
    use crate::vector3::Vector3;
    use crate::link::Link;
    use crate::rkd::Rkd;

    #[test]
    fn init() {
        let l = Rkd::new();
    }
}
