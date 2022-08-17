#[allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use heapless::Vec;

use crate::link::Link;

/// Robot Kinematics and Dynamics
pub struct Rkd<T> {
    links: Vec<Link<T>, 256>,
}

impl<T> Rkd<T> {
    fn new() -> Self {
        Self{
            links: Vec::new(),
        }
    }
}


#[cfg(test)]
mod test_rkd {
    use crate::link::Link;
    use crate::rkd::Rkd;

    #[test]
    fn init() {
        let l = Rkd::<f32>::new();
    }
}
