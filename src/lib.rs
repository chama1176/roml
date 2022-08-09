#![no_std]
//! This crate is for robotic math.
//!
#![allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul};
use core::result::Result;
use core::time::Duration;

pub mod matrix;
// use heapless::Vec;

// pub trait Interface {
//     fn write_byte(&mut self, data: u8);
//     fn read_byte(&mut self) -> Option<u8>;
// }
// pub trait Clock {
//     fn get_current_time(&self) -> Duration;
// }

#[allow(dead_code)]
pub struct Quaternion {
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
}

impl Quaternion {
    pub fn new() -> Self {
        Self {
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
        }
    }
    pub fn x(self) -> f32 {
        self.q0
    }
}

#[cfg(test)]
mod tests {
    use crate::Quaternion;

    #[test]
    fn init_quaternion() {
        let q = Quaternion::new();
        assert_eq!(q.q0, 1.0);
        assert_eq!(q.q1, 0.0);
        assert_eq!(q.q2, 0.0);
        assert_eq!(q.q3, 0.0);
    }

    #[test]
    fn quaternion() {
        let q = Quaternion::new();
        assert_eq!(q.x(), 1.0);
    }

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
