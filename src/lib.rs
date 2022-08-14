#![no_std]
//! This crate is for robotic math.
//!
#![allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use core::result::Result;
use core::time::Duration;
use num_traits::real::Real;

pub mod matrix;
pub use matrix::Matrix;
pub mod affine3;
pub mod matrix3;
pub mod vector3;
pub mod quaternion;