#![no_std]
//! This crate is for robotic math.
//!
#![allow(unused_imports)]
use core::ops::{Add, AddAssign, Mul, Neg};
use core::result::Result;
use core::time::Duration;
use num_traits::real::Real;
extern crate nalgebra as na;

pub mod dh;
pub mod matrix;
pub mod matrix3;
pub mod quaternion;
pub mod vector3;

pub mod link;
pub mod rkd;
