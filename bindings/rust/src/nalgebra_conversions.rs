//! Between the standard types and nalgebra's, as two traits since neither side's types are this crate's.

use nalgebra as na;
use rant_types::*;

use crate::Matrix;

/// A standard type as nalgebra's own.
pub trait ToNalgebra {
    type Output;
    fn to_nalgebra(&self) -> Self::Output;
}

/// One of nalgebra's types as the standard one.
pub trait ToRant {
    type Output;
    fn to_rant(&self) -> Self::Output;
}

impl ToNalgebra for Double2 {
    type Output = na::Vector2<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Vector2::new(self.x, self.y)
    }
}

impl ToNalgebra for Double3 {
    type Output = na::Vector3<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Vector3::new(self.x, self.y, self.z)
    }
}

impl ToNalgebra for Double4 {
    type Output = na::Vector4<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Vector4::new(self.x, self.y, self.z, self.w)
    }
}

impl ToNalgebra for Quaternion {
    type Output = na::UnitQuaternion<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::UnitQuaternion::new_normalize(na::Quaternion::new(self.w, self.x, self.y, self.z))
    }
}

impl ToNalgebra for Pose {
    type Output = na::Isometry3<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Isometry3::from_parts(na::Translation3::from(self.position.to_nalgebra()), self.orientation.to_nalgebra())
    }
}

impl ToNalgebra for Matrix3x3 {
    type Output = na::Matrix3<f32>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Matrix3::from_fn(|r, c| self.m[r][c])
    }
}

impl ToNalgebra for Matrix4x4 {
    type Output = na::Matrix4<f32>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Matrix4::from_fn(|r, c| self.m[r][c])
    }
}

impl ToNalgebra for Covariance3 {
    type Output = na::Matrix3<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Matrix3::from_fn(|r, c| self.m[r][c])
    }
}

impl ToNalgebra for Covariance6 {
    type Output = na::Matrix6<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Matrix6::from_fn(|r, c| self.m[r][c])
    }
}

impl ToNalgebra for Matrix {
    type Output = na::DMatrix<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::DMatrix::from_row_slice(self.rows as usize, self.cols as usize, &self.data)
    }
}

impl ToRant for na::Vector2<f64> {
    type Output = Double2;
    fn to_rant(&self) -> Double2 {
        Double2 { x: self.x, y: self.y }
    }
}

impl ToRant for na::Vector3<f64> {
    type Output = Double3;
    fn to_rant(&self) -> Double3 {
        Double3 { x: self.x, y: self.y, z: self.z }
    }
}

impl ToRant for na::Vector4<f64> {
    type Output = Double4;
    fn to_rant(&self) -> Double4 {
        Double4 { x: self.x, y: self.y, z: self.z, w: self.w }
    }
}

impl ToRant for na::UnitQuaternion<f64> {
    type Output = Quaternion;
    fn to_rant(&self) -> Quaternion {
        Quaternion { x: self.i, y: self.j, z: self.k, w: self.w }
    }
}

impl ToRant for na::Isometry3<f64> {
    type Output = Pose;
    fn to_rant(&self) -> Pose {
        Pose { position: self.translation.vector.to_rant(), orientation: self.rotation.to_rant() }
    }
}

impl ToRant for na::DMatrix<f64> {
    type Output = Matrix;
    fn to_rant(&self) -> Matrix {
        let mut out = Matrix::new(self.nrows() as u32, self.ncols() as u32);
        for r in 0..self.nrows() {
            for c in 0..self.ncols() {
                out.data[r * self.ncols() + c] = self[(r, c)];
            }
        }
        out
    }
}
