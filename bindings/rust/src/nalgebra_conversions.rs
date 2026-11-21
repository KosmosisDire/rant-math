//! Between the standard types and nalgebra's where it takes more than a copy: a rotation nalgebra holds as
//! a unit quaternion is made whole here. rant-types' feature nalgebra gives the copies as `From`.

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

impl ToNalgebra for Quaternion {
    type Output = na::UnitQuaternion<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::UnitQuaternion::new_normalize(na::Quaternion::from(*self))
    }
}

impl ToNalgebra for Pose {
    type Output = na::Isometry3<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::Isometry3::from_parts(na::Translation3::from(na::Vector3::from(self.position)), self.orientation.to_nalgebra())
    }
}

impl ToNalgebra for Matrix {
    type Output = na::DMatrix<f64>;
    fn to_nalgebra(&self) -> Self::Output {
        na::DMatrix::from_row_slice(self.rows as usize, self.cols as usize, &self.data)
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
