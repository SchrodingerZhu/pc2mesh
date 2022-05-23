#![feature(c_size_t)]
#![feature(core_ffi_c)]

use std::ffi::{c_char, c_double, c_size_t, c_ssize_t, CString};
use std::mem::ManuallyDrop;
use std::path::PathBuf;
use std::slice;
use clap::*;

#[derive(Debug, Parser)]
enum Interface {
    /// Poisson reconstruction.
    Poisson {
        /// Path to PLY input file.
        #[clap(short, long)]
        input_file: PathBuf,
        /// Path to DAE output file.
        #[clap(short, long)]
        output_file: PathBuf,
        /// FEM tree depth for Poisson Solver.
        #[clap(short, long, default_value = "8")]
        depth: usize,
        /// Width of FEM tree leaves.
        #[clap(short, long, default_value = "0")]
        width: f64,
        /// Ratio between reconstruction bounding box and the point cloud bounding box.
        #[clap(short, long, default_value = "1.1")]
        scale: f64,
        /// Enable linear interpolation to guess vertices.
        #[clap(short, long)]
        linear_fit: bool,
        /// Number of solvers.
        #[clap(short, long, default_value = "-1")]
        num_threads: isize,
        /// Filter threshold of sparse points.
        #[clap(short, long, default_value = "0.0")]
        filter: f64,
    },
    /// Ball-Pivoting reconstruction.
    BallPivoting {
        /// Path to PLY input file.
        #[clap(short, long)]
        input_file: PathBuf,
        /// Path to DAE output file.
        #[clap(short, long)]
        output_file: PathBuf,
        /// Ball radii
        #[clap(short, long, default_values = &["0.001","0.01","0.02","0.04","0.08","0.16","0.32"])]
        radii: Vec<f64>,
    },
}

#[repr(C)]
pub struct PoissonParm {
    input_file: *mut c_char,
    output_file: *mut c_char,
    depth: c_size_t,
    width: c_double,
    scale: c_double,
    linear_fit: bool,
    num_threads: c_ssize_t,
    filter: c_double
}

#[repr(C)]
pub struct BallPivotingParm {
    input_file: *mut c_char,
    output_file: *mut c_char,
    radii: *mut c_double,
    radii_size: c_size_t,
}

impl Drop for BallPivotingParm {
    fn drop(&mut self) {
        unsafe {
            let slice = slice::from_raw_parts_mut(self.radii, self.radii_size);
            drop(Box::from_raw(slice as *mut [c_double]));
            drop(CString::from_raw(self.input_file));
            drop(CString::from_raw(self.output_file));
        }
    }
}

impl Drop for PoissonParm {
    fn drop(&mut self) {
        unsafe {
            drop(CString::from_raw(self.input_file));
            drop(CString::from_raw(self.output_file));
        }
    }
}

#[repr(C)]
pub union CliParm {
    poisson: ManuallyDrop<PoissonParm>,
    ball_pivoting: ManuallyDrop<BallPivotingParm>,
}

#[repr(C)]
pub struct CliData {
    is_poisson: bool,
    parm: CliParm,
}

impl Interface {
    fn convert_string(path: &PathBuf) -> *mut c_char {
        let cstring = CString::new(path.to_string_lossy().into_owned()).unwrap();
        cstring.into_raw()
    }
    fn create_clidata(&self) -> CliData {
        let parm = match self {
            Interface::Poisson {
                input_file,
                output_file,
                depth,
                width,
                scale,
                linear_fit,
                num_threads,
                filter
            } => {
                CliParm {
                    poisson: ManuallyDrop::new(PoissonParm {
                        input_file: Self::convert_string(input_file),
                        output_file: Self::convert_string(output_file),
                        depth: *depth as c_size_t,
                        width: *width as c_double,
                        scale: *scale as c_double,
                        linear_fit: *linear_fit,
                        num_threads: *num_threads as c_ssize_t,
                        filter: *filter as c_double
                    })
                }
            }
            Interface::BallPivoting { input_file, output_file, radii } => {
                let slice = radii.iter().map(|x| *x as c_double).collect::<Vec<_>>().into_boxed_slice();
                let raw = Box::leak(slice);
                CliParm {
                    ball_pivoting: ManuallyDrop::new(BallPivotingParm {
                        input_file: Self::convert_string(input_file),
                        output_file: Self::convert_string(output_file),
                        radii: raw.as_mut_ptr(),
                        radii_size: raw.len() as c_size_t,
                    })
                }
            }
        };
        match self {
            Interface::Poisson { .. } => {
                CliData {
                    is_poisson: true,
                    parm,
                }
            }
            Interface::BallPivoting { .. } => {
                CliData {
                    is_poisson: false,
                    parm,
                }
            }
        }
    }
}

#[no_mangle]
extern "C" fn pc2mesh_create_cli() -> CliData {
    let interface: Interface = Interface::parse();
    interface.create_clidata()
}

#[no_mangle]
unsafe extern "C" fn pc2mesh_clean_cli(data: *mut CliData) {
    if (*data).is_poisson {
        ManuallyDrop::drop(&mut (*data).parm.poisson);
    } else {
        ManuallyDrop::drop(&mut (*data).parm.ball_pivoting);
    }
}