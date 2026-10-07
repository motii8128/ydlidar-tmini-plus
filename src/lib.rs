//! YDLIDAR T-mini Plus ドライバ。
//!
//! UART (230400 bps, 8N1) で接続した T-mini Plus から 2 次元点群を取得する。
//!
//! ```no_run
//! use ydlidar_tmini_plus::Lidar;
//!
//! # fn main() -> ydlidar_tmini_plus::Result<()> {
//! let mut lidar = Lidar::open("/dev/ttyUSB0")?;
//! lidar.start_scan()?;
//! let scan = lidar.read_scan()?; // 1 周分
//! for p in scan.valid_points() {
//!     let (x, y) = p.to_xy_m();
//!     println!("{x:.3},{y:.3}");
//! }
//! lidar.stop_scan()?;
//! # Ok(()) }
//! ```
//!
//! - [`protocol`]: バイト列のパース (I/O 非依存・単体テスト可能)
//! - [`driver`]: シリアルポート制御 (`Lidar`)

pub mod driver;
pub mod error;
pub mod protocol;

pub use driver::{FreqStep, Lidar};
pub use error::{Error, Result};
pub use protocol::{DeviceInfo, Health, Point, Sample, Scan, ScanPacket};
