//! 使い方: cargo run --release --example scan -- /dev/ttyUSB0 [周回数]
//!
//! デバイス情報とヘルスを表示し、指定回数スキャンして、最後の 1 周を scan.csv (x,y,intensity) に書き出す。

use std::fmt::Write as _;

use ydlidar_tmini_plus::{Lidar, Result};

fn main() -> Result<()> {
    let path = "/dev/ttyUSB0";
    let mut lidar = Lidar::open(path)?;

    println!("{}", lidar.device_info()?);
    let health = lidar.health()?;
    println!("health: {health:?} (ok = {})", health.is_ok());
    println!("scan frequency setting: {:.2} Hz", lidar.scan_frequency()?);
    lidar.start_scan()?;

    let mut last = None;
    for i in 0..10 {
        let scan = lidar.read_scan()?;
        let valid = scan.valid_points().count();
        let nearest = scan
            .valid_points()
            .min_by_key(|p| p.distance_mm)
            .map(|p| (p.angle_deg, p.distance_mm));
        println!(
            "#{i:03} points={} valid={} freq={:?}Hz nearest={:?} checksum_errors={}",
            scan.points.len(),
            valid,
            scan.scan_freq_hz,
            nearest,
            lidar.checksum_errors(),
        );
        last = Some(scan);
    }

    lidar.stop_scan()?;

    if let Some(scan) = last {
        let mut csv = String::from("x_m,y_m,intensity\n");
        for p in scan.valid_points() {
            let (x, y) = p.to_xy_m();
            writeln!(csv, "{x:.4},{y:.4},{}", p.intensity).unwrap();
        }
        std::fs::write("scan.csv", csv).map_err(ydlidar_tmini_plus::Error::from)?;
        println!("wrote scan.csv");
    }
    Ok(())
}
