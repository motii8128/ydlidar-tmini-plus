//! 回転周波数を目標値に近づける。
//!
//! 使い方: cargo run --example setfreq -- /dev/ttyUSB0 10
//!
//! 周波数変更コマンド (A5 09/0A/0B/0C) は 0.1Hz / 1Hz 単位で増減させる方式なので、
//! 現在値を読んで目標に届くまでステップを繰り返す。スキャン停止中にのみ使える。

use ydlidar_tmini_plus::{FreqStep, Lidar, Result};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap_or_else(|| "/dev/ttyUSB0".to_string());
    let target: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(10.0);

    if !(6.0..=12.0).contains(&target) {
        eprintln!("T-mini Pro の回転周波数は 6〜12 Hz です (指定: {target})");
        std::process::exit(2);
    }

    let mut lidar = Lidar::open(&path)?;
    let mut f = lidar.scan_frequency()?;
    println!("current: {f:.2} Hz");

    for _ in 0..30 {
        let diff = target - f;
        if diff.abs() < 0.05 {
            break;
        }
        let step = if diff >= 0.95 {
            FreqStep::CoarseUp
        } else if diff <= -0.95 {
            FreqStep::CoarseDown
        } else if diff > 0.0 {
            FreqStep::FineUp
        } else {
            FreqStep::FineDown
        };
        f = lidar.adjust_scan_frequency(step)?;
        println!("{step:?} -> {f:.2} Hz");
    }

    println!("final: {f:.2} Hz (target {target})");
    Ok(())
}
