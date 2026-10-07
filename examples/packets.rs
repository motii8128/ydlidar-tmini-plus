//! 診断用: 実機のパケットを本クレートのパーサで解読し、統計を表示する。
//!
//! 使い方: cargo run --example packets -- /dev/ttyUSB0
//!
//! チェックサムが通るか、角度の刻み・1 周の周期が妥当か、flag ビットの分布はどうか、を見る。

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use ydlidar_tmini_plus::protocol::{cmd, PacketParser, BAUD_RATE};

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "/dev/ttyUSB0".to_string());
    let mut port = serialport::new(&path, BAUD_RATE)
        .timeout(Duration::from_millis(200))
        .open()
        .expect("open port");

    port.write_all(&cmd::STOP_SCAN).unwrap();
    std::thread::sleep(Duration::from_millis(200));
    port.clear(serialport::ClearBuffer::Input).unwrap();
    port.write_all(&cmd::START_SCAN).unwrap();

    let mut parser = PacketParser::new();
    let mut buf = [0u8; 4096];
    let t0 = Instant::now();

    let mut packets = 0u32;
    let mut starts = 0u32;
    let mut samples = 0u32;
    let mut zero_dist = 0u32;
    let mut flag_hist = [0u32; 4];
    let mut late_spans: Vec<f32> = Vec::new();
    let mut printed_late = 0;

    while t0.elapsed() < Duration::from_secs(6) {
        let n = match port.read(&mut buf) {
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => {
                eprintln!("read error: {e}");
                break;
            }
        };
        parser.push(&buf[..n]);

        while let Some(pkt) = parser.next_packet() {
            packets += 1;
            if pkt.is_start() {
                starts += 1;
            }
            let mut span = pkt.last_angle_deg - pkt.first_angle_deg;
            if span < 0.0 {
                span += 360.0;
            }
            for s in &pkt.samples {
                samples += 1;
                flag_hist[s.flag as usize] += 1;
                if s.distance_mm == 0 {
                    zero_dist += 1;
                }
            }

            let late = t0.elapsed() > Duration::from_secs(3);
            if late && !pkt.is_start() {
                late_spans.push(span);
            }
            if packets <= 3 || (late && printed_late < 6) {
                if late {
                    printed_late += 1;
                }
                let head: Vec<_> = pkt
                    .samples
                    .iter()
                    .take(3)
                    .map(|s| (s.intensity, s.distance_mm, s.flag))
                    .collect();
                println!(
                    "{:>5}ms ct=0x{:02X} start={} lsn={} fsa={:.2} lsa={:.2} span={:.2} first(int,dist,flag)={:?}",
                    t0.elapsed().as_millis(),
                    pkt.ct,
                    pkt.is_start(),
                    pkt.samples.len(),
                    pkt.first_angle_deg,
                    pkt.last_angle_deg,
                    span,
                    head
                );
            }
        }
    }

    port.write_all(&cmd::STOP_SCAN).unwrap();

    let secs = t0.elapsed().as_secs_f32();
    println!("--- summary ({secs:.1}s) ---");
    println!("packets={packets} start_packets={starts} (~{:.1} rev/s)", starts as f32 / secs);
    println!("checksum_errors={}", parser.checksum_errors);
    println!("samples={samples} zero_distance={zero_dist}");
    println!("flag histogram [0,1,2,3] = {flag_hist:?}");
    if !late_spans.is_empty() {
        let mean = late_spans.iter().sum::<f32>() / late_spans.len() as f32;
        println!("mean angular span per packet (after 3s) = {mean:.2} deg");
    }
}