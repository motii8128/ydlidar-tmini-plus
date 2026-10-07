use std::io::{Read, Write};
use std::time::{Duration, Instant};

fn main() {
    let path = std::env::args().nth(1).unwrap_or("/dev/ttyUSB0".into());
    let mut port = serialport::new(&path, 230_400)
        .timeout(Duration::from_millis(200))
        .open()
        .expect("open");

    port.write_all(&[0xA5, 0x65]).unwrap(); // 念のため停止
    std::thread::sleep(Duration::from_millis(200));
    port.clear(serialport::ClearBuffer::Input).unwrap();

    port.write_all(&[0xA5, 0x60]).unwrap();
    let t0 = Instant::now();
    let mut total = 0usize;
    let mut buf = [0u8; 256];
    while t0.elapsed() < Duration::from_secs(8) {
        match port.read(&mut buf) {
            Ok(n) => {
                total += n;
                if total <= 256 {
                    println!("{:>5}ms +{n}: {:02X?}", t0.elapsed().as_millis(), &buf[..n.min(32)]);
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                println!("{:>5}ms (no data)", t0.elapsed().as_millis());
            }
            Err(e) => { println!("error: {e}"); break; }
        }
    }
    println!("total bytes: {total}");
    port.write_all(&[0xA5, 0x65]).unwrap();
}