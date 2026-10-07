//! シリアルポート経由で T-mini Plus を操作するドライバ。

use std::io::{Read, Write};
use std::thread::sleep;
use std::time::Duration;

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

use crate::error::{Error, Result};
use crate::protocol::{
    self, cmd, type_code, DeviceInfo, Health, PacketParser, ResponseHeader, Scan, ScanAssembler,
    BAUD_RATE, MODE_CONTINUOUS, RESPONSE_HEADER_LEN,
};

impl From<serialport::Error> for Error {
    fn from(e: serialport::Error) -> Self {
        Error::from(std::io::Error::from(e))
    }
}

/// 通常の読み取りタイムアウト。
const READ_TIMEOUT: Duration = Duration::from_millis(500);
/// スキャン開始直後はモーターの立ち上がりを待つので長めに取る。
const START_TIMEOUT: Duration = Duration::from_secs(3);
/// 応答ヘッダ (A5 5A) を探すときに読み捨てる最大バイト数。
const MAX_SYNC_BYTES: usize = 8192;

/// 回転周波数の調整ステップ。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreqStep {
    /// +0.1 Hz
    FineUp,
    /// -0.1 Hz
    FineDown,
    /// +1 Hz
    CoarseUp,
    /// -1 Hz
    CoarseDown,
}

pub struct Lidar {
    port: Box<dyn SerialPort>,
    parser: PacketParser,
    assembler: ScanAssembler,
    scanning: bool,
}

impl Lidar {
    /// ポートを開く (例: `/dev/ttyUSB0`, `COM3`)。
    ///
    /// 前回のプロセスがスキャン中のまま終了していても大丈夫なように、
    /// 開いた直後に停止コマンドを送って受信バッファを空にする。
    pub fn open(path: &str) -> Result<Self> {
        let port = serialport::new(path, BAUD_RATE)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(READ_TIMEOUT)
            .open()?;

        let mut lidar = Self {
            port,
            parser: PacketParser::new(),
            assembler: ScanAssembler::new(),
            scanning: false,
        };
        lidar.stop_scan()?;
        Ok(lidar)
    }

    pub fn is_scanning(&self) -> bool {
        self.scanning
    }

    /// 通信品質の目安: 捨てたパケット候補の数。
    pub fn checksum_errors(&self) -> u64 {
        self.parser.checksum_errors
    }

    // ---- 単発コマンド (スキャン停止中のみ) --------------------------------

    pub fn device_info(&mut self) -> Result<DeviceInfo> {
        let p = self.request(cmd::DEVICE_INFO, type_code::DEVICE_INFO, DeviceInfo::PAYLOAD_LEN)?;
        DeviceInfo::parse(&p)
    }

    pub fn health(&mut self) -> Result<Health> {
        let p = self.request(cmd::HEALTH, type_code::HEALTH, Health::PAYLOAD_LEN)?;
        Health::parse(&p)
    }

    /// 現在の設定回転周波数 [Hz] (実測値ではない)。
    pub fn scan_frequency(&mut self) -> Result<f32> {
        let p = self.request(cmd::GET_FREQ, type_code::FREQUENCY, 4)?;
        protocol::parse_frequency(&p)
    }

    /// 回転周波数を 1 ステップ増減し、変更後の設定値 [Hz] を返す。
    pub fn adjust_scan_frequency(&mut self, step: FreqStep) -> Result<f32> {
        let command = match step {
            FreqStep::FineUp => cmd::FREQ_UP_0_1HZ,
            FreqStep::FineDown => cmd::FREQ_DOWN_0_1HZ,
            FreqStep::CoarseUp => cmd::FREQ_UP_1HZ,
            FreqStep::CoarseDown => cmd::FREQ_DOWN_1HZ,
        };
        let p = self.request(command, type_code::FREQUENCY, 4)?;
        protocol::parse_frequency(&p)
    }

    // ---- スキャン -----------------------------------------------------------

    /// スキャン開始 (モーター始動 + レーザー点灯)。
    pub fn start_scan(&mut self) -> Result<()> {
        if self.scanning {
            return Ok(());
        }
        self.port.clear(ClearBuffer::Input)?;
        self.parser.clear();
        self.assembler.reset();

        self.port.write_all(&cmd::START_SCAN)?;
        self.port.flush()?;

        self.port.set_timeout(START_TIMEOUT)?;
        let header = self.read_response_header();
        self.port.set_timeout(READ_TIMEOUT)?;
        let header = header?;

        if header.type_code != type_code::SCAN || header.mode != MODE_CONTINUOUS {
            return Err(Error::Protocol(format!(
                "unexpected scan response: type=0x{:02X} mode={}",
                header.type_code, header.mode
            )));
        }
        self.scanning = true;
        Ok(())
    }

    /// スキャン停止。ポートを開いた直後など、未スキャン時に呼んでも安全。
    pub fn stop_scan(&mut self) -> Result<()> {
        self.port.write_all(&cmd::STOP_SCAN)?;
        self.port.flush()?;
        self.scanning = false;
        // 停止コマンドは無応答。飛んでくるデータが止まるのを待ってから捨てる。
        sleep(Duration::from_millis(100));
        self.port.clear(ClearBuffer::Input)?;
        self.parser.clear();
        self.assembler.reset();
        Ok(())
    }

    /// 1 周分の点群が揃うまでブロックして返す。
    ///
    /// 開始パケットをまたいで 1 周ぶんを区切るので、スキャン開始直後の最初の周は
    /// 途中から始まるため捨てられ、2 周目から返る。
    /// データが途切れた場合は `Error::Timeout`。呼び直せば続きから読める。
    pub fn read_scan(&mut self) -> Result<Scan> {
        if !self.scanning {
            return Err(Error::State("not scanning; call start_scan() first"));
        }
        let mut buf = [0u8; 4096];
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let mut got_data = false;
        loop {
            while let Some(pkt) = self.parser.next_packet() {
                if let Some(scan) = self.assembler.push(&pkt) {
                    return Ok(scan);
                }
            }
            match self.port.read(&mut buf) {
                Ok(n) => {
                    got_data = true;
                    self.parser.push(&buf[..n]);
                }
                Err(e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        && !got_data
                        && std::time::Instant::now() < deadline =>
                {
                    continue; // モーター立ち上がり待ち
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    // ---- 内部 ---------------------------------------------------------------

    /// 単発コマンドを送り、検証済みのペイロードを返す。
    fn request(&mut self, command: [u8; 2], expect_type: u8, expect_len: u32) -> Result<Vec<u8>> {
        if self.scanning {
            return Err(Error::State(
                "commands other than stop are not allowed while scanning",
            ));
        }
        self.port.clear(ClearBuffer::Input)?;
        self.port.write_all(&command)?;
        self.port.flush()?;

        let header = self.read_response_header()?;
        if header.type_code != expect_type || header.length != expect_len {
            return Err(Error::Protocol(format!(
                "unexpected response to {:02X?}: type=0x{:02X} len={} (expected type=0x{:02X} len={})",
                command, header.type_code, header.length, expect_type, expect_len
            )));
        }
        let mut payload = vec![0u8; header.length as usize];
        self.port.read_exact(&mut payload)?;
        Ok(payload)
    }

    /// ストリーム中から `A5 5A` を探して応答ヘッダを読む。
    fn read_response_header(&mut self) -> Result<ResponseHeader> {
        let mut prev = 0u8;
        let mut b = [0u8; 1];
        for _ in 0..MAX_SYNC_BYTES {
            self.port.read_exact(&mut b)?;
            if prev == 0xA5 && b[0] == 0x5A {
                let mut rest = [0u8; RESPONSE_HEADER_LEN - 2];
                self.port.read_exact(&mut rest)?;
                let raw = [0xA5, 0x5A, rest[0], rest[1], rest[2], rest[3], rest[4]];
                return ResponseHeader::parse(&raw);
            }
            prev = b[0];
        }
        Err(Error::Protocol("response header (A5 5A) not found".into()))
    }
}

impl Drop for Lidar {
    fn drop(&mut self) {
        if self.scanning {
            // ベストエフォートでモーターとレーザーを止める
            let _ = self.port.write_all(&cmd::STOP_SCAN);
            let _ = self.port.flush();
        }
    }
}
