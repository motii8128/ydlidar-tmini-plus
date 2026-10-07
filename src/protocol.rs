//! YDLIDAR T-mini Plus のバイト列プロトコル (I/O 非依存)。
//!
//! 仕様は "YDLIDAR T-mini Plus Development Manual" (DOC#01.13.005) に基づく。
//! シリアルポートには触らないので、実機なしで単体テストできる。

use std::fmt;

use crate::error::{Error, Result};

/// UART ボーレート (8N1, フロー制御なし)。
pub const BAUD_RATE: u32 = 230_400;

/// システムコマンド (2 バイト: 0xA5 + コマンド)。
pub mod cmd {
    pub const START_SCAN: [u8; 2] = [0xA5, 0x60];
    pub const STOP_SCAN: [u8; 2] = [0xA5, 0x65];
    pub const DEVICE_INFO: [u8; 2] = [0xA5, 0x90];
    pub const HEALTH: [u8; 2] = [0xA5, 0x92];
    pub const FREQ_UP_0_1HZ: [u8; 2] = [0xA5, 0x09];
    pub const FREQ_DOWN_0_1HZ: [u8; 2] = [0xA5, 0x0A];
    pub const FREQ_UP_1HZ: [u8; 2] = [0xA5, 0x0B];
    pub const FREQ_DOWN_1HZ: [u8; 2] = [0xA5, 0x0C];
    pub const GET_FREQ: [u8; 2] = [0xA5, 0x0D];
    pub const RESTART: [u8; 2] = [0xA5, 0x40];
}

/// 応答ヘッダのタイプコード。
pub mod type_code {
    pub const SCAN: u8 = 0x81;
    pub const DEVICE_INFO: u8 = 0x04;
    pub const HEALTH: u8 = 0x06;
    pub const FREQUENCY: u8 = 0x04;
}

/// 応答ヘッダ長 (開始サイン 2 + 長さ/モード 4 + タイプ 1)。
pub const RESPONSE_HEADER_LEN: usize = 7;

/// 応答モード: 単発 / 連続。
pub const MODE_SINGLE: u8 = 0;
pub const MODE_CONTINUOUS: u8 = 1;

/// 応答ヘッダ `A5 5A <len:30bit|mode:2bit LE> <type>`。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResponseHeader {
    pub length: u32,
    pub mode: u8,
    pub type_code: u8,
}

impl ResponseHeader {
    pub fn parse(b: &[u8; RESPONSE_HEADER_LEN]) -> Result<Self> {
        if b[0] != 0xA5 || b[1] != 0x5A {
            return Err(Error::Protocol(format!(
                "invalid response start sign: {:02X} {:02X}",
                b[0], b[1]
            )));
        }
        // 下位 30bit が長さ、上位 2bit がモード (6 バイト目の上位 2bit)。
        let raw = u32::from_le_bytes([b[2], b[3], b[4], b[5]]);
        Ok(Self {
            length: raw & 0x3FFF_FFFF,
            mode: (raw >> 30) as u8,
            type_code: b[6],
        })
    }
}

// ---------------------------------------------------------------------------
// 点・サンプル
// ---------------------------------------------------------------------------

/// パケット内の 1 サンプル (3 バイト)。
///
/// - byte0: intensity[7:0]
/// - byte1: distance[5:0] << 2 | flag[1:0]
/// - byte2: distance[13:6]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    pub intensity: u8,
    /// 距離 [mm]。0 は無効 (遮蔽・未検出)。
    pub distance_mm: u16,
    /// 2 = 鏡面反射の干渉, 3 = 環境光の干渉 (マニュアルより)。
    pub flag: u8,
}

impl Sample {
    pub fn from_bytes(b: [u8; 3]) -> Self {
        Self {
            intensity: b[0],
            distance_mm: ((b[2] as u16) << 6) | ((b[1] >> 2) as u16),
            flag: b[1] & 0x03,
        }
    }
}

/// 角度付きの 1 点。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// センサ出力の角度 [deg], 0..360 (センサ座標系で時計回りに増加)。
    pub angle_deg: f32,
    pub distance_mm: u16,
    pub intensity: u8,
    pub flag: u8,
}

impl Point {
    /// 距離が得られている点か。
    pub fn is_valid(&self) -> bool {
        self.distance_mm != 0
    }

    /// 干渉 (鏡面反射 / 環境光) としてマークされた点か。
    pub fn is_interference(&self) -> bool {
        matches!(self.flag, 2 | 3)
    }

    /// デカルト座標 [m] へ変換する。
    ///
    /// センサの角度は時計回りに増加するので、反時計回りを正とする
    /// 右手系 (ROS REP-103 風) に合わせて符号を反転している。
    /// 0° の向きがセンサのどちらを向くかは機体への取り付けに依存するので、
    /// 実機で確認して必要なら回転オフセットを掛けること。
    pub fn to_xy_m(&self) -> (f32, f32) {
        let r = self.distance_mm as f32 / 1000.0;
        let th = (-self.angle_deg).to_radians();
        (r * th.cos(), r * th.sin())
    }
}

/// 1 周分の点群。
#[derive(Debug, Clone, Default)]
pub struct Scan {
    pub points: Vec<Point>,
    /// 開始パケットの CT から得た回転周波数 [Hz] (取得できた場合)。
    pub scan_freq_hz: Option<f32>,
}

impl Scan {
    /// 距離が得られている点 (distance > 0) だけを返す。
    pub fn valid_points(&self) -> impl Iterator<Item = &Point> {
        self.points.iter().filter(|p| p.is_valid())
    }

    /// 距離が有効で、さらに干渉フラグ (2, 3) が立っていない点だけを返す。
    ///
    /// フラグの意味は T-mini Plus のマニュアルに基づく。他機種では
    /// 同じビットが別の意味を持つことがあるので、使う前に実機で確認すること。
    pub fn clean_points(&self) -> impl Iterator<Item = &Point> {
        self.points
            .iter()
            .filter(|p| p.is_valid() && !p.is_interference())
    }
}

// ---------------------------------------------------------------------------
// スキャンパケット
// ---------------------------------------------------------------------------

const PACKET_HEADER: [u8; 2] = [0xAA, 0x55];
const HEADER_LEN: usize = 10; // PH(2) CT(1) LSN(1) FSA(2) LSA(2) CS(2)
const SAMPLE_LEN: usize = 3;

/// 角度フィールド (下位 1bit はチェックビット = 1 固定) を度に変換する。
pub fn raw_angle_to_deg(raw: u16) -> f32 {
    (raw >> 1) as f32 / 64.0
}

/// デコード済みスキャンパケット。
#[derive(Debug, Clone, PartialEq)]
pub struct ScanPacket {
    pub ct: u8,
    pub first_angle_deg: f32,
    pub last_angle_deg: f32,
    pub samples: Vec<Sample>,
}

impl ScanPacket {
    /// CT bit0 = 1 は 1 周の先頭 (開始パケット, LSN = 1)。
    pub fn is_start(&self) -> bool {
        self.ct & 1 == 1
    }

    /// 開始パケットの CT[7:1] は回転周波数 ×10 を運ぶ (マニュアル Chart 5, index 0)。
    pub fn scan_freq_hz(&self) -> Option<f32> {
        if self.is_start() {
            Some((self.ct >> 1) as f32 / 10.0)
        } else {
            None
        }
    }

    /// FSA〜LSA を線形補間して各サンプルへ角度を割り当てる。
    pub fn points(&self) -> impl Iterator<Item = Point> + '_ {
        let n = self.samples.len();
        let first = self.first_angle_deg;
        let mut diff = self.last_angle_deg - first;
        if diff < 0.0 {
            diff += 360.0; // 0° をまたぐ場合 (時計回りの差)
        }
        let step = if n > 1 { diff / (n - 1) as f32 } else { 0.0 };
        self.samples.iter().enumerate().map(move |(i, s)| {
            let mut a = first + step * i as f32;
            if a >= 360.0 {
                a -= 360.0;
            }
            Point {
                angle_deg: a,
                distance_mm: s.distance_mm,
                intensity: s.intensity,
                flag: s.flag,
            }
        })
    }
}

/// `raw` はヘッダ 10 バイト + サンプル LSN*3 バイトちょうど。
/// チェックビットまたはチェックサムが不正なら None。
fn decode_packet(raw: &[u8]) -> Option<ScanPacket> {
    let ct = raw[2];
    let lsn = raw[3] as usize;
    let fsa = u16::from_le_bytes([raw[4], raw[5]]);
    let lsa = u16::from_le_bytes([raw[6], raw[7]]);
    let cs = u16::from_le_bytes([raw[8], raw[9]]);

    if fsa & 1 == 0 || lsa & 1 == 0 {
        return None; // 角度のチェックビットは 1 固定
    }

    // CS = PH ^ (LSN<<8 | CT) ^ FSA ^ LSA ^ Σ( 0x00<<8|Si[1] ,  Si[3]<<8 | Si[2] )
    let mut calc: u16 = 0x55AA ^ (((lsn as u16) << 8) | ct as u16) ^ fsa ^ lsa;
    let mut samples = Vec::with_capacity(lsn);
    for s in raw[HEADER_LEN..].chunks_exact(SAMPLE_LEN) {
        calc ^= s[0] as u16;
        calc ^= ((s[2] as u16) << 8) | s[1] as u16;
        samples.push(Sample::from_bytes([s[0], s[1], s[2]]));
    }
    if calc != cs {
        return None;
    }

    Some(ScanPacket {
        ct,
        first_angle_deg: raw_angle_to_deg(fsa),
        last_angle_deg: raw_angle_to_deg(lsa),
        samples,
    })
}

/// ストリーミングパーサ。任意の区切りでバイトを `push` し、`next_packet` で取り出す。
///
/// ヘッダ (AA 55) を探し、長さとチェックサムを検証する。壊れていれば 1 バイト進めて
/// 再同期するので、バイト欠落や途中接続でも復帰できる。
#[derive(Debug, Default)]
pub struct PacketParser {
    buf: Vec<u8>,
    /// チェックサム/チェックビット不一致で捨てた候補の数 (通信品質の目安)。
    pub checksum_errors: u64,
}

impl PacketParser {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }

    /// 完全なパケットが 1 つ取れれば返す。データ不足なら None (続きを push すること)。
    pub fn next_packet(&mut self) -> Option<ScanPacket> {
        loop {
            match self.buf.windows(2).position(|w| w == PACKET_HEADER) {
                Some(p) => {
                    self.buf.drain(..p);
                }
                None => {
                    // 末尾が 0xAA なら、次のバイト 0x55 と組になるかもしれないので残す
                    let keep = usize::from(self.buf.last() == Some(&0xAA));
                    let n = self.buf.len() - keep;
                    self.buf.drain(..n);
                    return None;
                }
            }

            if self.buf.len() < HEADER_LEN {
                return None;
            }
            let lsn = self.buf[3] as usize;
            if lsn == 0 {
                self.checksum_errors += 1;
                self.buf.drain(..1);
                continue;
            }
            let total = HEADER_LEN + SAMPLE_LEN * lsn;
            if self.buf.len() < total {
                return None;
            }

            match decode_packet(&self.buf[..total]) {
                Some(pkt) => {
                    self.buf.drain(..total);
                    return Some(pkt);
                }
                None => {
                    self.checksum_errors += 1;
                    self.buf.drain(..1); // 偽ヘッダかもしれないので 1 バイトだけ進める
                }
            }
        }
    }

    pub fn clear(&mut self) {
        self.buf.clear();
    }
}

/// パケット列から 1 周分の `Scan` を組み立てる。
///
/// 開始パケット (CT bit0 = 1) が来たら、それまでに溜めた点を 1 周分として返す。
/// 最初の開始パケットより前のデータ (途中の周) は捨てる。
#[derive(Debug, Default)]
pub struct ScanAssembler {
    synced: bool,
    points: Vec<Point>,
    freq: Option<f32>,
}

impl ScanAssembler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, pkt: &ScanPacket) -> Option<Scan> {
        let mut finished = None;
        if pkt.is_start() {
            if self.synced {
                finished = Some(Scan {
                    points: std::mem::take(&mut self.points),
                    scan_freq_hz: self.freq,
                });
            }
            self.synced = true;
            self.freq = pkt.scan_freq_hz();
        } else if !self.synced {
            return None;
        }
        self.points.extend(pkt.points());
        finished
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

// ---------------------------------------------------------------------------
// 単発応答 (A5 90 / A5 92 / 周波数)
// ---------------------------------------------------------------------------

/// `A5 90` の応答内容 (20 バイト)。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    /// モデルコード (T-mini Plus は 151)。
    pub model: u8,
    pub firmware_major: u8,
    pub firmware_minor: u8,
    pub hardware: u8,
    pub serial: [u8; 16],
}

impl DeviceInfo {
    pub const PAYLOAD_LEN: u32 = 20;

    pub fn parse(p: &[u8]) -> Result<Self> {
        if p.len() < Self::PAYLOAD_LEN as usize {
            return Err(Error::Protocol(format!(
                "device info payload too short: {} bytes",
                p.len()
            )));
        }
        let mut serial = [0u8; 16];
        serial.copy_from_slice(&p[4..20]);
        Ok(Self {
            model: p[0],
            // ファームウェア 2 バイト: 下位バイトが major, 上位バイトが minor
            firmware_major: p[1],
            firmware_minor: p[2],
            hardware: p[3],
            serial,
        })
    }
}

impl fmt::Display for DeviceInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "model={} firmware={}.{} hardware={} serial=",
            self.model, self.firmware_major, self.firmware_minor, self.hardware
        )?;
        for b in &self.serial {
            write!(f, "{b:02X}")?;
        }
        Ok(())
    }
}

/// `A5 92` の応答内容 (3 バイト)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Health {
    /// bit0 Sensor, bit1 Encode, bit2 WiPwr, bit3 PD, bit4 LD, bit5 Data (1 = 異常)。
    pub status: u8,
    pub error_code: u16,
}

impl Health {
    pub const PAYLOAD_LEN: u32 = 3;

    pub fn parse(p: &[u8]) -> Result<Self> {
        if p.len() < Self::PAYLOAD_LEN as usize {
            return Err(Error::Protocol(format!(
                "health payload too short: {} bytes",
                p.len()
            )));
        }
        Ok(Self {
            status: p[0],
            error_code: u16::from_le_bytes([p[1], p[2]]),
        })
    }

    /// 下位 6bit (bit6,7 は予約) がすべて 0 なら正常。
    pub fn is_ok(&self) -> bool {
        self.status & 0x3F == 0
    }
}

/// 周波数コマンド (`A5 09/0A/0B/0C/0D`) の応答内容 (4 バイト, 1/100 Hz) を Hz に変換。
pub fn parse_frequency(p: &[u8]) -> Result<f32> {
    let b: [u8; 4] = p
        .get(..4)
        .and_then(|s| <[u8; 4]>::try_from(s).ok())
        .ok_or_else(|| Error::Protocol("frequency payload too short".into()))?;
    Ok(u32::from_le_bytes(b) as f32 / 100.0)
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// マニュアル 3.1.6 の例 (LSN = 19, CS = 0xE85A)。
    const MANUAL_PACKET: &str = "AA 55 20 13 95 A9 63 00 5A E8 01 08 B9 01 AC B8 01 F8 B6 01 88 B5 \
        01 58 AE 01 C0 AB 01 04 AA 01 78 A9 01 34 A9 01 C4 AD 01 34 AF 01 78 AE 01 2C AD 01 02 00 \
        00 62 AD 01 60 AD 01 F0 AB 01 88 AD 01 FE AD";

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace()
            .map(|h| u8::from_str_radix(h, 16).unwrap())
            .collect()
    }

    /// テスト用にチェックサム付きパケットを組み立てる。
    fn build_packet(ct: u8, fsa_deg: f32, lsa_deg: f32, samples: &[[u8; 3]]) -> Vec<u8> {
        let enc = |deg: f32| (((deg * 64.0) as u16) << 1) | 1;
        let (fsa, lsa) = (enc(fsa_deg), enc(lsa_deg));
        let lsn = samples.len() as u8;
        let mut cs: u16 = 0x55AA ^ (((lsn as u16) << 8) | ct as u16) ^ fsa ^ lsa;
        for s in samples {
            cs ^= s[0] as u16;
            cs ^= ((s[2] as u16) << 8) | s[1] as u16;
        }
        let mut v = vec![0xAA, 0x55, ct, lsn];
        v.extend_from_slice(&fsa.to_le_bytes());
        v.extend_from_slice(&lsa.to_le_bytes());
        v.extend_from_slice(&cs.to_le_bytes());
        for s in samples {
            v.extend_from_slice(s);
        }
        v
    }

    #[test]
    fn response_header_scan() {
        let h = ResponseHeader::parse(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]).unwrap();
        assert_eq!(h.length, 5);
        assert_eq!(h.mode, MODE_CONTINUOUS);
        assert_eq!(h.type_code, type_code::SCAN);
    }

    #[test]
    fn response_header_device_info() {
        let h = ResponseHeader::parse(&[0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04]).unwrap();
        assert_eq!((h.length, h.mode, h.type_code), (20, MODE_SINGLE, 0x04));
    }

    #[test]
    fn response_header_rejects_bad_sign() {
        assert!(ResponseHeader::parse(&[0x00, 0x5A, 0, 0, 0, 0, 0]).is_err());
    }

    #[test]
    fn sample_matches_manual_example() {
        // マニュアル 3.1.2 / 3.1.4: 64 E5 6F → intensity 100, distance 7161 mm
        let s = Sample::from_bytes([0x64, 0xE5, 0x6F]);
        assert_eq!(s.intensity, 100);
        assert_eq!(s.distance_mm, 7161);
        assert_eq!(s.flag, 1);
    }

    #[test]
    fn manual_packet_decodes() {
        let mut p = PacketParser::new();
        p.push(&hex(MANUAL_PACKET));
        let pkt = p.next_packet().expect("packet");
        assert!(p.next_packet().is_none());

        assert_eq!(pkt.ct, 0x20);
        assert!(!pkt.is_start());
        assert_eq!(pkt.samples.len(), 19);
        assert!((pkt.first_angle_deg - 339.15625).abs() < 1e-3);
        assert!((pkt.last_angle_deg - 0.765625).abs() < 1e-3);

        assert_eq!(
            pkt.samples[0],
            Sample { intensity: 1, distance_mm: 11842, flag: 0 }
        );
        // 14 番目 (01 02 00): 距離 0 + flag 2 (干渉)
        assert_eq!(pkt.samples[13].distance_mm, 0);
        assert_eq!(pkt.samples[13].flag, 2);

        let pts: Vec<_> = pkt.points().collect();
        assert_eq!(pts.len(), 19);
        assert!((pts[0].angle_deg - 339.15625).abs() < 1e-3);
        // 0° をまたいで最後は LSA に一致する
        assert!((pts[18].angle_deg - 0.765625).abs() < 1e-3);
        assert!(pts.iter().all(|p| (0.0..360.0).contains(&p.angle_deg)));
    }

    #[test]
    fn corrupted_packet_is_rejected() {
        let mut bytes = hex(MANUAL_PACKET);
        bytes[20] ^= 0xFF;
        let mut p = PacketParser::new();
        p.push(&bytes);
        assert!(p.next_packet().is_none());
        assert!(p.checksum_errors >= 1);
    }

    #[test]
    fn resyncs_after_garbage_and_split_reads() {
        let pkt = hex(MANUAL_PACKET);
        let mut p = PacketParser::new();

        let mut first = vec![0x00, 0xAA, 0x13]; // ゴミ
        first.extend_from_slice(&pkt[..7]);
        p.push(&first);
        assert!(p.next_packet().is_none());

        p.push(&pkt[7..]);
        let got = p.next_packet().expect("packet after rest arrives");
        assert_eq!(got.samples.len(), 19);
    }

    #[test]
    fn stray_0xaa_before_header_is_skipped() {
        // 開始パケットの前に LastCRC 相当の 1 バイトが付くケース (値が 0xAA でも同期できる)
        let mut bytes = vec![0xAA];
        bytes.extend(hex(MANUAL_PACKET));
        let mut p = PacketParser::new();
        p.push(&bytes);
        assert!(p.next_packet().is_some());
    }

    #[test]
    fn back_to_back_packets() {
        let mut bytes = hex(MANUAL_PACKET);
        bytes.extend(hex(MANUAL_PACKET));
        let mut p = PacketParser::new();
        p.push(&bytes);
        assert!(p.next_packet().is_some());
        assert!(p.next_packet().is_some());
        assert!(p.next_packet().is_none());
    }

    #[test]
    fn start_packet_carries_frequency() {
        // CT = (10.0Hz * 10) << 1 | 1 = 0xC9
        let bytes = build_packet(0xC9, 0.0, 0.0, &[[0x10, 0x00, 0x01]]);
        let mut p = PacketParser::new();
        p.push(&bytes);
        let pkt = p.next_packet().unwrap();
        assert!(pkt.is_start());
        assert_eq!(pkt.scan_freq_hz(), Some(10.0));
        let pts: Vec<_> = pkt.points().collect();
        assert_eq!(pts.len(), 1);
        assert_eq!(pts[0].distance_mm, 64);
    }

    #[test]
    fn assembler_returns_one_revolution_per_start_packet() {
        let start = build_packet(0xC9, 0.0, 0.0, &[[10, 0x00, 0x01]]);
        let body = build_packet(0x00, 1.0, 3.0, &[[10, 0x00, 0x01], [10, 0x00, 0x01], [10, 0x00, 0x01]]);

        let mut parser = PacketParser::new();
        let mut asm = ScanAssembler::new();
        let mut scans = Vec::new();

        // 同期前の途中パケット → 捨てられる。start, body, body, start で 1 周完成。
        for chunk in [&body, &start, &body, &body, &start] {
            parser.push(chunk);
            while let Some(pkt) = parser.next_packet() {
                if let Some(s) = asm.push(&pkt) {
                    scans.push(s);
                }
            }
        }

        assert_eq!(scans.len(), 1);
        assert_eq!(scans[0].points.len(), 1 + 3 + 3);
        assert_eq!(scans[0].scan_freq_hz, Some(10.0));
    }

    #[test]
    fn xy_conversion_is_counter_clockwise_positive() {
        let p = Point { angle_deg: 90.0, distance_mm: 1000, intensity: 0, flag: 0 };
        let (x, y) = p.to_xy_m();
        assert!(x.abs() < 1e-5);
        assert!((y + 1.0).abs() < 1e-5); // 時計回り 90° → y = -1
    }

    #[test]
    fn device_info_and_health_parse() {
        let mut payload = vec![151, 0x02, 0x01, 0x03];
        payload.extend(1..=16u8);
        let info = DeviceInfo::parse(&payload).unwrap();
        assert_eq!((info.model, info.firmware_major, info.firmware_minor, info.hardware), (151, 2, 1, 3));

        let h = Health::parse(&[0x00, 0x00, 0x00]).unwrap();
        assert!(h.is_ok());
        let h = Health::parse(&[0x02, 0x34, 0x12]).unwrap();
        assert!(!h.is_ok());
        assert_eq!(h.error_code, 0x1234);

        assert_eq!(parse_frequency(&[0xE8, 0x03, 0x00, 0x00]).unwrap(), 10.0);
    }
}