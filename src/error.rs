use std::fmt;
use std::io;

#[derive(Debug)]
pub enum Error {
    /// シリアルポート / OS レベルの I/O エラー
    Io(io::Error),
    /// 読み取りタイムアウト (データが来ない)
    Timeout,
    /// 想定外の応答・フォーマット不正
    Protocol(String),
    /// 現在の状態では実行できない操作 (スキャン中にコマンドを送る等)
    State(&'static str),
}

pub type Result<T> = std::result::Result<T, Error>;

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "I/O error: {e}"),
            Error::Timeout => write!(f, "timed out waiting for data from the lidar"),
            Error::Protocol(msg) => write!(f, "protocol error: {msg}"),
            Error::State(msg) => write!(f, "invalid state: {msg}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        if e.kind() == io::ErrorKind::TimedOut {
            Error::Timeout
        } else {
            Error::Io(e)
        }
    }
}
