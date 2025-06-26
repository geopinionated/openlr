use thiserror::Error;

#[derive(Error, Debug, PartialEq, Eq)]
pub enum OpenLrError {
    #[error("Invalid Base 64")]
    InvalidBase64,
    #[error("I/O error")]
    IO,
    #[error("OpenLR version not supported")]
    VersionNotSupported,
    #[error("Invalid binary representation")]
    BinaryParseError,
}

impl From<base64::DecodeError> for OpenLrError {
    fn from(_: base64::DecodeError) -> Self {
        Self::InvalidBase64
    }
}

impl From<std::io::Error> for OpenLrError {
    fn from(_: std::io::Error) -> Self {
        Self::IO
    }
}
