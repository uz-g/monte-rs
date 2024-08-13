use alloc::{string::ToString, sync::Arc};

use serde_json::{json, Value};
use vexide::{core::sync::Mutex, devices::smart::SerialPort, prelude::Write};

pub struct Telemetry {
    serial: Arc<Mutex<SerialPort>>,
}

impl Telemetry {
    pub fn new(serial: SerialPort) -> Self {
        Self {
            serial: Arc::new(Mutex::new(serial)),
        }
    }

    pub async fn send(&self, bytes: &[u8]) {
        self.serial.lock().await.write_all(bytes).unwrap(); // TODO: fix later
        self.serial.lock().await.flush().unwrap();
    }

    pub async fn send_json(&self, data: impl serde::ser::Serialize) {
        self.send(
            serde_json::to_string(&data)
                .unwrap_or("".to_string())
                .as_bytes(),
        )
        .await;
    }

    pub async fn send_str(&self, data: &str) {
        self.send(data.as_bytes()).await;
    }
}

impl Clone for Telemetry {
    fn clone(&self) -> Self {
        Self {
            serial: self.serial.clone(),
        }
    }
}
