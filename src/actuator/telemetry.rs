use alloc::{string::ToString, sync::Arc};
use core::{fmt::Debug, time::Duration};

use serde_json::{json, Value};
use vexide::{
    core::sync::Mutex,
    devices::smart::SerialPort,
    prelude::{println, sleep, Write},
};

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
        let mut serial_lock = self.serial.lock().await;
        for i in bytes.chunks(SerialPort::INTERNAL_BUFFER_SIZE) {
            while serial_lock.available_write_bytes().unwrap() < SerialPort::INTERNAL_BUFFER_SIZE {
                sleep(Duration::from_millis(1)).await;
            }
            serial_lock.write_all(i).unwrap(); // TODO: fix later
            serial_lock.flush().unwrap();
        }
    }

    pub async fn send_json(&self, data: impl serde::ser::Serialize) {
        // println!("{}", serde_json::to_string(&data).unwrap_or("".to_string()));
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
