use heapless::Vec;
use mavlink::common::{CameraCapFlags, MavMessage};
use mavlink::error::MessageReadError;
use std::{env, sync::Arc, thread, time::Duration};

const CONNECTION: &str = "tcpout:localhost:5762";

fn main() {
    let mut mavconn = mavlink::connect::<MavMessage>(CONNECTION).unwrap();
    mavconn.set_protocol_version(mavlink::MavlinkVersion::V2);

    let vehicle = Arc::new(mavconn);

    let mut header = mavlink::MavHeader::default();
    header.system_id = 2;
    header.component_id = mavlink::common::MavComponent::MAV_COMP_ID_CAMERA as u8;

    thread::spawn({
        let vehicle = vehicle.clone();
        let header = header.clone();

        move || loop {
            let res = vehicle.send(&header, &heartbeat_message());
            println!("Send heartbeat");

            if res.is_ok() {
                thread::sleep(Duration::from_secs(1));
            } else {
                println!("send failed: {res:?}");
            }
        }
    });

    loop {
        match vehicle.recv() {
            Ok((their_header, msg)) => match msg {
                MavMessage::COMMAND_LONG(command_long) => {
                    vehicle.send(
                    &header,
                    &MavMessage::COMMAND_ACK(mavlink::common::COMMAND_ACK_DATA {
                        command: command_long.command,
                        result: mavlink::common::MavResult::MAV_RESULT_ACCEPTED,
                        target_system: their_header.system_id,
                        target_component: their_header.component_id,
                        ..Default::default()
                    }),
                    ).unwrap();
                    println!("Sent ack: {command_long:?}");

                    match command_long.command {
                        command @ mavlink::common::MavCmd::MAV_CMD_REQUEST_MESSAGE => {
                            println!("Message requested {command:?}!");
                            vehicle.send(&header, &camera_information());
                        },
                        _ => {}
                    }

                },
                other @ _ => {println!("{other:?}")}
            },
            Err(MessageReadError::Io(e)) => {
                if let std::io::ErrorKind::WouldBlock = e.kind() {
                    //no messages currently available to receive -- wait a while
                    thread::sleep(Duration::from_secs(1));
                    continue;
                } else {
                    println!("recv error: {e:?}");
                    break;
                }
            }
            // messages that didn't get through due to parser errors are ignored
            _ => {}
        }
    }
}

/// Create a heartbeat message using 'common' dialect
pub fn heartbeat_message() -> MavMessage {
    MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_CAMERA,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 0x3,
    })
}

pub fn camera_information() -> MavMessage {
    MavMessage::CAMERA_INFORMATION(mavlink::common::CAMERA_INFORMATION_DATA {
        time_boot_ms: 0,
        firmware_version: (1 & 0xff) << 24 | 0 << 16 | 0 << 8,
        focal_length: 0.0,
        sensor_size_h: 35.9,
        sensor_size_v: 24.0,
        flags: CameraCapFlags::CAMERA_CAP_FLAGS_CAPTURE_IMAGE
            | CameraCapFlags::CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE,
        resolution_h: 7952,
        resolution_v: 5304,
        cam_definition_version: 1,
        vendor_name: str_to_fixed_arr("Davis Vendor"),
        model_name: str_to_fixed_arr("Sony a7r ii"),
        lens_id: 0,
        cam_definition_uri: string_to_uri("Nill"),
    })
}

fn str_to_fixed_arr<const N: usize>(src: &str) -> [u8; N] {
    let bytes = src.as_bytes();
    let mut dst = [0u8; N];
    let len = std::cmp::min(bytes.len(), N);
    dst[..len].copy_from_slice(&bytes[..len]);
    dst
}

fn string_to_uri<const N: usize>(src: &str) -> Vec<u8, N> {
    Vec::from_slice(src.as_bytes()).unwrap()
}
