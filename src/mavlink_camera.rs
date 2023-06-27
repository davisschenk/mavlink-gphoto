use heapless::Vec;
use mavlink::ardupilotmega::COMMAND_LONG_DATA;
use mavlink::common::{CameraCapFlags, MavCmd, MavMessage};
use mavlink::error::MessageReadError;
use mavlink::MavConnection;
use std::sync::{Arc, Mutex, RwLock};
use std::{env, thread, time::Duration};

use anyhow::Result;

type Vehicle = Arc<RwLock<Box<dyn MavConnection<MavMessage> + Sync + Send>>>;

struct MavlinkCameraComponent {
    system_id: u8,
    component_id: u8,
    vendor_name: String,
    model_name: String,
}

struct MavlinkCameraInformation {
    component: MavlinkCameraComponent,
    mavlink_connection_string: String,
    vehicle: Vehicle,
}

pub struct MavLinkCameraHandle {
    camera_information: Arc<Mutex<MavlinkCameraInformation>>,
    heartbeat_thread: std::thread::JoinHandle<()>,
    receive_message_thread: std::thread::JoinHandle<()>,
}

impl MavLinkCameraHandle {
    pub fn try_new(mavlink_connection_string: String) -> Result<Self> {
        let component = MavlinkCameraComponent {
            system_id: 100,
            component_id: 100,
            vendor_name: "Davis Vendor".to_owned(),
            model_name: "Davis Model".to_owned(),
        };

        let vehicle = mavlink::connect(&mavlink_connection_string).unwrap();

        let information = Arc::new(Mutex::new(MavlinkCameraInformation {
            component,
            mavlink_connection_string,
            vehicle: Arc::new(RwLock::new(vehicle)),
        }));

        let heartbeat_info = information.clone();
        let heartbeat_thread = thread::spawn(|| camera_heartbeat(heartbeat_info));

        let receive_message_info = information.clone();
        let receive_message_thread = thread::spawn(|| receieve_message(receive_message_info));

        Ok(MavLinkCameraHandle {
            camera_information: information,
            heartbeat_thread,
            receive_message_thread,
        })
    }
}

fn heartbeat_message() -> MavMessage {
    MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_CAMERA,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

fn camera_heartbeat(mavlink_info: Arc<Mutex<MavlinkCameraInformation>>) {
    let information = mavlink_info.lock().unwrap();
    let vehicle = information.vehicle.clone();

    let mut header = mavlink::MavHeader::default();
    header.system_id = information.component.system_id;
    header.component_id = information.component.component_id;
    println!("{header:?}");

    drop(information);

    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));

        if let Err(error) = vehicle.read().unwrap().send(&header, &heartbeat_message()) {
            println!("Failed to send heartbeat: {error}");
        } else {
            println!("Sent heartbeat!")
        }
    }
}

fn receieve_message(mavlink_info: Arc<Mutex<MavlinkCameraInformation>>) {
    let information = mavlink_info.lock().unwrap();
    let vehicle = information.vehicle.clone();

    let mut header = mavlink::MavHeader::default();
    header.system_id = information.component.system_id;
    header.component_id = information.component.component_id;

    drop(information);

    loop {
        thread::sleep(Duration::from_millis(100));

        match vehicle.read().unwrap().recv() {
            Ok((recv_header, recv_msg)) => match recv_msg {
                MavMessage::COMMAND_LONG(command_long) => {
                    send_command_ack(
                        &vehicle,
                        &header,
                        &recv_header,
                        command_long.command,
                        mavlink::common::MavResult::MAV_RESULT_ACCEPTED,
                    );

                    println!("Received Command: {:?}", command_long.command);

                    match command_long {
                        cmd @ mavlink::common::COMMAND_LONG_DATA {param1: 259.0, ..} => {
                            println!("Requesting camera info: {cmd:?}");
                            vehicle.read().unwrap().send(&header, &camera_information());
                        },
                        _ => {}
                    }
                },
                _ => {}
            },
            Err(error) => {}
        }
    }
}

fn send_command_ack(
    vehicle: &Vehicle,
    our_header: &mavlink::MavHeader,
    their_header: &mavlink::MavHeader,
    command: mavlink::common::MavCmd,
    result: mavlink::common::MavResult,
) {
    if let Err(err) = vehicle.read().unwrap().send(
        our_header,
        &MavMessage::COMMAND_ACK(mavlink::common::COMMAND_ACK_DATA {
            command,
            result,
            target_system: their_header.system_id,
            target_component: their_header.component_id,
            ..Default::default()
        }),
    ) {
        eprintln!("Failed to send command ack");
    }
}

pub fn camera_information() -> MavMessage {
    MavMessage::CAMERA_INFORMATION(mavlink::common::CAMERA_INFORMATION_DATA {
        time_boot_ms: (sys_info::boottime().unwrap().tv_usec / 1000) as u32,
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
