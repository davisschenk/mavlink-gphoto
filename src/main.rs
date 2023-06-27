use mavlink_camera::MavLinkCameraHandle;
mod mavlink_camera;

const CONNECTION: &str = "tcpout:localhost:5762";

fn main() {
    let handle = MavLinkCameraHandle::try_new(CONNECTION.into());
    loop {}
}
