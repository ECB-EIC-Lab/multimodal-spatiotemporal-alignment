import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

if len(devices) == 0:
    print("ERROR: No RealSense device detected")
    raise SystemExit(1)

for dev in devices:

    print("=" * 80)

    info_items = [
        ("Name", rs.camera_info.name),
        ("Serial", rs.camera_info.serial_number),
        ("Firmware", rs.camera_info.firmware_version),
        ("USB", rs.camera_info.usb_type_descriptor),
        ("Product line", rs.camera_info.product_line),
        ("Physical port", rs.camera_info.physical_port),
    ]

    for name, item in info_items:
        try:
            if dev.supports(item):
                print(f"{name}: {dev.get_info(item)}")
        except Exception:
            pass

    for sensor in dev.query_sensors():

        print()
        print("-" * 80)

        try:
            print(
                "Sensor:",
                sensor.get_info(rs.camera_info.name)
            )
        except Exception:
            print("Sensor: Unknown")

        for p in sensor.get_stream_profiles():

            try:
                vp = p.as_video_stream_profile()

                print(
                    p.stream_type(),
                    f"{vp.width()}x{vp.height()}",
                    p.format(),
                    p.fps(),
                    "Hz"
                )

            except Exception:

                print(
                    p.stream_type(),
                    p.format(),
                    p.fps(),
                    "Hz"
                )
