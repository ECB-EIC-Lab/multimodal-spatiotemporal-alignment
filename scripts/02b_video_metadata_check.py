import time
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30
)

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

profile = pipeline.start(config)

print("=" * 70)
print("Pipeline started")
print("Warm-up 3 seconds...")
print("=" * 70)

time.sleep(3)

try:
    for i in range(10):

        frames = pipeline.wait_for_frames()

        for name, frame in [
            ("Depth", frames.get_depth_frame()),
            ("RGB", frames.get_color_frame()),
        ]:

            if not frame:
                continue

            print()
            print("=" * 70)
            print("Stream:", name)
            print("Frame number:", frame.get_frame_number())
            print("SDK timestamp:", frame.get_timestamp())
            print(
                "Timestamp domain:",
                frame.get_frame_timestamp_domain()
            )

            items = [
                (
                    "SENSOR_TIMESTAMP",
                    rs.frame_metadata_value.sensor_timestamp
                ),
                (
                    "FRAME_TIMESTAMP",
                    rs.frame_metadata_value.frame_timestamp
                ),
                (
                    "BACKEND_TIMESTAMP",
                    rs.frame_metadata_value.backend_timestamp
                ),
                (
                    "TIME_OF_ARRIVAL",
                    rs.frame_metadata_value.time_of_arrival
                ),
            ]

            for label, item in items:

                try:
                    supported = frame.supports_frame_metadata(item)

                    if supported:
                        value = frame.get_frame_metadata(item)

                        print(
                            f"{label}: supported=True, value={value}"
                        )
                    else:
                        print(
                            f"{label}: supported=False"
                        )

                except Exception as e:
                    print(
                        f"{label}: ERROR {e}"
                    )

finally:
    pipeline.stop()

print()
print("Finished")
