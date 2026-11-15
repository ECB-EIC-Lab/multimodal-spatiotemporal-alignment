import can
import time
from collections import Counter

print("=" * 70)
print("NERO RAW CAN PASSIVE TEST")
print("No control command will be sent.")
print("=" * 70)

bus = can.Bus(
    interface="agx_cando",
    channel=0,
    bitrate=1_000_000
)

duration = 10.0
start = time.perf_counter()

count = 0
ids = Counter()

try:

    while time.perf_counter() - start < duration:

        msg = bus.recv(timeout=1.0)

        if msg is None:
            continue

        count += 1
        ids[msg.arbitration_id] += 1

        # 前20帧打印出来看看
        if count <= 20:

            print(
                f"ID=0x{msg.arbitration_id:03X}",
                f"DLC={msg.dlc}",
                f"DATA={bytes(msg.data).hex(' ')}",
                f"timestamp={msg.timestamp}"
            )

finally:

    bus.shutdown()


print()
print("=" * 70)
print("RESULT")
print("=" * 70)

print("Total CAN frames:", count)

if count == 0:

    print("NO CAN FRAME RECEIVED")

else:

    print()
    print("CAN IDs:")

    for can_id, n in ids.most_common():

        print(
            f"0x{can_id:03X}",
            "count =",
            n,
            "approx Hz =",
            n / duration
        )
