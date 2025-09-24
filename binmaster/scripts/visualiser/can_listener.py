import json
import argparse
import can
import logging
import time
from protocol import Protocol, PhysicalLayer
from datetime import datetime
from util import detect_bottom
import os
import csv

# Log and CSV file directories
LOG_DIR = "/home/ubuntu/amiga-dipbob/binmaster/scripts/visualiser/logs"
CSV_DIR = "/home/ubuntu/amiga-dipbob/binmaster/scripts/visualiser/csvs"

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CSV_DIR, exist_ok=True)

# Create new files


def get_next_file_number(directory, extension):
    files = [f for f in os.listdir(directory) if f.endswith(extension)]
    if not files:
        return 1
    numbers = [int(f.split('.')[0].split('_')[-1]) for f in files]
    return max(numbers) + 1


# Generate auto-incremented file paths
log_file_number = get_next_file_number(LOG_DIR, ".log")
csv_file_number = get_next_file_number(CSV_DIR, ".csv")
log_file_path = os.path.join(LOG_DIR, f"log_{log_file_number}.log")
csv_file_path = os.path.join(CSV_DIR, f"log_{csv_file_number}.csv")

# Configure logging
logging.basicConfig(
    filename=log_file_path,
    level=logging.INFO,  # Log only INFO-level messages and above
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Initialize serial
phy = PhysicalLayer(baud=1000000, port='/dev/ttyACM0')
mac = Protocol(phy)

if not os.path.exists(csv_file_path):
    with open(csv_file_path, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Hole Number", "Timestamp",
                            "Depth (mm)", "Water Level (mm)"])


# TODO: Create new function for motor up and motor down signals
# def wait_for_can_signal(bus_channel, can_id, trigger_byte):
#     """Wait for a specific CAN signal to start the cycle."""
#     with can.interface.Bus(channel=bus_channel, interface='socketcan') as bus:
#         while True:
#             message = bus.recv()  # Blocking call
#             if message.arbitration_id == can_id:
#                 # Check for Button A (3rd byte = 02)
#                 if len(message.data) > 2 and message.data[2] & trigger_byte:
#                     logging.info(
#                         f"Signal detected at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
#                     return

# replaces: wait_for_can_signal(bus_channel, can_id, trigger_byte) -> None
def wait_for_can_event(bus_channel, can_id, bitmap, timeout=None):
    """
    Block until a CAN frame with matching can_id arrives where data[2] matches one of the masks.
    `bitmap` is an ordered list of (name, mask) tuples—first match wins.
    Returns the matched `name` (e.g., "UP", "DOWN", "CYCLE").
    """
    with can.interface.Bus(channel=bus_channel, interface='socketcan') as bus:
        prev_pressed = 0
        start = datetime.now()
        while True:
            # short poll so we can do edge-detect & timeout
            message = bus.recv(timeout=0.05)
            if message is None:
                if timeout is not None and (datetime.now() - start).total_seconds() > timeout:
                    return None
                continue
            if message.arbitration_id != can_id or len(message.data) <= 2:
                continue

            b = message.data[2]
            # rising-edge detect on any of the masks
            for name, mask in bitmap:
                pressed = bool(b & mask)
                prev = bool(prev_pressed & mask)
                if pressed and not prev:
                    prev_pressed = b
                    return name
            prev_pressed = b


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='CAN Trigger for Dip and Bob Cycle')
    parser.add_argument('--can-channel', help='CAN bus channel',
                        default='can0', dest='can_channel')
    parser.add_argument('--can-id', help='CAN signal ID',
                        type=int, default=0x18FF0007, dest='can_id')
    parser.add_argument('--hole-number', help='Hole number',
                        type=int, default=1, dest='hole_number')

    args = parser.parse_args()

    # while True:
    #     # Wait for Button A signal
    #     wait_for_can_signal(args.can_channel, args.can_id,
    #                         0x02)  # Execute cycle
    #     # wait_for_can_signal(args.can_channel, args.can_id, 0x08)  # Motor Up
    #     # wait_for_can_signal(args.can_channel, args.can_id, 0x10)  # Motor Down
    #     buf = mac.execute_cycle()
    # Bit masks in message.data[2]
CYCLE_MASK = 0x02
UP_MASK    = 0x08
DOWN_MASK  = 0x10

HOLD_TIMEOUT_S = 0.30  # if we don't see a refresh within this, treat as release

with can.interface.Bus(channel=args.can_channel, interface='socketcan') as bus:
    prev_bits = 0

    # Track "considered held" state + last-seen times
    up_held = False
    down_held = False
    up_last_seen   = 0.0
    down_last_seen = 0.0

    while True:
        now = time.monotonic()

        # Poll CAN (non-blocking-ish)
        msg = bus.recv(timeout=0.05)
        if msg and msg.arbitration_id == args.can_id and len(msg.data) > 2:
            b = msg.data[2]

            # --- Cycle (rising edge only) ---
            if (b & CYCLE_MASK) and not (prev_bits & CYCLE_MASK):
                print("Cycle command received")
                buf = mac.execute_cycle()
                try:
                    bottom_info = detect_bottom(buf)
                    depth = bottom_info['depth']
                    water_level = bottom_info.get('water_level', 'N/A')
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                    logging.info(f"Hole {args.hole_number}: Depth={depth} mm, Water Level={water_level}")
                    with open(csv_file_path, 'a', newline='') as csv_file:
                        csv.writer(csv_file).writerow([args.hole_number, timestamp, depth, water_level])
                    print(f"Hole {args.hole_number}: Depth={depth} mm, Water Level={water_level}")
                    args.hole_number += 1
                except Exception as e:
                    logging.error(f"Error during bottom detection: {e}")
                    print(f"Error: {e}")

            # --- PTO: UP press/hold/release ---
            if (b & UP_MASK):
                # rising edge → start
                if not up_held:
                    mac.send_motor_up()
                    up_held = True
                    print("Motor up (press)")
                up_last_seen = now  # refresh heartbeat while held
            else:
                # explicit falling edge (if a release frame actually arrives)
                if up_held and (prev_bits & UP_MASK):
                    mac.send_motor_stop()
                    up_held = False
                    print("Motor stop (UP release)")

            # --- PTO: DOWN press/hold/release ---
            if (b & DOWN_MASK):
                if not down_held:
                    mac.send_motor_down()
                    down_held = True
                    print("Motor down (press)")
                down_last_seen = now
            else:
                if down_held and (prev_bits & DOWN_MASK):
                    mac.send_motor_stop()
                    down_held = False
                    print("Motor stop (DOWN release)")

            # --- Safety: both engaged -> stop ---
            if (b & UP_MASK) and (b & DOWN_MASK):
                mac.send_motor_stop()
                up_held = down_held = False
                print("Motor stop (both pressed)")

            print(f"bits: prev=0x{prev_bits:02X} -> now=0x{b:02X}")
            prev_bits = b

        # -------- Watchdog for missing release frames --------
        # If the bus goes quiet after a press, these will still stop the motor.
        if up_held and (now - up_last_seen) > HOLD_TIMEOUT_S:
            mac.send_motor_stop()
            up_held = False
            print("Motor stop (UP timeout)")

        if down_held and (now - down_last_seen) > HOLD_TIMEOUT_S:
            mac.send_motor_stop()
            down_held = False
            print("Motor stop (DOWN timeout)")
