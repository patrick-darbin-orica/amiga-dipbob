import json
import argparse
import can
import logging
import re, time
import serial
from protocol import Protocol, PhysicalLayer
from datetime import datetime
from util import detect_bottom
import os
import csv

# Log and CSV file directories
LOG_DIR = "/home/ubuntu/amiga-dipbob/binmaster/scripts/visualiser/logs"
CSV_DIR = "/home/ubuntu/amiga-dipbob/binmaster/scripts/visualiser/csvs"

# RS-232 weight indicator config
WEIGHT_DEV = "/dev/ttyUSB0"
WEIGHT_BAUD = 9600

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CSV_DIR, exist_ok=True)

# Create new files


def get_next_file_number(directory, extension):
    files = [f for f in os.listdir(directory) if f.endswith(extension)]
    if not files:
        return 1
    numbers = [int(f.split('.')[0].split('_')[-1]) for f in files]
    return max(numbers) + 1

def _parse_weight_line(line: str):
    # e.g. "ST,GS       0kg"
    s = line.strip()
    if not s: return None
    status = []
    m = re.match(r'^([A-Z]{2})(?:,([A-Z]{2}))?\s*(.*)$', s)
    rest = s
    if m:
        status = [t for t in m.groups()[:2] if t]
        rest = m.group(3)
    m2 = re.search(r'(-?\d+(?:\.\d+)?)\s*([a-zA-Z]+)', rest)
    if not m2: return None
    val = float(m2.group(1))
    unit = m2.group(2).lower()
    if unit == "g": val /= 1000.0
    return (round(val, 3), status)


def read_weight_stable(ser, wait_s: float = 1.5, prefer_stable: bool = True):
    if ser is None: return None
    deadline = time.time() + wait_s
    last = None
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore")
        if not line:
            continue
        parsed = _parse_weight_line(line)
        if not parsed:
            continue
        val, status = parsed
        last = val
        if (not prefer_stable) or ("ST" in status):
            return val
    return last


def read_weight_once(ser, wait_s: float = 1.0):
    """
    Non-blocking-ish: read lines for up to wait_s seconds and return first parsed kg value.
    Falls back to last parsed in window if none parse cleanly early.
    """
    if ser is None:
        return None
    deadline = time.time() + wait_s
    last_val = None
    while time.time() < deadline:
        try:
            line = ser.readline().decode(errors="ignore")
        except Exception:
            break
        if not line:
            continue
        val = _parse_weight_line(line)
        if val is not None:
            return round(val, 3)
        # remember last parsable if needed later
        last_val = val
    return last_val


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

# Open weight serial once
ser_weight = None
if serial:
    try:
        ser_weight = serial.Serial(
            WEIGHT_DEV, WEIGHT_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            xonxoff=False,
            rtscts=False,
        )
        ser_weight.reset_input_buffer()
    except Exception as e:
        logging.warning(f"Weight serial not available on {WEIGHT_DEV}: {e}")


if not os.path.exists(csv_file_path):
    with open(csv_file_path, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Hole Number", "Timestamp",
                            "Depth (mm)", "Water Level (mm)", "Weight (kg)"])

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

                # 1) Execute the cycle (returns raw buffer for depth/water processing)
                buf = mac.execute_cycle()

                # 2) Flush old serial lines and read a fresh, stable weight
                if ser_weight:
                    try:
                        ser_weight.reset_input_buffer()
                    except Exception:
                        pass
                time.sleep(0.6)  # allow a fresh frame to arrive; tune 0.4–1.0s if needed
                weight_kg = read_weight_stable(ser_weight, wait_s=2.0, prefer_stable=True)  # assumes helper is defined

                # 3) Detect bottom / extract depth & water level from 'buf'
                try:
                    bottom_info = detect_bottom(buf)
                    depth = bottom_info['depth']
                    water_level = bottom_info.get('water_level', 'N/A')

                    # 4) Single timestamp for this sample
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                    # 5) Log + CSV exactly once
                    weight_str = f"{weight_kg:.3f}" if weight_kg is not None else ""
                    logging.info(
                        f"Hole {args.hole_number}: Depth={depth} mm, Water Level={water_level}, "
                        f"Weight={weight_str or 'N/A'} kg"
                    )

                    with open(csv_file_path, 'a', newline='') as csv_file:
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow([args.hole_number, timestamp, depth, water_level, weight_str or 'N/A'])

                    print(f"Hole {args.hole_number}: Depth={depth} mm, Water Level={water_level}, "
                        f"Weight={weight_str or 'NA'} kg")

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
