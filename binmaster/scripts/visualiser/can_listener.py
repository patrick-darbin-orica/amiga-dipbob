import json
import argparse
import can
import logging
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
        csv_writer.writerow(["Hole Number", "Timestamp", "Depth (mm)", "Water Level (mm)"])

def wait_for_can_signal(bus_channel, can_id, trigger_byte):
    """Wait for a specific CAN signal to start the cycle."""
    with can.interface.Bus(channel=bus_channel, interface='socketcan') as bus:
        while True:
            message = bus.recv()  # Blocking call
            if message.arbitration_id == can_id:
                if len(message.data) > 2 and message.data[2] & trigger_byte:  # Check for Button A (3rd byte = 02)
                    logging.info(f"Signal detected at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                    return
                
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CAN Trigger for Dip and Bob Cycle')
    parser.add_argument('--can-channel', help='CAN bus channel', default='can0', dest='can_channel')
    parser.add_argument('--can-id', help='CAN signal ID', type=int, default=0x18FF0007, dest='can_id')
    parser.add_argument('--hole-number', help='Hole number', type=int, default=1, dest='hole_number')

    args = parser.parse_args()

    while True:
        # Wait for Button A signal
        wait_for_can_signal(args.can_channel, args.can_id, 0x02)        
        buf = mac.execute_cycle()
        try:
            bottom_info = detect_bottom(buf)
            depth = bottom_info['depth']
            water_level = bottom_info.get('water_level', 'N/A')
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Log results
            logging.info(f"Hole {args.hole_number}: Depth={depth} mm, Water Level={water_level}")

            # Append to CSV
            with open(csv_file_path, 'a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([args.hole_number, timestamp, depth, water_level])

            print(f"Hole {args.hole_number}: Depth={depth} mm, Water Level={water_level}")

            # Increment hole number for the next cycle
            args.hole_number += 1

        except Exception as e:
            logging.error(f"Error during bottom detection: {e}")
            print(f"Error: {e}")