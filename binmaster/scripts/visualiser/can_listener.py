import argparse
import subprocess
import can
import logging
from protocol import Protocol, PhysicalLayer

# Configure logging
logging.basicConfig(
    filename='~/binmaster_auto/binmaster/scripts/visualiser/can_listener.log',  # Replace with your desired log file path
    level=logging.INFO,  # Log only INFO-level messages and above
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Initialize serial
phy = PhysicalLayer(baud=1000000, port='/dev/ttyACM0')
mac = Protocol(phy)

def wait_for_can_signal(bus_channel, can_id, trigger_byte):
    """Wait for a specific CAN signal to start the cycle."""
    print(f"Listening for CAN messages on {bus_channel}...")
    logging.info(f"Listening for CAN messages on {bus_channel}...")
    with can.interface.Bus(channel=bus_channel, interface='socketcan') as bus:
        while True:
            message = bus.recv()  # Blocking call
            if message.arbitration_id == can_id:
                logging.info(f"Received: ID={message.arbitration_id}, Data={message.data}")
                if len(message.data) > 2 and message.data[2] & trigger_byte:  # Check for Button A (3rd byte = 02)
                    logging.info(f"Button A detected: ID={message.arbitration_id}, Data={message.data}")
                    print(f"Received valid CAN signal for Button A: ID={message.arbitration_id}, Data={message.data}")
                    return
                else:
                    print(f"Ignored invalid CAN signal: ID={message.arbitration_id}, Data={message.data}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CAN Trigger for Dip and Bob Cycle')
    parser.add_argument('--can-channel', help='CAN bus channel', default='can0', dest='can_channel')
    parser.add_argument('--can-id', help='CAN signal ID', type=int, default=0x18FF0007, dest='can_id')
    parser.add_argument('--cycle-script', help='Path to cycle.py script', default='cycle.py', dest='cycle_script')
    args = parser.parse_args()

    while True:
        # Wait for Button A signal
        wait_for_can_signal(args.can_channel, args.can_id, 0x02)

        # Trigger Dipbob (run cycle.py script as a subprocess)
        result = subprocess.run(
            ['python3', '/home/ubuntu/binmaster_auto/binmaster/scripts/visualiser/cycle.py', '-f', 'logs/latest_cycle.json'],
            capture_output=True,
            text=True
        )

        # Log and print output from cycle.py
        logging.info(f"Cycle completed: {result.stdout}")
        print(result.stdout)
        print(result.stderr)
