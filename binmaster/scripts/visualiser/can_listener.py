import argparse
import subprocess
import can
import logging
from protocol import Protocol, PhysicalLayer

# Configure logging
logging.basicConfig(
    filename='/binmaster_auto/binmaster/scripts/visualiser/can_listener.log',  # Replace with your desired log file path
    level=logging.INFO,  # Log only INFO-level messages and above
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Intitialise serial
phy = PhysicalLayer(baud=1000000, port='/dev/ttyACM0')
mac = Protocol(phy)

def wait_for_can_signal(bus_channel, can_ids):
    """Wait for a specific CAN signal to start the cycle."""
    print(f"Listening for CAN messages on {bus_channel}...")
    logging.info(f"Listening for CAN messages on {bus_channel}...")
    with can.interface.Bus(channel=bus_channel, interface='socketcan') as bus:
        while True:
            message = bus.recv()  # Blocking call
            if message.arbitration_id == can_ids:
                logging.info(f"Received: ID={message.arbitration_id}, Data={message.data}")
                if len(message.data) > 0 and message.data[0] == 0x01: #Adjust if needed
                    logging.info(f"Valid signal detected: ID={message.arbitration_id}, Data={message.data}")
                    print(f"Received valid CAN signal: ID={message.arbitration_id}, Data={message.data}")
                    return
                else:
                    print(f"Ignored invalid CAN signal: ID={message.arbitration_id}, Data={message.data}")

try:
    wait_for_can_signal('vcan0', 0x123)
    buf = mac.execute_cycle()
    print(f"Cycle completed: {buf}")
except serial.SerialException as e:
    print(f"Serial communication error: {e}. Resetting serial port.")
    if phy.port:
        phy.port.close()
    phy.port = serial.Serial(args.port, args.baud, timeout=1)  # Reinitialize the port
    buf = None

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CAN Trigger for Dip and Bob Cycle')
    parser.add_argument('--can-channel', help='CAN bus channel', default='vcan0', dest='can_channel')
    parser.add_argument('--can-ids', help='Comma-separated list of CAN signal IDs', type=str, default='0x123, 0x18EF0070',dest='can_ids')
    parser.add_argument('--cycle-script', help='Path to cycle.py script', default='cycle.py', dest='cycle_script')
    args = parser.parse_args()
    can_ids = [int(id_str, 16) for id_str in args.can_ids.split(',')]


    while True:
        # Wait for CAN signal
        wait_for_can_signal(args.can_channel, args.can_id)

        # Trigger cycle.py as a subprocess
        result = subprocess.run(
            ['python3', '/home/ubuntu/binmaster_auto/binmaster/scripts/visualiser/cycle.py', '-f', 'logs/latest_cycle.json'],
            capture_output=True,
            text=True
        )

        # Print output from cycle.py
        print(result.stdout)
        print(result.stderr)
