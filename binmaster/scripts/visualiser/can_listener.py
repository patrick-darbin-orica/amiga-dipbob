import argparse
import subprocess
import can
from protocol import Protocol, PhysicalLayer

# Intitialise serial
phy = PhysicalLayer(baud=1000000, port='/dev/ttyACM0')
mac = Protocol(phy)

def wait_for_can_signal(bus_channel, can_id):
    """Wait for a specific CAN signal to start the cycle."""
    print(f"Listening for CAN messages on {bus_channel}...")
    with can.interface.Bus(channel=bus_channel, interface='socketcan') as bus:
        while True:
            message = bus.recv()  # Blocking call
            if message.arbitration_id == can_id:
                if len(message.data) > 0 and message.data[0] == 0x01:
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
    parser.add_argument('--can-id', help='CAN signal ID', type=int, default=0x123, dest='can_id')
    parser.add_argument('--cycle-script', help='Path to cycle.py script', default='cycle.py', dest='cycle_script')
    args = parser.parse_args()

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
