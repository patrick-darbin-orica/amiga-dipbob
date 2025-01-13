import json
import argparse
from protocol import Protocol, PhysicalLayer
from util import detect_bottom

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dip and Bob Visualiser')
    parser.add_argument('-b', '--baud', help='Baudrate', default=1000000, dest='baud', type=int)
    parser.add_argument('-p', '--port', help='Serial Port', default='/dev/ttyACM0', dest='port')
    parser.add_argument('-f', '--file', help='File to log too', default='temp.json', dest='file')
    args = parser.parse_args()

    phy = PhysicalLayer(baud=args.baud, port=args.port)
    mac = Protocol(phy)

    buf = mac.execute_cycle()

try:
    bottom_info = detect_bottom(buf)
    depth = bottom_info['depth']
    print(f"Depth measurement: {depth} mm")

    # Add the bottom detection results to the log data
    buf['bottom_stats'] = bottom_info
except Exception as e:
    print(f"Error in bottom detection: {e}")
    buf['bottom_stats'] = None  # Indicate failure in the log
    depth = None
with open(args.file, 'w') as f:
    json.dump(buf, f)
