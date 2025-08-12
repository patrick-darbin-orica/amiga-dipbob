import asyncio
import struct
import can
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.gps import gps_pb2
from pathlib import Path

CAN_INTERFACE = "can0"
CAN_ID_GPS = 0x18FF00AA  # Chosen CAN ID for GPS transmission

def encode_gps_frame(lat, lon):
    # Scale to 1e7 and pack as two signed 32-bit integers
    lat_i = int(lat * 1e7)
    lon_i = int(lon * 1e7)
    return struct.pack("<ii", lat_i, lon_i)

async def main(service_config_path: Path):
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
    bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")

    async for event, msg in EventClient(config).subscribe(config.subscriptions[0]):
        if isinstance(msg, gps_pb2.GpsFrame):
            data = encode_gps_frame(msg.latitude, msg.longitude)
            can_msg = can.Message(arbitration_id=CAN_ID_GPS, data=data, is_extended_id=True)
            try:
                bus.send(can_msg)
                print(f"Sent GPS to CAN: {msg.latitude}, {msg.longitude}")
            except can.CanError as e:
                print(f"CAN send failed: {e}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--service-config", type=Path, required=True)
    args = parser.parse_args()
    asyncio.run(main(args.service_config))
