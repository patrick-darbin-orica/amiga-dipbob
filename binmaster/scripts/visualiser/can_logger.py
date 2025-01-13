import can

def log_can_messages():
    # Initialize the CAN bus interface
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    print("Listening for CAN messages...")

    while True:
        # Wait for a CAN message to be received
        message = bus.recv()
        # Print the received message details
        print(f"ID: {message.arbitration_id}, Data: {message.data}")

if __name__ == "__main__":
    log_can_messages()
