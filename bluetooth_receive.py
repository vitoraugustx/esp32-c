import asyncio
from bleak import BleakScanner, BleakClient
from Crypto.Cipher import AES
from Crypto.Util.Padding import unpad
from base64 import b64decode

aes_iv = bytes([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

class NotificationHandler:
    def __init__(self, aes_iv):
        self.notifications = []
        self.aes_key = None
        self.aes_iv = aes_iv
        self.first_message_received = False

    def handle(self, sender, data):
        try:
            if not self.first_message_received:
                self.set_key(data)
                self.first_message_received = True
                print(f"Key set to: {self.aes_key.hex()}")
            else:
                message = self.uncypher(data)
                self.notifications.append(message)
                print(f"Encrypted notification from {sender} (Handle: {sender.handle}): {data}")
                print(f"Decrypted notification from {sender} (Handle: {sender.handle}): {message}")
        except Exception as e:
            print(f"Notification from {sender} (Handle: {sender.handle}): Error: {str(e)}")

    def set_key(self, data):
        self.aes_key = bytes.fromhex(data.decode('utf-8'))  # Assuming the data contains the key in hex string format
        print(f"Received key: {self.aes_key.hex()}")

    def uncypher(self, data):
        encrypted_message_bytes = b64decode(data)
        cipher = AES.new(self.aes_key, AES.MODE_CBC, self.aes_iv)
        decrypted_message_bytes = unpad(cipher.decrypt(encrypted_message_bytes), AES.block_size)
        return decrypted_message_bytes.decode('utf-8')

async def scan_for_devices():
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Device {device.name} with address {device.address}")

async def list_services_and_characteristics(client):
    services = await client.get_services()
    for service in services:
        print(f"Service: {service.uuid}")
        for characteristic in service.characteristics:
            print(f"  Characteristic: {characteristic.uuid} - {characteristic.properties}")

async def connect_and_listen(target_name, notification_handler):
    devices = await BleakScanner.discover()
    target_device = next((device for device in devices if device.name == target_name), None)
    if target_device:
        print(f"Found target device {target_device.name} with address {target_device.address}, connecting...")
        async with BleakClient(target_device.address) as client:
            await list_services_and_characteristics(client)
            print("Connected. Listening for notifications...")
            
            # Subscribe to notifications for characteristics that support it
            for service in client.services:
                for characteristic in service.characteristics:
                    if "notify" in characteristic.properties:
                        await client.start_notify(characteristic.uuid, notification_handler.handle)
            
            # Keep the connection alive to receive notifications
            await asyncio.sleep(30)  # Adjust the duration as needed
            
            # Unsubscribe from notifications before disconnecting
            for service in client.services:
                for characteristic in service.characteristics:
                    if "notify" in characteristic.properties:
                        await client.stop_notify(characteristic.uuid)
    else:
        print("Target device not found.")

try:
    notification_handler = NotificationHandler(aes_iv)
    asyncio.run(connect_and_listen("ESP32", notification_handler))

    # Print all notifications received
    print("All notifications received:")
    for notification in notification_handler.notifications:
        print(f"Decrypted message: {notification}")

except ValueError:
    print("Invalid hexadecimal key. Please try again.")
