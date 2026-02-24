import threading
import time

class Joypad:
    def __init__(self):
        self.lock = threading.Lock()
        self.parameters = {}
        self.heartbeat_active = True

    def set_parameter(self, key, value):
        with self.lock:
            self.parameters[key] = value

    def get_parameter(self, key):
        with self.lock:
            return self.parameters.get(key, None)

    def joypad_heartbeat(self):
        while self.heartbeat_active:
            print("Sending heartbeat...")
            time.sleep(1)

    def start_heartbeat(self):
        heartbeat_thread = threading.Thread(target=self.joypad_heartbeat)
        heartbeat_thread.start()

    def control_loop(self):
        while True:
            # Simplified control logic
            print("Controlling the rover...")
            time.sleep(0.5)

joypad = Joypad()
joypad.start_heartbeat()
joypad.control_loop()