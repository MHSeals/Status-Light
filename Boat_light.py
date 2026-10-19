import time
import json
# Status light configuration
LED_LAYERS = 6
COLORS = {
    "OK": [0, 255, 0],        # Green
    "WARNING": [255, 140, 0], # Orange
    "ERROR": [255, 0, 0],     # Red
    "OFF": [0, 0, 0]          # Off
}
# Example status checks
def emergency_stop():
    return False
def hardware_error():
    return False
def communication_warning():
    return False
def system_ready():
    return True
class StatusLight:
    def __init__(self):
        self.statuses = [
            ("EMERGENCY STOP", emergency_stop, "ERROR", 100),
            ("HARDWARE ERROR", hardware_error, "ERROR", 90),
            ("COMMUNICATION WARNING", communication_warning, "WARNING", 60),
            ("SYSTEM READY", system_ready, "OK", 10),
        ]
    def get_status(self):
        active = []
        for name, check_function, color, priority in self.statuses:
            try:
                if check_function():
                    active.append((priority, name, color))
            except Exception as error:
                print(f"Error checking {name}: {error}")
                active.append((90, name, "ERROR"))
        # Highest priority first
        active.sort(reverse=True)
        layers = []
        for priority, name, color in active[:LED_LAYERS]:
            layers.append(COLORS[color])
        # Fill unused layers with OFF
        while len(layers) < LED_LAYERS:
            layers.append(COLORS["OFF"])
        return layers
    def run(self):
        print("RoboBoat Status Light Started")
        while True:
            layers = self.get_status()
            output = {
                "layers": layers
            }
            print(json.dumps(output))
            time.sleep(1)
if __name__ == "__main__":
    status_light = StatusLight()
    try:
        status_light.run()
    except KeyboardInterrupt:
        print("\nStatus Light Stopped")