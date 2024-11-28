import time
from adafruit_servokit import ServoKit

class ServoController:
    def __init__(self, channel=0, min_pulse=1000, max_pulse=2000, reverse_angle=False, max_angle=90):
        # Initialize ServoKit
        self.kit = ServoKit(channels=16)
        self.channel = channel
        self.angle_fix = -5
        
        # Set up the servo
        self.max_angle = max_angle
        # self.kit.servo[self.channel].set_pulse_width_range(min_pulse, max_pulse)
        self.reverse_factor = -1 if reverse_angle else 1

    def set_angle(self, angle):
        """Set the servo to a specific angle (-90 to 90 degrees)."""
        if -90 <= angle <= 90:
            angle += self.angle_fix
            # angle += 90
            mapped_angle = (self.reverse_factor * angle) + 90
            self.kit.servo[self.channel].angle = mapped_angle
        else:
            raise ValueError("Angle must be between -90 and 90 degrees")

    def sweep(self, start=-90, end=90, step=5, delay=0.1):
        """Perform a sweep from start to end angle."""
        for angle in range(start, end + 1, step):
            self.set_angle(angle)
            time.sleep(delay)

    def center(self):
        """Center the servo (set to 0 degrees)."""
        self.set_angle(0)

def create_servo(channel=0, min_pulse=1000, max_pulse=2000, reverse_angle=False, max_angle=90):
    """Create and return a ServoController instance."""
    return ServoController(channel, min_pulse, max_pulse, reverse_angle, max_angle)
