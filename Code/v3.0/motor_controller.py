from gpiozero import PWMOutputDevice, Buzzer

buzzer = Buzzer(23)
class DCMotorController:
    def __init__(self, forward_pin=17, reverse_pin=27):
        self.motor_forward = PWMOutputDevice(forward_pin)
        self.motor_reverse = PWMOutputDevice(reverse_pin)
        

    def set_speed(self, speed):
        """
        Set the motor speed and direction.
        :param speed: Float between -1 and 1. Negative for reverse, positive for forward.
        """
        if speed < -1 or speed > 1:
            raise ValueError("Speed must be between -1 and 1")

        if speed < 0:
            self.motor_forward.value = -speed
            self.motor_reverse.off()
        elif speed > 0:
            self.motor_reverse.value = speed
            self.motor_forward.off()
        else:
            self.motor_forward.off()
            self.motor_reverse.off()

    def stop(self):
        """Stop the motor."""
        self.motor_forward.off()
        self.motor_reverse.off()
        
    @staticmethod
    def buzzer_on():
        buzzer.on()
   
    @staticmethod
    def buzzer_off():
        buzzer.off()

def create_dc_motor(forward_pin=17, reverse_pin=27):
    """Create and return a DCMotorController instance."""
    return DCMotorController(forward_pin, reverse_pin)


