from std_msgs.msg import String
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
from gpiozero import Servo

class FlywheelSubscriber(Node):

    def __init__(self):
        super().__init__('flywheel_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'flywheel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'flywheel_status', 10)
        self.subscription  # prevent unused variable warning

        # GPIO Setup
        self.ESC_PIN = 18
        self.SERVO_PIN = 21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        GPIO.setup(self.ESC_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.ESC_PIN, 50)  # 50Hz for ESC
        self.pwm.start(0)
        self.p = GPIO.PWM(self.SERVO_PIN, 50)  # 50 Hz frequency
        self.p.start(7.5)  # 7.5% duty cycle -> 90 degrees
        print("should be not moving")
        duty_cycle = 30*0.1
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.p.ChangeDutyCycle(0)
    def listener_callback(self, msg):
        global p #ayo why this broken
        def calibrate_stop():
            #self.my_servo.value = None  # Stop (neutral)
            print("Calibrating Stop Position...")
            #GPIO.output(self.SERVO_POWER, GPIO.HIGH)
        def move_forward():
            duty = 90 / 18 + 2.5
            self.p.ChangeDutyCycle(duty)
            sleep(0.5)  # Allow servo time to reach position
            self.p.ChangeDutyCycle(0)  # Reset PWM to prevent jitter
            print("Moving Forward...")
            #GPIO.output(self.SERVO_POWER, GPIO.LOW)

        def move_reverse():
            duty = 65 / 18 + 2.5
            self.p.ChangeDutyCycle(duty)
            sleep(0.5)  # Allow servo time to reach position
            self.p.ChangeDutyCycle(0)  # Reset PWM to prevent jitter
            print("Moving Reverse...")
            #GPIO.output(self.SERVO_POWER, GPIO.LOW)
        throttle = msg.data
        if throttle == 50:
                #move_reverse()
                #sleep(0.8)
                #throttle = 55
                duty_cycle = throttle*0.1
                self.pwm.ChangeDutyCycle(duty_cycle)
                self.get_logger().info(f'Throttle set to {throttle}%')
                sleep(2)
                move_forward()
                #first ball shot
                sleep(1)
                move_reverse() #0.5
                sleep(3) #1.5
                move_forward() #2
                #second ball shot 0
                move_reverse() #0.5
                sleep(1)#3.5
                move_forward() #4
                #third ball shot
                sleep(0.5)
                move_reverse()
                #send message to say this shit done
                duty_cycle = 0
                throttle = 0
                self.pwm.ChangeDutyCycle(duty_cycle)
                self.get_logger().info(f'Throttle set to {throttle}%')
                done_msg = String()
                done_msg.data = "Flywheel sequence complete"
                self.publisher_.publish(done_msg)
                self.get_logger().info("Published status: Flywheel sequence complete")

        else:
            self.get_logger().warn('Throttle out of range)')

        def destroy_node(self):
                self.pwm.stop()
                GPIO.cleanup()
                super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    flywheel_subscriber = FlywheelSubscriber()

    try:
        rclpy.spin(flywheel_subscriber)
    except KeyboardInterrupt:
        pass

    flywheel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()