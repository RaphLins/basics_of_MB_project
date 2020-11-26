from Thymio import Thymio, Message

GROUND_THRESHOLD = 500

class MyThymio(Thymio):
    def __init__(self, port="COM3", refreshing_rate=0.1):
        import serial  # pip3 install pyserial
        if port is None:
            port = Thymio.serial_default_port()
        node_id = 1
        Thymio.__init__(self, serial.Serial(port), node_id, refreshing_rate)
        self.handshake()

    def set_speed(self, left, right):
        left = round(left)
        right = round(right)
        MAX_SPEED = 200

        left = min(MAX_SPEED, max(-MAX_SPEED, left))
        right = min(MAX_SPEED, max(-MAX_SPEED, right))

        if left >= 0:
            self.set_var("motor.left.target", left)
        else:
            self.set_var("motor.left.target", 2 ** 16 + left)
        if right >= 0:
            self.set_var("motor.right.target", right)
        else:
            self.set_var("motor.right.target", 2 ** 16 + right)

    def measure_speeds(self):
        speed_left = self["motor.left.speed"]
        speed_right = self["motor.right.speed"]

        if speed_left > 2 ** 16/2:
            speed_left -= 2 ** 16

        if speed_right > 2 ** 16/2:
            speed_right -= 2 ** 16

        if abs(speed_left) < 7:
            speed_left = 0
        if abs(speed_right) < 7:
            speed_right = 0

        return speed_left, speed_right

    def measure_ground_sensors(self):
        ground_left_measure = self["prox.ground.delta"][0]
        ground_right_measure = self["prox.ground.delta"][1]
        #white = 1000, black = 200
        return ground_left_measure > GROUND_THRESHOLD, ground_right_measure > GROUND_THRESHOLD