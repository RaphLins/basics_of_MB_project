from Thymio import Thymio, Message

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
        left = min(255, max(-255, left))
        right = min(255, max(-255, right))

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

        return speed_left, speed_right