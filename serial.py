
import machine
import serial


class software_serial:

    def __init__(self, tx=22, rx=21, Inverse=False, buffSize=512):
        self.port = serial.new(tx, rx, Inverse, buffSize)

    def __del__(self):
        serial.del(self.port)

    def any(self):
        return serial.any(self.port)

    def open(self, baudRate):
        return serial.open(self.port, baudRate)

    def stop(self):
        return serial.stop(self.port)

    def write(self, byte):
        return serial.write(self.port, byte)

    def read(self):
        return serial.read(self.port)


def unit_test():
    t = serial.new(22, 21, False, 512)
    serial.stop(t)
    serial.open(t, 115200)
    serial.write(t, 0x11)
    serial.write(t, 0x11)
    serial.any(t)
    serial.read(t)

    # tmp = software_serial()

    # tmp.write(0x12)

    # tmp.__del__()


if __name__ == "__main__":
    unit_test()
