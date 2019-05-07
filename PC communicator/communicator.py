import sys, glob
import serial
from struct import unpack, pack

commands = {
    "d": bytearray([0x00, 0x00]),
    "b": bytearray([0x01, 0x00]),
    "p": bytearray([0x02, 0x00])
}


class Communicator:
    def __init__(self, port, baud, timeout):
        self.serial = serial.Serial(port, baud, timeout=timeout)

    def write_cmd(self, cmd):
        if isinstance(cmd, bytearray):
            self.serial.write(cmd)
        else:
            self.serial.write(cmd.encode('ascii'))

    def read_trace(self):
        line = self.serial.readline()
        while line:
            print(line)
            line = self.serial.readline()
        print("trace finished")

    def read_line(self, print_out=True):
        line = self.serial.readline().decode().strip()
        if print_out:
            print("NUCLEO ANSWER: ", line)
        return line

    def read_float(self):
        num = self.serial.read(size=4)
        # print("REC FLOAT bytes: ", num)
        return unpack("f", num)[0]

    def read_int(self):
        num = self.serial.read(size=4)
        return unpack("I", num)[0]

    def read_short(self):
        num = self.serial.read(size=2)
        return unpack("H", num)[0]


def serial_ports():
    """ Lists serial port names
        :raises EnvironmentError: On unsupported or unknown platforms
        :returns: A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux'):
        ports = glob.glob('/dev/tty[A,U,a,u]*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


if __name__ == '__main__':
    serials = serial_ports()
    print("Availiable serial ports: ", serial_ports())
    com = Communicator(serials[0], 115200, 20)

    while True:
        print("\nCOMMAND: ", end="")
        cmd = input()
        if cmd == "d":
            direction = input("Desired direction 0/1 [forward/backward]: ")
            if direction != '0' and direction != '1':
                print("ERROR: Bad input!!!")
            else:
                m = commands[cmd] + bytes([int(direction)])
                com.write_cmd(m)
                print("DEBUG: MESSAGE ", m)
                line = com.read_line()

        elif cmd == "b":
            breaks = input("Desired break state 0/1 [turn off/turn on]: ")
            if breaks != '0' and breaks != '1':
                print("ERROR: Bad input!!!")
            else:
                m = commands[cmd] + bytes([int(breaks)])
                com.write_cmd(m)
                print("DEBUG: MESSAGE ", m)
                line = com.read_line()

        elif cmd == "p":
            pwm = input("Desired motor speed, PWM pulse width  in % [0-100]: ")
            pwm = int(pwm)
            if pwm < 0 or pwm > 100:
                print("ERROR: Bad input!!!")
            else:
                m = commands[cmd] + bytes([int(pwm)])
                com.write_cmd(m)
                print("DEBUG: MESSAGE ", m)
                line = com.read_line()

        else:
            print("Bad user input!")
