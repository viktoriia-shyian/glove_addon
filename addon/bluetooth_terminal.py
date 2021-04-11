import serial
import time
import codecs


class Device():

    serial_port = 'COM5'
    serial_baudrate = 38400
    serialPort

    @staticmethod
    def connect():
        serialPort = serial.Serial(port=serial_port, baudrate=serial_baudrate)

    @staticmethod
    def disconnect():
        serialPort.close()

    @staticmethod
    def calibrate():
        # TODO
        serialPort.write()

    @staticmethod
    def get_data():
        # TODO

    @staticmethod
    def check_connection():
        # TODO

        # while 1:
        #    command_number = input("Enter a command: ")
        #    serialPort.write(command_number.encode())
        #    print("Sent command is: ", command_number)

        # b = serialPort.readline()
        # print(b)
        # print(type(b))
        # # str_rn = codecs.decode(b)
        # str_rn = bytes.decode(b)
        # # .strip()
        # # str_rn = b.encode().strip()
        # print(str_rn)

        # line = []
        # while True:
        #     for c in serialPort.read():
        #         line.append(c)
        #         if c == '\n':
        #             print("Line: " + ''.join(line))
        #             line = []
        #             break
