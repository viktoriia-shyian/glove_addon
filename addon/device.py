import serial

from .command import (Command)

    def connect(self):
        self.serialPort = serial.Serial(
            port=self.serial_port, baudrate=self.serial_baudrate)
        return "Connected"

    def disconnect(self):
        self.serialPort.close()
        return "Disconnected"

    def write(self, message):
        self.serialPort.write(message.encode())
        return "Wrote " + message

    def read(self, expected_lines):
        lines = []
        for i in range(expected_lines):
            lines.append(self.read_line())
        return lines

    def read_line(self):
        line = ""
        read_bytes = self.serialPort.read().decode()
        while read_bytes != '\n':
            line += str(read_bytes)
            read_bytes = self.serialPort.read().decode()
        return line

    def check_connection(self):
        self.write(Command.connection_state.id)
        return self.read(Command.connection_state.expected_lines)

    def set_sensitivity(self):
        self.write(Command.sensitivity.id)
        return self.read(Command.sensitivity.expected_lines)

    def calibrate(self):
        self.write(Command.calibrate.id)
        return self.read(Command.calibrate.expected_lines)

    def get_offsets(self):
        self.write(Command.offsets.id)
        return self.read(Command.offsets.expected_lines)

    def get_raw_data(self):
        self.write(Command.raw_data.id)
        return self.read(Command.raw_data.expected_lines)

    def get_dmp(self):
        self.write(Command.get_dmp.id)
        return self.parse(self.read(Command.get_dmp.expected_lines))

    def start_timer(self):
        self.write(Command.start_timer.id)
        return self.read(Command.start_timer.expected_lines)

    def stop_timer(self):
        self.write(Command.stop_timer.id)
        return self.read(Command.stop_timer.expected_lines)

    def parse(self, list):
        result = []
        for i in list:
            result.append(i.split(';'))
        return result