# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.





# Press the green button in the gutter to run the script.
#if __name__ == '__main__':
#    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/



import tkinter as tk
from tkinter import ttk
import serial

class Device():
    serial_port = 'COM3'
    serial_baudrate = 38400
    state = 0
    serialPort = serial.Serial()

    #serialPort

    staticmethod
    def connect(self):
        self.serialPort = serial.Serial(port=self.serial_port, baudrate=self.serial_baudrate)
        self.state = 1

    staticmethod
    def get_data(self):
        while (self.state == 2):
            string = self.serialPort.readline();
            sensor1 = parse(string)
            s1x.set(sensor1[0])
            s1y.set(sensor1[1])
            s1z.set(sensor1[2])

            string = self.serialPort.readline();
            sensor2 = parse(string)
            s2x.set(sensor2[0])
            s2y.set(sensor2[1])
            s2z.set(sensor2[2])

            string = self.serialPort.readline();
            sensor3 = parse(string)
            s3x.set(sensor3[0])
            s3y.set(sensor3[1])
            s3z.set(sensor3[2])

            string = self.serialPort.readline();
            sensor4 = parse(string)
            s4x.set(sensor4[0])
            s4y.set(sensor4[1])
            s4z.set(sensor4[2])

            string = self.serialPort.readline();
            sensor5 = parse(string)
            s5x.set(sensor5[0])
            s5y.set(sensor5[1])
            s5z.set(sensor5[2])

            string = self.serialPort.readline();
            sensor6 = parse(string)
            s6x.set(sensor6[0])
            s6y.set(sensor6[1])
            s6z.set(sensor6[2])

    staticmethod

        self.serial_port.end()
        self.state = 0

    staticmethod
    def print_mes(self):
        while (self.state):
            string = self.serialPort.readline();
            print(string)


device = Device()
window = tk.Tk()

window.title("Test serial")
window.minsize(600, 400)


def startSerial():
    device.connect()
    device.print_mes()


def data():
    device.state = 2
    device.get_data()


def parse(string):
    print(string)
    string = str(string)
    return string.split(";")


def stop():
    device.close()


label1 = ttk.Label(window, text="1")
label2 = ttk.Label(window, text="2")
label3 = ttk.Label(window, text="3")
label4 = ttk.Label(window, text="4")
label5 = ttk.Label(window, text="5")
label6 = ttk.Label(window, text="6")
label1.grid(column=0, row=1)
label2.grid(column=0, row=2)
label3.grid(column=0, row=3)
label4.grid(column=0, row=4)
label5.grid(column=0, row=5)
label6.grid(column=0, row=6)

s1x = tk.StringVar()
s1y = tk.StringVar()
s1z = tk.StringVar()
sens1_x = ttk.Entry(window, width=15, textvariable=s1x)
sens1_y = ttk.Entry(window, width=15, textvariable=s1y)
sens1_z = ttk.Entry(window, width=15, textvariable=s1z)
sens1_x.grid(column=1, row=1)
sens1_y.grid(column=2, row=1)
sens1_z.grid(column=3, row=1)


s2x = tk.StringVar()
s2y = tk.StringVar()
s2z = tk.StringVar()
sens2_x = ttk.Entry(window, width=15, textvariable=s2x)
sens2_y = ttk.Entry(window, width=15, textvariable=s2y)
sens2_z = ttk.Entry(window, width=15, textvariable=s2z)
sens2_x.grid(column=1, row=2)
sens2_y.grid(column=2, row=2)
sens2_z.grid(column=3, row=2)


s3x = tk.StringVar()
s3y = tk.StringVar()
s3z = tk.StringVar()
sens3_x = ttk.Entry(window, width=15, textvariable=s3x)
sens3_y = ttk.Entry(window, width=15, textvariable=s3y)
sens3_z = ttk.Entry(window, width=15, textvariable=s3z)
sens3_x.grid(column=1, row=3)
sens3_y.grid(column=2, row=3)
sens3_z.grid(column=3, row=3)


s4x = tk.StringVar()
s4y = tk.StringVar()
s4z = tk.StringVar()
sens4_x = ttk.Entry(window, width=15, textvariable=s4x)
sens4_y = ttk.Entry(window, width=15, textvariable=s4y)
sens4_z = ttk.Entry(window, width=15, textvariable=s4z)
sens4_x.grid(column=1, row=4)
sens4_y.grid(column=2, row=4)
sens4_z.grid(column=3, row=4)


s5x = tk.StringVar()
s5y = tk.StringVar()
s5z = tk.StringVar()
sens5_x = ttk.Entry(window, width=15, textvariable=s5x)
sens5_y = ttk.Entry(window, width=15, textvariable=s5y)
sens5_z = ttk.Entry(window, width=15, textvariable=s5z)
sens5_x.grid(column=1, row=5)
sens5_y.grid(column=2, row=5)
sens5_z.grid(column=3, row=5)


s6x = tk.StringVar()
s6y = tk.StringVar()
s6z = tk.StringVar()
sens6_x = ttk.Entry(window, width=15, textvariable=s6x)
sens6_y = ttk.Entry(window, width=15, textvariable=s6y)
sens6_z = ttk.Entry(window, width=15, textvariable=s6z)
sens6_x.grid(column=1, row=6)
sens6_y.grid(column=2, row=6)
sens6_z.grid(column=3, row=6)


button = ttk.Button(window, text="Start serial", command=startSerial)
button.grid(column=0, row=7)
button2 = ttk.Button(window, text="Get data", command=data)
button2.grid(column=1, row=7)
button3 = ttk.Button(window, text="Stop", command=stop)
button3.grid(column=2, row=7)

window.mainloop()



