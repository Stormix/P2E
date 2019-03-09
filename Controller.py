import serial
import time
from Helper import Helper
from Quaternions import Quaternion


class Controller():
    def __init__(self, port):
        self.port = port
        self.rate = 115200
        self.status = "Disconnected"
        self.error = False
        self.serial = None
        self.delay = 3
        # Contains bytes read from the serial buffer
        self.dataBuffer = [0x00]*20
        self.bufferIndex = 0  # used to go through the data
        self.currentReadMode = None
        self.buttons = [False] * 5
        self.gyroData = None
        self.Voltage = None
        self.stop = False

    def __str__(self):
        return """Status: {}, Errored: {}""".format(self.status, self.error)

    def connect(self):
        try:
            self.serial = serial.Serial(
                self.port, self.rate, timeout=1000, write_timeout=1000, rtscts=True)
            self.status = "Connected"
        except expression as identifier:
            self.error = True
            self.CleanUpPort()

    def queryVersion(self):
        try:
            self.serial.write("VERSION?\r\n".encode())
            time.sleep(self.delay)
            print(self.serial.readline())
        except expression as identifier:
            self.error = True
            self.CleanUpPort()

    def querySerial(self):
        try:
            self.serial.write("SERIAL?\r\n".encode())
            time.sleep(self.delay)
            print(self.serial.readline())
        except expression as identifier:
            self.error = True
            self.CleanUpPort()

    def startData(self):
        try:
            self.serial.write("START\r\n".encode())
            self.status = "Communicating"
            time.sleep(self.delay)
        except expression as identifier:
            self.error = True
            self.CleanUpPort()

    def stopData(self):
        try:
            self.serial.write("STOP\r\n".encode())
            self.status = "Connected"
            time.sleep(self.delay)
        except expression as identifier:
            self.error = True
            self.CleanUpPort()

    def getData(self):
        if self.status != "Communicating":
            self.startData()
        while not self.stop:  # Or: while ser.inWaiting()
            num = ord(self.serial.read())
            if self.bufferIndex == 0:
                # TODO: figurout the other read modes & their use cases
                if num != -1:
                    if num != 85:
                        self.currentReadMode = "message"
                    else:
                        self.currentReadMode = "DATA"
            if self.currentReadMode == "DATA":
                if num > -1:
                    self.dataBuffer[self.bufferIndex] = num
                    self.bufferIndex += 1
                if self.bufferIndex == 20:
                    self.bufferIndex = 0
                    self.handleNewBytes(self.dataBuffer)
                    print(self.displayButtons(self.buttons), self.Voltage,
                          self.gyroData)

    def handleNewBytes(self, dataBuffer=None):
        try:
            if dataBuffer:
                self.dataBuffer = dataBuffer
            num = self.dataBuffer[1]
            self.buttons[0] = (num & 1) != 0
            self.buttons[1] = (num & 2) != 0
            self.buttons[2] = (num & 4) != 0
            self.buttons[3] = (num & 8) != 0
            self.buttons[4] = (num & 16) != 0
            x = Helper.bit32ToFloat(self.dataBuffer[2:6])
            y = Helper.bit32ToFloat(self.dataBuffer[6:10])
            z = Helper.bit32ToFloat(self.dataBuffer[10:14])
            w = Helper.bit32ToFloat(self.dataBuffer[14:18])
            self.gyroData = Quaternion(x, y, z, w)
            self.Voltage = (dataBuffer[18] & 127) / 10
        except Exception as e:
            print(e.__doc__)
            self.error = True
            self.stopData()
            self.CleanUpPort()

    def displayButtons(self, buttons):
        return """
        ____________________

        | {} | {} | {} | {} | {} |
        ____________________
        """.format("x" if buttons[0] else "-", "x" if buttons[1] else "-", "x" if buttons[2] else "-", "x" if buttons[3] else "-", "x" if buttons[4] else "-")

    def CleanUpPort(self):
        if self.serial:
            self.serial.close()
            self.serial = None
            self.status = "Disconnected"
