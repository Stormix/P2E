import serial
import time
from time import sleep 
import struct


SERIAL_PORT = 'COM3'
SERIAL_RATE = 115200

dataBuffer = [0x00]*20 # Contains bytes read from the serial buffer
messageBuffer = [0x00]*40 # TODO: remove messageBuffer, it's useless
bufferIndex = 0 # used to go through the data
buttons = [False] * 5 #Buttons default status

def main():
    global dataBuffer,messageBuffer,bufferIndex

    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE,timeout=100,write_timeout=100,rtscts=True)
    # To check the versions
    # ser.write("VERSION?\r\n".encode())
    # reading = ser.readline()
    # print(reading)
    # sleep(2)
    ser.write("START\r\n".encode())
    sleep(2)
    while ser.in_waiting:  # Or: while ser.in_waiting
        num = ord(ser.read())
        if bufferIndex == 0:
            # TODO: figurout the other read modes & their use cases  
            if num != -1:
                if num !=  85:
                    currentReadMode = "message"
                else:
                    currentReadMode = "DATA"
            else:
                currentReadMode = "None"
        if currentReadMode == "DATA":
            if num > -1:
                dataBuffer[bufferIndex] = num
                bufferIndex += 1
            if bufferIndex == 20:
                bufferIndex = 0
                HandleNewBytes(dataBuffer)

def HandleNewBytes(dataBuffer):
    global buttons
    num = dataBuffer[1]
    buttons[0] = (num & 1) != 0
    buttons[1] = (num & 2) != 0
    buttons[2] = (num & 4) != 0
    buttons[3] = (num & 8) != 0
    buttons[4] = (num & 16) != 0
    x = bit32ToFloat(dataBuffer[2:6])
    y = bit32ToFloat(dataBuffer[6:10])
    z = bit32ToFloat(dataBuffer[10:14])
    w = bit32ToFloat(dataBuffer[14:18])
    Voltage = (dataBuffer[18] & 127) / 10
    # (x,y,z,w) formes un quaternions 
    # mais le gyroscope ne nous retourne que 3 angles, les angles d'euler (precession, nutation, rotation propre)
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # faut créer une fonction qui prend le role/pitch/yaw et retourne 4 angles
    print(buttons)  
    print(x,y,z,w,Voltage)

def bit32ToFloat(c):
    byts = bytearray()
    byts.extend(c)
    return struct.unpack('<f', byts)[0]

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
def quaternion_to_euler(x, y, z, w):
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z
    return [qx, qy, qz, qw]
if __name__ == "__main__":
    main()