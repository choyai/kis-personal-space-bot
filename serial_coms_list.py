# from scipy.spatial import distance as dist
# from imutils import perspective
# from imutils import contours
import numpy as np
# import argparse
# import imutils
# import cv2
import serial
import time
import struct
# from InverseKinematics import *
# from Graph import astar
import pickle
from math import pi
from random import randint

countsPerMillimeter = (321 / 300 * 400) / (np.pi * 10)
countsPerMillimeter_z = (12 * 66) / (np.pi * 12)

# Connect to mcu
setPath = {

}


def autoConnect(baud, portName):
    while(1):
        try:
            serialDevice = serial.Serial()
            serialDevice.baudrate = baud
            # serialDevice.parity = 'E'
            serialDevice.port = portName
            serialDevice.timeout = 1
            serialDevice.rts = 0
            serialDevice.dtr = 0
            serialDevice.open()
            print('connected to mcu')
            return serialDevice
        except:
            print('connection failed')
            pass


def sendCommand(command, ser):
    ser.write(bytes(command))
    # while(1):
    #     if ser.inWaiting() > 0:
    #         # data = ser.read(1)
    #         # print("data =", ord(data))
    #         response = ser.readline().decode('utf-8')
    #         print(response)
    #         # if response != 'received' or 'starting':
    #         #     pass
    #         if response == 'resend':
    #             ser.write(bytes(command))
    #         elif response == 'done':
    #             return 1
    #         elif response == 'starting':
    #             break
    #         else:
    #             pass
    #         #     return 0
    #         #     break
    #     # else:
    #     #     ser.write(bytes(command))


def setHome(ser):
    buffer = [255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    checksum = 0
    for i in buffer:
        checksum += i
    checksum = checksum % 256
    buffer.append(checksum)
    print('sending ')
    print(buffer)
    sendCommand(buffer, ser)


def setPosXY(x, y, ser):
    buffer = [255, 255, 1]
    # a = int(np.sqrt(2) / 2 * (y - x))
    # b = int(np.sqrt(2) / 2 * (y + x))
    print("x = " + str(x / countsPerMillimeter))
    print("y = " + str(y / countsPerMillimeter))
    # print("a = " + str(a))
    # print("b = " + str(b))
    x_sign = 0 if x >= 0 else 1
    y_sign = 0 if y >= 0 else 1
    buffer.append(x_sign)
    buffer.extend(split_large_ints(abs(x)))
    buffer.append(y_sign)
    buffer.extend(split_large_ints(abs(y)))
    checksum = 0
    for i in buffer:
        checksum += i
    checksum = checksum % 256
    buffer.append(checksum)
    print('sending ')
    print(buffer)
    sendCommand(buffer, ser)
#
#
# def setPosXY_mm(x, y, ser, x_pix, y_pix, countsPerMillimeter=countsPerMillimeter):
#     buffer = [255, 255, 1]
#     x = x * countsPerMillimeter
#     y = y * countsPerMillimeter
#
#     a = int(np.sqrt(2) / 2 * (y - x))
#     b = int(np.sqrt(2) / 2 * (y + x))
#     print("x = " + str(x / countsPerMillimeter))
#     print("y = " + str(y / countsPerMillimeter))
#     print("a = " + str(a))
#     print("b = " + str(b))
#     a_sign = 0 if a >= 0 else 1
#     b_sign = 0 if b >= 0 else 1
#     buffer.extend(split_large_ints(abs(a)))
#     buffer.extend(split_large_ints(abs(b)))
#     buffer.extend([a_sign, b_sign])
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def setPosZ(z, ser):
#     buffer = [255, 255, 2]
#     buffer.extend(split_large_ints(z))
#     buffer.extend([0, 0, 0, 0])
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def setPosZ_mm(z, ser, countsPerMillimeter_z):
#     buffer = [255, 255, 2]
#     z = int(z * countsPerMillimeter_z)
#     buffer.extend(split_large_ints(z))
#     buffer.extend([0, 0, 0, 0])
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def gripClose(ser):
#     buffer = [255, 255, 3, 0, 0, 0, 0, 0, 0]
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def gripOpen(ser):
#     buffer = [255, 255, 4, 0, 0, 0, 0, 0, 0]
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def gripHalf(duty, ser):
#     buffer = [255, 255, 10, ]
#     buffer.extend(split_large_ints(duty))
#     buffer.extend([0, 0, 0, 0])
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def gripRotate(angle, ser):
#     buffer = [255, 255, 5]
#     buffer.extend(split_large_ints(angle))
#     buffer.extend([0, 0, 0, 0])
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def setAGains(K_P, K_I, K_D, ser):
#     buffer = [255, 255, 6]
#     buffer.extend(split_floats(K_P))
#     buffer.extend(split_floats(K_I))
#     buffer.extend(split_floats(K_D))
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def setBGains(K_P, K_I, K_D, ser):
#     buffer = [255, 255, 7]
#     buffer.extend(split_floats(K_P))
#     buffer.extend(split_floats(K_I))
#     buffer.extend(split_floats(K_D))
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def setZGains(K_P, K_I, K_D, ser):
#     buffer = [255, 255, 8]
#     buffer.extend(split_floats(K_P))
#     buffer.extend(split_floats(K_I))
#     buffer.extend(split_floats(K_D))
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)
#
#
# def setTolerances(ser):
#     buffer = [255, 255, 9]
#     tolerances = []
#     for al in ['a', 'b', 'z']:
#         while(1):
#             try:
#                 tolerance = (int(input("set tolerance_" + al + ": ")))
#                 buffer.extend(split_large_ints(tolerance))
#                 break
#             except:
#                 print("try again")
#     checksum = 0
#     for i in buffer:
#         checksum += i
#     checksum = checksum % 256
#     buffer.append(checksum)
#     print('sending ')
#     print(buffer)
#     sendCommand(buffer, ser)

# splits large ints into msb and lsb. Doesn't support ints larger than 16 bits


def split_large_ints(num):
    # numstring = str(hex(num))
    # lsB = '0x'
    # msB = '0x'
    # if len(numstring) < 5:
    #     msB = '0x00'
    # else:
    #     if len(numstring) == 5:
    #         msB += numstring[2]
    #     else:
    #         msB = msB + numstring[len(numstring) - 4] + \
    #             numstring[len(numstring) - 3]
    # if len(numstring) < 4:
    #     lsB += numstring[len(numstring) - 1]
    # else:
    #     lsB = lsB + numstring[len(numstring) - 2] + \
    #         numstring[len(numstring) - 1]
    msB = (num // 256) % 256
    lsB = num % 256

    return [msB, lsB]


# splits floats from their decimals and turns them into ints
def split_floats(num):
    a, b = divmod(num, 1.0)
    a = int(a) % 256
    b = int(b * 256)
    return [a, b]
