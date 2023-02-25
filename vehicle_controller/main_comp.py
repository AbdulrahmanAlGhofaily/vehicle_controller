#!/usr/bin/env python3

# Make sure to run: sudo chmod 666 /dev/ttyUSB0
# sudo chmod 777 /dev/gpiomem

import rclpy
import time
import math
import RPi.GPIO as GPIO
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import threading
import asyncio
from rplidar import RPLidar
from statistics import mean

class MainComp:
    lidarWorking = False
    dir1 = [] # Front
    dir2 = [] # Left
    dir3 = [] # Back
    dir4 = [] # Right
    moveDir1 = False
    moveDir2 = False
    moveDir3 = False
    moveDir4 = False

    # Define constructure
    def __init__(self, portName: str, baudrate: int, timeout: int, rightMotor: int, rightMotorOpp: int, leftMotor: int, leftMotorOpp: int, PwmLeft: int, PwmRight:int):
        # Define lidar attr
        self.portName = portName
        self.baudrate = baudrate
        self.timeout = timeout

        # Define motor controller pins
        self.rightMotor = rightMotor # Backward
        self.rightMotorOpp = rightMotorOpp # Forward
        self.leftMotor = leftMotor # Forward
        self.leftMotorOpp = leftMotorOpp # Backward

        self.PwmLeft = PwmLeft
        self.PwmRight = PwmRight

        # Setup GPIO
        self.setupGPIO()

        # GUI
        self.GUI()
        
    # Connect the lidar and start it
    def connectLidar(self):
        try:
            if not self.lidarWorking:
                self.lidar = RPLidar(self.portName, self.baudrate, self.timeout)
                self.lidarWorking = True

            return
        except Exception as e:
            print(str(e))

    # Stop the lidar and discconect it
    def stopLidar(self):
        try:
            if self.lidarWorking:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                self.lidarWorking = False
                GPIO.cleanup()
        except Exception as e:
            print(str(e))

    # Reading the data from the lidar
    def readingData(self):
        try:
            if self.lidarWorking:
                for i, scan in enumerate(self.lidar.iter_scans(max_buf_meas=800)):
                    if i % 10 == 0:
                        for data in scan:
                            angle = data[1]
                            distance = data[2]
                            print(angle, distance)
                            self.filterData(angle, distance)
                            
                        self.voteDirection()
                        # print(f'Forward: {self.moveDir1}, Left: {self.moveDir2}, Back: {self.moveDir3}, Right: {self.moveDir4}')
                        self.moveVehicle()
                        self.clearDirectionData()

                        if(not self.lidarWorking):
                            break

            return

        except Exception as e:
            if(self.lidarWorking):
                self.threadingMed()
            print(str(e))
            return

    def filterData(self, angle: float, distance: float):
        try:
            if(angle >= 330 and angle <= 30):
                self.dir1.append(distance)

            if(angle >= 260 and angle <= 330):
                self.dir2.append(distance)

            if(angle >= 100 and angle <= 260):
                self.dir3.append(distance)

            if(angle >= 30 and angle <= 100):
                self.dir4.append(distance)
            
            return

        except Exception as e:
            print(str(e))
            return

    def voteDirection(self):
        try:
            avg1 = self.directionAvg(self.dir1)
            avg2 = self.directionAvg(self.dir2)
            avg3 = self.directionAvg(self.dir3)
            avg4 = self.directionAvg(self.dir4)
            # print(avg1, avg2, avg3 ,avg4)

            if(not avg1 is None and avg1 >= 500):
                self.moveDir1 = True
                self.moveDir2 = False
                self.moveDir3 = False
                self.moveDir4 = False
                return
            
            if(not avg2 is None and avg2 >= 500 and (not avg4 is None and avg2 >= avg4)):
                self.moveDir1 = False
                self.moveDir2 = True
                self.moveDir3 = False
                self.moveDir4 = False
                return
            
            if(not avg4 is None and avg4 >= 500 and (not avg2 is None and avg4 >= avg2)):
                self.moveDir1 = False
                self.moveDir2 = False
                self.moveDir3 = False
                self.moveDir4 = True
                return

            if(not avg3 is None and avg3 >= 500):
                self.moveDir1 = False
                self.moveDir2 = False
                self.moveDir3 = True
                self.moveDir4 = False
                return
            
            
        except Exception as e:
            print(str(e))
            return


    def directionAvg(self, dirList):
        try:
            if(len(dirList) > 5):
                return round(mean(dirList), 3)
        except Exception as e:
            print(str(e))
            return
            

    def clearDirectionData(self):
        try:
            self.dir1 = []
            self.dir2 = []
            self.dir3 = []
            self.dir4 = []

            return
        
        except Exception as e:
            print(str(e))
            return
        
    # Setup GPIO pins
    def setupGPIO(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.leftMotor, GPIO.OUT)
        GPIO.setup(self.leftMotorOpp, GPIO.OUT)
        GPIO.setup(self.rightMotor, GPIO.OUT)
        GPIO.setup(self.rightMotorOpp, GPIO.OUT)

        GPIO.setup(self.PwmRight, GPIO.OUT)
        GPIO.setup(self.PwmLeft, GPIO.OUT)

        self.PwmLeft = GPIO.PWM(self.PwmLeft, 500)
        self.PwmLeft.start(0)
        self.PwmRight = GPIO.PWM(self.PwmRight, 500)
        self.PwmRight.start(0)

        

    def moveVehicle(self):
        print(self.moveDir1)
        if(self.moveDir1):
            self.PwmLeft.start(100)
            GPIO.output(self.leftMotor, True)
            GPIO.output(self.leftMotorOpp, False)
            self.PwmRight.start(100)
            GPIO.output(self.rightMotorOpp, True)
            GPIO.output(self.rightMotor, False)

        elif(self.moveDir2):
            self.PwmLeft.start(70)
            GPIO.output(self.leftMotor, False)
            GPIO.output(self.leftMotorOpp, True)
            self.PwmRight.start(70)
            GPIO.output(self.rightMotorOpp, True)
            GPIO.output(self.rightMotor, False)

        elif(self.moveDir3):
            self.PwmLeft.start(50)
            GPIO.output(self.leftMotor, False)
            GPIO.output(self.leftMotorOpp, True)
            self.PwmRight.start(50)
            GPIO.output(self.rightMotorOpp, False)
            GPIO.output(self.rightMotor, True)

        elif(self.moveDir4):
            self.PwmLeft.start(70)
            GPIO.output(self.leftMotor, True)
            GPIO.output(self.leftMotorOpp, False)
            self.PwmRight.start(70)
            GPIO.output(self.rightMotorOpp, False)
            GPIO.output(self.rightMotor, True)
            



    # Tkinter threading
    def threadingMed(self):
        try:
            t1 = threading.Thread(target=self.readingData)
            t1.start()
            return
        except Exception as e:
            print(str(e))
            return

    # GUI to control the lidar
    def GUI(self):
        try:
            # Initialize tkinter GUI
            root = tk.Tk()
            root.geometry('200x150')

            root.title("Lidar control")

            startBtn = tk.Button(root, text="Start Lidar", command=self.connectLidar)
            readingDataBtn = tk.Button(root, text="Reading Data", command=self.threadingMed)
            stopBtn = tk.Button(root, text="Stop Lidar", command=self.stopLidar)

            startBtn.pack()
            readingDataBtn.pack(pady=10)
            stopBtn.pack()

            root.mainloop()
            return
        except Exception as e:
            print(str(e))
            return
        
def main(args=None):
    newScan = MainComp("/dev/ttyUSB0", 256000, 3, 15, 16, 18, 22, 29, 11)

if __name__ == '__main__':
    main()