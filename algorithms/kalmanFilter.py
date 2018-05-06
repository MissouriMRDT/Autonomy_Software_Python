import time
import math
import datetime
import os
import threading

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
# [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
G_GAIN = 0.070
AA = 0.40      # Complementary filter constant

Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005


################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading values.
magXmin = -0.42965999245643616
magYmin = -0.2641800045967102
magZmin = -0.5138000249862671
magXmax = 0.5223399996757507
magYmax = 0.6645799875259399
magZmax = 0.4537400007247925


class KalmanFilter:
    
    def __init__(self, navBoardRef):
        
        self.navBoard = navBoardRef

        # Kalman filter variables
        self.y_bias = 0.0
        self.x_bias = 0.0
        self.XP = [[0.0, 0.0],
                    [0.0, 0.0]]
        self.YP = [[0.0, 0.0],
                    [0.0, 0.0]]
        self.KFangleX = 0.0
        self.KFangleY = 0.0

        # Returning data variables that are updated
        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0
        self.CFangleX = 0.0
        self.CFangleY = 0.0
        self.kalmanX = 0.0
        self.kalmanY = 0.0

        self.heading = 0
        self.headingCompensated = 0
        self.pitch = 0
        self.roll = 0
        self.accXnorm = 0
        self.accZnorm = 0

        self.loopStart = datetime.datetime.now()

        # start a thread to update this in the background
        self.calcThread = threading.Thread(target=self.calculateLoop)
        self.calcThread.daemon = True
        self.calcThread.start()

    def getHeadings(self):
        return (self.heading, self.headingCompensated)

    def getPitch(self):
        return self.pitch

    def getRoll(self):
        return self.roll

    def kalmanFilterY(self, accAngle, gyroRate, DT):
        y = 0.0
        S = 0.0

        self.KFangleY += DT * (gyroRate - self.y_bias)

        self.YP[0][0] += (- DT * (self.YP[1][0] + self.YP[0][1]) + Q_angle * DT)
        self.YP[0][1] += (- DT * self.YP[1][1])
        self.YP[1][0] += (- DT * self.YP[1][1])
        self.YP[1][1] += (+ Q_gyro * DT)

        y = accAngle - self.KFangleY
        S = self.YP[0][0] + R_angle
        K_0 = self.YP[0][0] / S
        K_1 = self.YP[1][0] / S

        self.KFangleY += (K_0 * y)
        self.y_bias += (K_1 * y)

        self.YP[0][0] -= (K_0 * self.YP[0][0])
        self.YP[0][1] -= (K_0 * self.YP[0][1])
        self.YP[1][0] -= (K_1 * self.YP[0][0])
        self.YP[1][1] -= (K_1 * self.YP[0][1])

        return self.KFangleY


    def kalmanFilterX(self, accAngle, gyroRate, DT):
        x = 0.0
        S = 0.0

        self.KFangleX += DT * (gyroRate - self.x_bias)

        self.XP[0][0] += (- DT * (self.XP[1][0] + self.XP[0][1]) + Q_angle * DT)
        self.XP[0][1] += (- DT * self.XP[1][1])
        self.XP[1][0] += (- DT * self.XP[1][1])
        self.XP[1][1] += (+ Q_gyro * DT)

        x = accAngle - self.KFangleX
        S = self.XP[0][0] + R_angle
        K_0 = self.XP[0][0] / S
        K_1 = self.XP[1][0] / S

        self.KFangleX += (K_0 * x)
        self.x_bias += (K_1 * x)

        self.XP[0][0] -= (K_0 * self.XP[0][0])
        self.XP[0][1] -= (K_0 * self.XP[0][1])
        self.XP[1][0] -= (K_1 * self.XP[0][0])
        self.XP[1][1] -= (K_1 * self.XP[0][1])

        return self.KFangleX

    def interp(self, x, inrange, outrange):
        inspan = inrange[1] - inrange[0]
        outspan = outrange[1] - outrange[0]
        scaled = (x - inrange[0]) / inspan
        return (scaled * outspan) + outrange[0]

    def calculateLoop(self):
    
        while True:

            # Read the accelerometer,gyroscope and magnetometer values
            ACCx, ACCy, ACCz = self.navBoard.accelerometerXYZ()
            GYRx, GYRy, GYRz = self.navBoard.gyroscopeXYZ()
            MAGx, MAGy, MAGz = self.navBoard.magnetometerXYZ()

            # Apply compass calibration
            MAGx -= (magXmin + magXmax) / 2
            MAGy -= (magYmin + magYmax) / 2
            MAGz -= (magZmin + magZmax) / 2

            # Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - self.loopStart
            self.loopStart = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)

            # Convert Gyro raw to degrees per second
            #rate_gyr_x = GYRx * G_GAIN
            #rate_gyr_y = GYRy * G_GAIN
            #rate_gyr_z = GYRz * G_GAIN

            # Calculate the angles from the gyro.
            #self.gyroXangle += rate_gyr_x*LP
            #self.gyroYangle += rate_gyr_y*LP
            #self.gyroZangle += rate_gyr_z*LP

            # Convert Accelerometer values to degrees
            #AccXangle = (math.atan2(ACCy, ACCz)+M_PI)*RAD_TO_DEG
            #AccYangle = (math.atan2(ACCz, ACCx)+M_PI)*RAD_TO_DEG

            ####################################################################
            ######################Correct rotation value########################
            ####################################################################
            # Change the rotation value of the accelerometer to -/+ 180 and
            # move the Y axis '0' point to up.
            #
            # Two different pieces of code are used depending on how your IMU is mounted.
            # If IMU is up the correct way, Skull logo is facing down, Use these lines
            #AccXangle -= 180.0
            #if AccYangle > 90:
            #    AccYangle -= 270.0
            #else:
            #    AccYangle += 90.0
            

            # Complementary filter used to combine the accelerometer and gyro values.
            #self.CFangleX = AA*(self.CFangleX+rate_gyr_x*LP) + (1 - AA) * AccXangle
            #self.CFangleY = AA*(self.CFangleY+rate_gyr_y*LP) + (1 - AA) * AccYangle

            # Kalman filter used to combine the accelerometer and gyro values.
            #self.kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y, LP)
            #self.kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x, LP)
            

            # Calculate heading

            head = (180 * math.atan2(MAGz, MAGx)/M_PI) % 360

            self.heading = head

            ####################################################################
            ###################Tilt compensated heading#########################
            ####################################################################
            # Normalize accelerometer raw values.
            self.accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            self.accZnorm = ACCz/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

            # Calculate pitch and roll
            self.roll = math.asin(self.accXnorm)
            self.pitch = math.asin(self.accZnorm/math.cos(self.roll))

            # Calculate the new tilt compensated values
            magXcomp = MAGx*math.cos(self.pitch)+MAGy*math.sin(self.pitch)

            magZcomp = MAGx*math.sin(self.roll)*math.sin(self.pitch)+MAGz * \
                math.cos(self.roll)-MAGy*math.sin(self.roll)*math.cos(self.pitch)

            # Calculate tilt compensated heading
            tiltCompensatedHeading = (180 * math.atan2(magZcomp, magXcomp)/M_PI) % 360

            self.headingCompensated = tiltCompensatedHeading

            ############################ END ##################################

            time.sleep(0.01)
