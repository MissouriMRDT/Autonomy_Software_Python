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
magXmin = -0.49546000361442566
magYmin = 0.12809999287128448
magZmin = -0.1325799971818924
magXmax = 0.20958000421524048
magYmax = 0.8580600023269653
magZmax = 0.5951399803161621


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
            rate_gyr_x = GYRx * G_GAIN
            rate_gyr_y = GYRy * G_GAIN
            rate_gyr_z = GYRz * G_GAIN

            # Calculate the angles from the gyro.
            self.gyroXangle += rate_gyr_x*LP
            self.gyroYangle += rate_gyr_y*LP
            self.gyroZangle += rate_gyr_z*LP

            # Convert Accelerometer values to degrees
            AccXangle = (math.atan2(ACCy, ACCz)+M_PI)*RAD_TO_DEG
            AccYangle = (math.atan2(ACCz, ACCx)+M_PI)*RAD_TO_DEG

            ####################################################################
            ######################Correct rotation value########################
            ####################################################################
            # Change the rotation value of the accelerometer to -/+ 180 and
            # move the Y axis '0' point to up.
            #
            # Two different pieces of code are used depending on how your IMU is mounted.
            # If IMU is up the correct way, Skull logo is facing down, Use these lines
            AccXangle -= 180.0
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle += 90.0
            #
            #
            #
            #
            # If IMU is upside down E.g Skull logo is facing up;
            # if AccXangle >180:
                #        AccXangle -= 360.0
            # AccYangle-=90
            # if (AccYangle >180):
                #        AccYangle -= 360.0
            ############################ END ##################################

            # Complementary filter used to combine the accelerometer and gyro values.
            self.CFangleX = AA*(self.CFangleX+rate_gyr_x*LP) + (1 - AA) * AccXangle
            self.CFangleY = AA*(self.CFangleY+rate_gyr_y*LP) + (1 - AA) * AccYangle

            # Kalman filter used to combine the accelerometer and gyro values.
            self.kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y, LP)
            self.kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x, LP)

            ####################################################################
            ############################MAG direction ##########################
            ####################################################################
            # If IMU is upside down, then use this line.  It isnt needed if the
            # IMU is the correct way up
            #MAGy = -MAGy
            #
            ############################ END ##################################

            # Calculate heading
            head = 180 * math.atan2(MAGy, MAGx)/M_PI

            # Only have our heading between 0 and 360
            if head < 0:
                head += 360

            self.heading = head

            ####################################################################
            ###################Tilt compensated heading#########################
            ####################################################################
            # Normalize accelerometer raw values.
            self.accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            self.accZnorm = ACCz/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

            # Calculate pitch and roll
            # Use these two lines when the IMU is up the right way. Skull logo is facing down
            self.roll = math.asin(self.accXnorm)
            self.pitch = math.asin(self.accZnorm/math.cos(self.roll))

            # Us these four lines when the IMU is upside down. Skull logo is facing up
            # accXnorm = -accXnorm				#flip Xnorm as the IMU is upside down
            # accYnorm = -accYnorm				#flip Ynorm as the IMU is upside down
            #pitch = math.asin(accXnorm)
            #roll = math.asin(accYnorm/math.cos(pitch))
            #
            # Calculate the new tilt compensated values
            magXcomp = MAGx*math.cos(self.pitch)+MAGz*math.sin(self.pitch)

            # The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
            # is also reversed. This needs to be taken into consideration when performing the calculations
            if(False):
                magYcomp = MAGx*math.sin(self.roll)*math.sin(self.pitch)+MAGy * \
                    math.cos(self.roll)-MAGz*math.sin(self.roll)*math.cos(self.pitch)  # LSM9DS0
            else:
                magYcomp = MAGx*math.sin(self.roll)*math.sin(self.pitch)+MAGy * \
                    math.cos(self.roll)+MAGz*math.sin(self.roll)*math.cos(self.pitch)  # LSM9DS1

            # Calculate tilt compensated heading
            tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp)/M_PI

            if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360

            self.headingCompensated = tiltCompensatedHeading

            ############################ END ##################################

            time.sleep(0.01)
