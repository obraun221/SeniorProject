#Import Packages Needed
import time
import board
import adafruit_icm20x
from adafruit_icm20x import MagDataRate
import numpy as np
import imufusion
import math
import asyncio
import moteus

#Pin Breakout for Raspberry Pi and ICM20948 IMU
    #Red is 3V
    #Black is Ground
    #Yellow is SCL
    #Orange is SDA


async def main():
    #Transfer Function for Attitude Control
    num_attitude = [-0.0263345303314015, 0.106955615865643, -0.0806210855342416]
    den_attitude = [1, -1.99021086134684, 0.990259392593513]
    #Transfer Function for Roll Rate Control
    num = [0.00537358566051155, 0.0712339196899109, -0.0753671110794183]
    den = [1, -1.91796891902861, 0.919209313299618]

    #Moment of Intertia of Reaction Wheel
    J = 0.753
    #Moment of Intertia for Flight Vehicle
    Ir = 5.18

    #Establish I2C Protocol
    i2c = board.I2C()

    #Define Accelerometer
    imu = adafruit_icm20x.ICM20948(i2c)
    #Set Sampling Rates (Number of Samples/second)
    sampling_rate = 1000 
    imu.gyro_data_rate = sampling_rate
    imu.accelerometer_data_rate = sampling_rate
    imu.magnetometer_data_rate = MagDataRate.RATE_100HZ

    #Rolling Average Error Compensation
    num_samples = 1000
    #Allocate Arrays
    gyro_offset = np.zeros(3)
    accel_offset = np.zeros(3)
    mag_offset = np.zeros(3)
    #Loop Through the Number of Samples
    for i in range(num_samples):
        gx, gy, gz = imu.gyro
        gyro_offset += np.array([gx, gy, gz])
   
        ax, ay, az = imu.acceleration
        accel_offset += np.array([ax, ay, az])
   
        mx, my, mz = imu.magnetic
   
    #Average the offsets
    gyro_offset /= num_samples
    accel_offset /= num_samples

    #Create AHRS Filter
    ahrs = imufusion.Ahrs()
    roll_angle = 0

    #Desired Attitude (Degrees Relative to Intialization)
    setAttitude = 90

    #Array Containing Last Two Angular Rates
    gz_prev = [0, 0]

    #Set Intial Angular Velocity Conditions
    omegaDesired = 0 #deg/s
    omega_prev = [omegaDesired, omegaDesired]

    #Set Intial Attitude Error (degrees)
    error_prev = [0,0]
    #Set Previous omega desired
    omegaDesired_prev = [0,0]
   
    #Set Output Array
    y_prev = [0,0]
   
    #Set Start Velocity At 0
    commanded_velocity = 0
    #Starting Angle
    roll_angle = 0

    #Create Motor
    c = moteus.Controller()
    await c.set_stop()
    #System Is Activated Control Loop Will Begin
    print("Activated")

    #Continious Control Loop
    while True:
        #Get Angular Rates
        gx, gy, gz = imu.gyro
        gx -= gyro_offset[0]
        gy -= gyro_offset[1]
        gz -= gyro_offset[2]
        #Convert Angular Rates to Deg/s
        gx *= 180/math.pi
        gy *= 180/math.pi
        gz *= 180/math.pi
       
        #Get Accelerations
        ax, ay, az = imu.acceleration
        ax -= accel_offset[0]
        ay -= accel_offset[1]
        az = az - accel_offset[2] + 9.081
       
        #Calculate Current Error (Desired Attitude - Current Attitude (degs))
        error = setAttitude - roll_angle
        #Calculate the Systems Desired Roll Rate
        uterms = num_attitude[0] * error + num_attitude[1] * error_prev[0] + num_attitude[2] * error_prev[1]
        yterms = den_attitude[1] * omegaDesired_prev[0] + den_attitude[2] * omegaDesired_prev[1]
        omegaDesired = uterms - yterms

        #Calculate Roll Rate Error (Desired Roll Rate - Current Roll Rate (deg/s))
        u = omegaDesired - gz
        #Calculate For Output System Roll Rate (deg/s)
        uterms = num[0] * u + num[1] * omega_prev[0] + num[2] * omega_prev[1]
        yterms = den[1] * y_prev[0] + den[2] * y_prev[1]
        y = uterms - yterms
        #Convert Velocity to Hz
        velocity_diff = y * (math.pi/180) * (Ir/J) / (2*math.pi) * 1/10
        #Compute New Motor Velocity (Hz)
        commanded_velocity += velocity_diff

        #Define Maximum Motor Velocity to Establish Wheel Saturation
        max_velocity = 60
        if commanded_velocity > max_velocity:
            commanded_velocity = max_velocity
        elif commanded_velocity<-max_velocity:
            commanded_velocity = -max_velocity

        #Send Velocity Command to the Motor
        await c.set_position(position=math.nan,maximum_torque = 1.5, velocity = commanded_velocity,accel_limit = 40.0)

        #Update Previous Values
        y_prev[1] = y_prev[0]
        y_prev[0] = y
        omega_prev[1] = omega_prev[0]
        omega_prev[0] = u
        error_prev[1] = error_prev[0]
        error_prev[0] = error
        omegaDesired_prev[1] = omegaDesired_prev[0]
        omegaDesired_prev[0] = omegaDesired

        #Store Data In Arrays
        gyro_data = np.array([gx,gy,gz])
        accel_data = np.array([ax,ay,az])

        #Update Orientation with AHRS Filter
        ahrs.update_no_magnetometer(gyro_data,accel_data,1/sampling_rate)
        euler = ahrs.quaternion.to_euler()
       
   
        #Euler Integration to Get Orientation Relative to Start Angle
        delta_angle = gz / sampling_rate
        roll_angle += delta_angle
   
if __name__ == '__main__':
    asyncio.run(main())
