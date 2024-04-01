import machine
import utime
import ustruct
import sys
import math
from mpu import MPU6050
from bmp import BMP180
machine.freq(125000000)
###############################################################################
# Constants

# I2C address
GYRO_ADDR = 0x68
###############################################################################
# Settings

# Initialize I2C with pins
i2c = machine.I2C(0,scl=machine.Pin(5),sda=machine.Pin(4),freq=300000)
#utime.sleep(0.5)
mpu=MPU6050(i2c, GYRO_ADDR)
bmp=BMP180(i2c)
# devices=i2c.scan()
# for d in devices:
#     print(hex(d))
# ###############################################################################
mpu.wake()
mpu.write_lpf_range(6) # low pass filter, as it is floating!!!
mpu.write_accel_range(3) # lowest sensitivity
mpu.write_gyro_range(3)# lowest sensitivity
print("gX   gY   gZ    TEMP   X    Y    Z    P") # printing to the console

while True: # infinite. written to see & test experimental data
    
    bmp.temperature # bmp180 init
    utime.sleep(0.1) # arbitrary
    gyr=mpu.read_gyro_data() # gyro now
    acc=mpu.read_accel_data()
    temp_C=mpu.read_temperature()
    pressure=bmp.pressure
    gyr_x=gyr[0]
    gyr_y=gyr[1]
    gyr_z=gyr[2]
    acc_x=acc[0]
    acc_y=acc[1]
    acc_z=acc[2]
    print(str(acc_x)+"\t"+ \
          str(acc_y)+"\t"+ \
          str(acc_z)+"\t"+\
          str(temp_C)+"\t"+\
          str(gyr_x)+"\t"+ \
          str(gyr_y)+"\t"+ \
          str(gyr_z)+"\t"+str(pressure)) # printed to the console.
