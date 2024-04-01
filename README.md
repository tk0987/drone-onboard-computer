# drone_tiltrotor_soft

Still creating. Drone will have 2 servos, 2 motors, rpi pico (one on drone, one in hand), mpu6050 and bmp180 (both i2c), and two i2c transcievers.

Currently only one class for motion processing is quite ready (as i started yesterday).

Another class will drive servos and motors. Another - responsible for communication.

main code on pi pico will process and acquire all data needed

all in micropython.
