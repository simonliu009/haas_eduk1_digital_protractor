import sh1106
import utime, math
from driver import SPI
from driver import GPIO
from kalman import KalmanAngle
from mpu6050 import MPU6050


# [components/py_engine/framework/mpu6050.py]
# (https://gitee.com/alios-things/AliOS-Things/blob/master/components/py_engine/framework/mpu6050.py)

# Globals
last_read_time = 0.0   

# These are the filtered angles
last_x_angle = 0.0          
last_y_angle = 0.0
last_z_angle = 0.0  

# Calibrated measurements to offset some bias or error in the readings.
calib_x_accel = 0.0 
calib_y_accel = 0.0 
calib_z_accel = 0.0 
calib_x_gyro  = 0.0 
calib_y_gyro  = 0.0 
calib_z_gyro  = 0.0 

def set_last_read_angles(time, x, y):
    global last_read_time, last_x_angle, last_y_angle
    last_read_time = time
    last_x_angle = x 
    last_y_angle = y
    #last_z_angle = z

# accelerometer data can't be used to calculate 'yaw' angles or rotation around z axis.
def acc_angle(Ax, Ay, Az):
    radToDeg = 180/3.14159
    ax_angle = math.atan(Ay/math.sqrt(math.pow(Ax,2) + math.pow(Az, 2)))*radToDeg
    ay_angle = math.atan((-1*Ax)/math.sqrt(math.pow(Ay,2) + math.pow(Az, 2)))*radToDeg
    return (ax_angle, ay_angle)

def gyr_angle(Gx, Gy, Gz, dt):
    gx_angle = Gx*dt + last_x_angle
    gy_angle = Gy*dt + last_y_angle
    gz_angle = Gz*dt + last_z_angle
    return (gx_angle, gy_angle, gz_angle)

  # A complementary filter to determine the change in angle by combining accelerometer and gyro values. Alpha depends on the sampling rate...
def c_filtered_angle(ax_angle, ay_angle, gx_angle, gy_angle):
    alpha = 0.90
    c_angle_x = alpha*gx_angle + (1.0 - alpha)*ax_angle
    c_angle_y = alpha*gy_angle + (1.0 - alpha)*ay_angle
    return (c_angle_x, c_angle_y)

 # Kalman filter to determine the change in angle by combining accelerometer and gyro values. 
def k_filtered_angle(ax_angle, ay_angle, Gx, Gy, dt):
    k_angle_x = kalmanX.getAngle(ax_angle, Gx, dt)
    k_angle_y = kalmanY.getAngle(ay_angle, Gy, dt)
    return (k_angle_x, k_angle_y)

def read_MPU6050():
    ac = []
    gy = []
    ac = mpu6050Dev.get_Accelerometer()
    gy = mpu6050Dev.get_Gyroscope()
    return (ac[0], ac[1], ac[2], gy[0], gy[1], gy[2])

def calibrate_sensors():
    x_accel = 0
    y_accel = 0
    z_accel = 0
    x_gyro  = 0
    y_gyro  = 0
    z_gyro = 0
    ac = []
    gy = []
    #print("Starting Calibration")

    #Discard the first set of values read from the IMU
    ac = mpu6050Dev.get_Accelerometer()
    gy = mpu6050Dev.get_Gyroscope()

    # Read and average the raw values from the IMU
    for int in range(100): 
        ac = mpu6050Dev.get_Accelerometer()
        gy = mpu6050Dev.get_Gyroscope()
        x_accel += ac[0]
        y_accel += ac[1]
        z_accel += ac[2]
        x_gyro  += gy[0]
        y_gyro  += gy[1]
        z_gyro  += gy[2]
        utime.sleep_ms(10)
    
    x_accel /= 10
    y_accel /= 10
    z_accel /= 10
    x_gyro  /= 10
    y_gyro  /= 10
    z_gyro  /= 10

    # Store the raw calibration values globally
    calib_x_accel = x_accel
    calib_y_accel = y_accel
    calib_z_accel = z_accel
    calib_x_gyro  = x_gyro
    calib_y_gyro  = y_gyro
    calib_z_gyro  = z_gyro

    print("---=== Finishing Calibration ===---")


mpu6050Dev = MPU6050()
mpu6050Dev.open("mpu6050")
mpu6050Dev.init()
print("mpu6050 init finished")

calibrate_sensors()

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

spi0 = SPI()
spi0.open("oled_spi")

gpio_dc = GPIO()
gpio_dc.open("oled_dc")

gpio_res = GPIO()
gpio_res.open("oled_res")

display = sh1106.SH1106_SPI(width=132, height=64, spi=spi0, dc = gpio_dc, res = gpio_res)
# display.init_display()
ac = []
gy = []
while (True):
    
    t_now = utime.ticks_ms()
    dt = (t_now - last_read_time)/1000.0
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = read_MPU6050()

    #Full scale range +/- 250 degree/C as per sensitivity scale factor. The is linear acceleration in each of the 3 directions ins g's
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    
    # This is angular velocity in each of the 3 directions 
    Gx = (gyro_x - calib_x_gyro)/131.0
    Gy = (gyro_y - calib_y_gyro)/131.0
    Gz = (gyro_z - calib_z_gyro)/131.0

    acc_angles = acc_angle(Ax, Ay, Az) # Calculate angle of inclination or tilt for the x and y axes with acquired acceleration vectors
    gyr_angles = gyr_angle(Gx, Gy, Gz, dt) # Calculate angle of inclination or tilt for x,y and z axes with angular rates and dt 
    (c_angle_x, c_angle_y) = c_filtered_angle(acc_angles[0], acc_angles[1], gyr_angles[0], gyr_angles[1]) # filtered tilt angle i.e. what we're after
    (k_angle_x, k_angle_y) = k_filtered_angle(acc_angles[0], acc_angles[1], Gx, Gy, dt)

    set_last_read_angles(t_now, c_angle_x, c_angle_y)
    # xstr = "anglex : " + str(c_angle_x)
    # ystr = "angely : " + str(c_angle_y)
    ystr = ("{:.1f}".format(c_angle_y))
    
    display.fill(0)

    # display.text(xstr, 20, 20, 2)
    display.text(ystr, 20, 40, 32)
    # ac = mpu6050Dev.get_Accelerometer()
    # print("mpu6050 acc is: ", ac[0], ac[1], ac[2])
    # ac0_str = "ac[0] : " + str(ac[0])
    # ac1_str = "ac[1] : " + str(ac[1])
    # ac2_str = "ac[2] : " + str(ac[2])
    # display.text(ac0_str, 20, 0, 2)
    # display.text(ac0_str, 20, 20, 2)
    # display.text(ac0_str, 20, 40, 2)
    
    #display.fill_circle(50, 30, 20, 0xAF)
    #display.draw_circle(90, 30, 20, 2, 0xAF)
    # display.draw_circle(66, 32, 10, 1, 1)
    # display.fill_circle(int(66 - ac[0] / 250), int(32 + ac[1] / 500), 8, 1)
    display.show()
# display.test()
mpu6050Dev.close()
print("test mpu6050 success!")
