import smbus
import time
import math
 
# QMC5883L register addresses
QMC5883L_ADDR = 0x0D


############## Register Location
RegCTRL1 = 0x09 # Control Register--> MSB(OSR:2,RNG:2,ODR:2,MODE:2)LSB
RegCTRL2 = 0x0A # Control Register2--> MSB(Soft_RS:1,Rol_PNT:1,none:5,INT_ENB:1)LSB
RegFBR   = 0x0B # SET/RESET Period Register--> MSB(FBR:8)LSB
RegXLo   = 0x00
RegXHi   = 0x01
RegYLo   = 0x02
RegYHi   = 0x03
RegZLo   = 0x04
RegZHi   = 0x05

############## Cpntrol Register Value 
Mode_Standby    = 0b00000000
Mode_Continuous = 0b00000001
ODR_10Hz        = 0b00000000
ODR_50Hz        = 0b00000100
ODR_100Hz       = 0b00001000
ODR_200Hz       = 0b00001100
RNG_2G          = 0b00000000
RNG_8G          = 0b00010000
OSR_512         = 0b00000000
OSR_256         = 0b01000000
OSR_128         = 0b10000000
OSR_64          = 0b11000000

declinationAngle = 1.28 # 0.0404  
pi          = 3.14159265359

bus = smbus.SMBus(1)
 
def setup():
    # bus.write_byte_data(ADDRESS, CONFIG_A, 0x00)  # Set to 8 samples @ 15Hz
    # bus.write_byte_data(ADDRESS, CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
    bus.write_byte_data(QMC5883L_ADDR, RegCTRL1, Mode_Continuous | ODR_200Hz | RNG_8G | OSR_512)  # Continuous measurement mode
 

#     # Read raw 16-bit value
# low = bus.read_byte_data(ADDRESS, addr)
# high = bus.read_byte_data(ADDRESS, addr+1)

########### Read

def main():

    while True:
        buffer = bus.read_byte_data(QMC5883L_ADDR,RegXLo)
        xLo = bus.read_byte_data(QMC5883L_ADDR,RegXLo)
        xHi = bus.read_byte_data(QMC5883L_ADDR,RegXHi) <<8
        yLo = bus.read_byte_data(QMC5883L_ADDR,RegYLo)
        yHi = bus.read_byte_data(QMC5883L_ADDR,RegYHi) <<8
        zLo = bus.read_byte_data(QMC5883L_ADDR,RegZLo)
        zHi = bus.read_byte_data(QMC5883L_ADDR,RegZHi) <<8
        x = xHi + xLo
        y = yHi + yLo
        z = zHi + zLo

        ########### Convert
        heading = math.atan2(y, x)
        heading = heading + declinationAngle
        #Due to declination check for >360 degree
        if(heading > 2*pi):
            heading = heading - 2*pi
        #check for sign
        if(heading < 0):
            heading = heading + 2*pi

        #convert into angle
        headingAngle = (heading * 180/pi)

        print("3-axis : x={}/{},{} y={}/{},{} z={}/{},{}".format(x,xHi,xLo,y,yHi,yLo,z,zHi,zLo))
        print ("Heading Angle = {}°".format(headingAngle))

        time.sleep(0.5)

if __name__ == "__main__":
    setup()
    main()
    