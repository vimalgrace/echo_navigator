import smbus
import time
import math
 
# QMC5883L register addresses
ADDRESS = 0x0D


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
    bus.write_byte_data(ADDRESS, RegCTRL1, Mode_Continuous)  # Continuous measurement mode
 
def read_raw_data(addr):
    # Read raw 16-bit value
    low = bus.read_byte_data(ADDRESS, addr)
    high = bus.read_byte_data(ADDRESS, addr+1)
    
    # Combine them to get a 16-bit value
    value = (high << 8) + low
    if value > 32768:  # Adjust for 2's complement
        value = value - 65536
    return value
 
def compute_heading(x, y):
    # Calculate heading in radians
    heading_rad = math.atan2(y, x)
    
    # Adjust for declination angle (e.g. 0.22 for ~13 degrees)
    declination_angle = 1.28 # 0.22
    heading_rad += declination_angle
    
    # Correct for when signs are reversed.
    if heading_rad < 0:
        heading_rad += 2 * math.pi
 
    # Check for wrap due to addition of declination.
    if heading_rad > 2 * math.pi:
        heading_rad -= 2 * math.pi
 
    # Convert radians to degrees for readability.
    heading_deg = heading_rad * (180.0 / math.pi)
    
    return heading_deg
 
def main():
    setup()
    
    while True:
        x = read_raw_data(RegXLo)
        y = read_raw_data(RegYLo)
        z = read_raw_data(RegZLo)
        
        heading = compute_heading(x, y)
        
        print(f"X: {x} uT, Y: {y} uT, Z: {z} uT, Heading: {heading:.2f}°")
        
        time.sleep(0.5)
 
if __name__ == "__main__":
    main()