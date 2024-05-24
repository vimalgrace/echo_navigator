import smbus
import time
import math

# QMC5883L address on i2c bus
QMC5883L_ADDR = 0x0D

# QMC5883L register addresses
QMC5883L_XOUT_LSB = 0x00
QMC5883L_XOUT_MSB = 0x01
QMC5883L_YOUT_LSB = 0x02
QMC5883L_YOUT_MSB = 0x03
QMC5883L_ZOUT_LSB = 0x04
QMC5883L_ZOUT_MSB = 0x05
QMC5883L_STATUS = 0x06
QMC5883L_CONTROL_1 = 0x07
QMC5883L_CONTROL_2 = 0x08
QMC5883L_SET_RESET = 0x0B

# Initialise the i2c bus
bus = smbus.SMBus(1)

# Read from QMC5883L register
def read_register(address):
    return bus.read_byte_data(QMC5883L_ADDR, address)

# Read 2 bytes from QMC5883L register
def read_2_byte_register(address):
    return bus.read_word_data(QMC5883L_ADDR, address)

# Write to QMC5883L register
def write_register(address, data):
    bus.write_byte_data(QMC5883L_ADDR, address, data)

# Write to QMC5883L 2 byte register
def write_2_byte_register(address, data):
    bus.write_word_data(QMC5883L_ADDR, address, data)

# Read data from QMC5883L
def read_data():
    x_lsb = read_register(QMC5883L_XOUT_LSB)
    x_msb = read_register(QMC5883L_XOUT_MSB)
    y_lsb = read_register(QMC5883L_YOUT_LSB)
    y_msb = read_register(QMC5883L_YOUT_MSB)
    z_lsb = read_register(QMC5883L_ZOUT_LSB)
    z_msb = read_register(QMC5883L_ZOUT_MSB)

    x_out = x_lsb | (x_msb << 8)
    y_out = y_lsb | (y_msb << 8)
    z_out = z_lsb | (z_msb << 8)

    return x_out, y_out, z_out

# Read QMC5883L status
def read_status():
    return read_register(QMC5883L_STATUS)

# Set QMC5883L output data rate
def set_output_data_rate(rate):
    if rate not in [0, 1, 2, 3, 4, 5, 6, 7]:
        print("Invalid data rate")
        return

    data = read_register(QMC5883L_CONTROL_1)
    data &= 0x78  # Clear ODR bits
    data |= rate  # Set ODR bits
    write_register(QMC5883L_CONTROL_1, data)

# Set QMC5883L measurement mode
def set_measurement_mode(mode):
    if mode not in [0, 1, 2]:
        print("Invalid measurement mode")
        return

    data = read_register(QMC5883L_CONTROL_1)
    data &= 0x8F  # Clear M bits
    data |= (mode << 4)  # Set M bits
    write_register(QMC5883L_CONTROL_1, data)

# Set QMC5883L gain
def set_gain(gain):
    if gain not in [0, 1, 2, 3]:
        print("Invalid gain")
        return

    data = read_register(QMC5883L_CONTROL_2)
    data &= 0xF0  # Clear gain bits
    data |= gain  # Set gain bits
    write_register(QMC5883L_CONTROL_2, data)

# Initialise QMC5883L
def init_qmc5883l():
    # Reset QMC5883L
    write_register(QMC5883L_SET_RESET, 0x01)
    time.sleep(0.1)

    # Set ODR to 100 Hz
    set_output_data_rate(100)

    # Set measurement mode to continuous
    set_measurement_mode(1)

    # Set gain to 1.3
    set_gain(1.3)

# Main function
def main():
    init_qmc5883l()

    while True:
        x, y, z = read_data()
        status = read_status()

        # Calculate the angles between the X-axis and the Z-axis, and between the X-axis and the X-Y plane
        theta = math.atan2(y, math.sqrt(y**2 + z**2)) * 180 / math.pi
        phi = math.atan2(y, z) * 180 / math.pi

        # Calculate the heading as the sum of the angles
        heading = (theta + phi) % 360

        print("X: {:04x} Y: {:04x} Z: {:04x} Heading: {:.2f}° Status: {:02x}".format(x, y, z, heading, status))

        time.sleep(0.1)

if __name__ == "__main__":
    main()