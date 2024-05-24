import py_qmc5883l
import time
sensor = py_qmc5883l.QMC5883L()


sensor.mode_continuous()

sensor.declination = -1.48

sensor.calibration = [[ 1.00891199e+00,-1.66022293e-02 , 2.15077417e+03],
                        [-1.66022293e-02, 1.03092846e+00, 2.42391646e+03],
                        [ 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

while True:
    print(type(sensor.get_bearing()))
    time.sleep(0.1)