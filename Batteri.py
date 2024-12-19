from machine import I2C
from ina219_lib import INA219
from time import sleep

i2c_port = 0

ina219_i2c_addr = 0x40

i2c = I2C(i2c_port)

ina219 = INA219(i2c, ina219_i2c_addr)


print("\nINA219 current measurement program\n")

while True:
    current = ina219.get_current()
    print(str(current) +"mA")
    
    sleep(1)
    
    