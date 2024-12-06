import network
import espnow
import ubinascii
from machine import I2C
from eeprom_24xx64 import EEPROM_24xx64
import gc
from time import sleep

i2c = I2C(0)

eeprom = EEPROM_24xx64(i2c, 0x50)

wlan_sta = network.WLAN(network.STA_IF)
wlan_sta.active(True)

wlan_mac = wlan_sta.config('mac')
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

e = espnow.ESPNow()
e.active(True)
ronin_peer = b'\xc8.\x18\x16\x9bl'
e.add_peer(ronin_peer)

gc.enable

while True:
    e.send("Hello World")
    sleep(5)
    e.send("Goodbye World")
    sleep(5)
        

        
        