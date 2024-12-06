import network
import espnow
import ubinascii
from gpio_lcd import GpioLcd
from time import sleep
from machine import Pin

wlan_sta = network.WLAN(network.STA_IF)
wlan_sta.active(True)

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25), d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22), num_lines=4, num_columns=20)




wlan_mac = wlan_sta.config('mac')
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

### If sending
e = espnow.ESPNow()
e.active(True)
peer_gyro = b'\xd4\x8a\xfch\x9f\x84'
e.add_peer(peer_gyro)

while True:
    host, msg = e.recv()
    if msg:
        print(host, msg)
        msg_decode = msg.decode('ascii')
        current_data = str(msg_decode)
        if msg == b'end':
            break
    if current_data != None:
        lcd.clear()
        lcd.move_to (1,0)
        lcd.putstr(current_data)

    

### If recieving