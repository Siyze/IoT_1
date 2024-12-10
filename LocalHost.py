import network
import espnow
import ubinascii
from gpio_lcd import GpioLcd
from time import sleep, ticks_ms, ticks_diff
from machine import Pin, WDT, Timer, UART, reset, deepsleep, I2C
import gc
from gps_simple import GPS_SIMPLE
import esp32
from imu import MPU6050
import sys
from neopixel import NeoPixel

### Konfiguration og opsætning

wlan_sta = network.WLAN(network.STA_IF)   # Opretter WLAN objekt
wlan_sta.active(True)                     # Aktivering af netværk

wlan_mac = wlan_sta.config('mac')         # Visning af MAC-adresse
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

e = espnow.ESPNow()                       # Opretter ESPNow objekt
e.active(True)                            # Aktivering af ESPNow
peer_pulse = b'\xd4\x8a\xfch\x9f\x84'     # Tilføjer peer til pulsmeter
e.add_peer(peer_pulse)
current_data = None                       # Default value for current_data skal være None
park_accel = 1                            # Default value for park_accel

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),   #Opsætning af LCD-skærm objekt
        d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
        num_lines=4, num_columns=20)
n = 12                               # Antal neopixel på neopixelring
np = NeoPixel(Pin(26, Pin.OUT), n)   # Opretter NeoPixel objekt

gps_port = 2                       # ESP32 UART port
gps_speed = 9600                   # UART fart
uart = UART(gps_port, gps_speed)   # Opretter UART objekt
gps = GPS_SIMPLE(uart)             # Opretter GPS objekt

i2c = I2C(0)                       # Initialisere I2C objekt
imu = MPU6050(i2c)                 # Initialisere MPU6050 objekt
acceleration = imu.accel           # Opretter accelerations variabel

timer_deepsleep = Timer(0)         # Opretter Timer objekt

### Definere funktioner
    
def get_gps_data():     # Funktion til at få lat, lon og course fra gps
    lat = lon = course = None
    
    if gps.receive_nmea_data():
        if gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_course() != -999.0 and gps.get_validity() == "A":
            lat = gps.get_latitude()
            lon = gps.get_longitude()
            course = gps.get_course()
            speed = gps.get_speed()
            return lat, lon, course, speed
        else:
            return False
    else:
        return False
    
def set_color(r, g, b):
    for i in range(n):
        np[i] = (r, g, b)
    np.write()
    
def sleep_bish(obj):
    print("sleeping, bish!")
    set_color(0, 0, 0)
    lcd.clear()
    deepsleep(20000)

### Programmer

sleep(1)
if acceleration.x > .2 or acceleration.y > .2:
    print(acceleration.x, acceleration.y)
    print("waking")
else:
    print(acceleration.x, acceleration.y)
    print("going back to sleep")
    deepsleep(20000)
    
lcd_display = 0

while True:
    try:
        if gc.mem_free() < 2000:   # Frigør hukommelse, hvis der er mindre end 2000 bytes tilbage
            gc.collect()
            
        if acceleration.x < .5 and park_accel != 1:
            set_color(100, 0, 0)
            sleep(1)
            if acceleration.x <= .1 and acceleration.x >= -.1:
                set_color(255, 0, 0)
                park_accel = 1
        if acceleration.x > .5 or acceleration.x < -.5:
            set_color(0, 0, 0)
            park_accel = None
        
        gps_data = get_gps_data()   # Opretter lat, lon og course som tuple
        print(gps_data)
        print(round(acceleration.x,2))
        
        if lcd_display == 0:
            if gps_data != None and gps_data != False:   # Viser nuværende lat, lon, course og speed på LCD-skærm
                lcd.clear()
                lcd.move_to (0, 0)
                lcd.putstr(f"Lat: {str(gps_data[0])}")
                lcd.move_to (0, 1)
                lcd.putstr(f"Lon: {str(gps_data[1])}")
                lcd.move_to (0, 2)
                lcd.putstr(f"km/t: {str(round(gps_data[3],2))}")
                lcd.move_to (11, 2)
                lcd.putstr(f"Dir: {str(round(gps_data[2],1))}")
            else:
                lcd.clear()
                lcd.move_to (0, 0)
                lcd.putstr("GPS Connecting...")
                
        if lcd_display == 50:
            lcd.clear()
            lcd.move_to (0, 0)
            lcd.putstr("TEST")
            
        lcd_display += 1
            
        if lcd_display == 100:
            lcd_display = 0

#         host, msg = e.recv()                   # Opretter MAC-adresse fra sender som host og besked som msg
#         if msg:                                # Printer besked i shell og gemmer besked som string
#             print(host, msg)
#             msg_decode = msg.decode('ascii')
#             current_data = str(msg_decode)
#             if msg == b'end':                  # Slutter programmet, hvis slutbesked modtages
#                 break
            
        if current_data != None:   # Viser nuværende data på LCD-skærm
            lcd.clear()
            lcd.move_to (1,0)
            lcd.putstr(current_data)
            
        if acceleration.x <= 0.1 and acceleration.x >= -0.1 and parked != True:   # Tjekker om cyklen står stille
            timer_deepsleep.init(period=180000, mode=Timer.ONE_SHOT, callback=sleep_bish)   # Starter timer til deepsleep
            parked = True
        if acceleration.x < -0.1 or acceleration.x > 0.1:
            timer_deepsleep.deinit()
            parked = False
        
        sleep(.1)
    
    except KeyboardInterrupt:
        reset()