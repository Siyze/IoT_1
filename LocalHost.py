import network
import espnow
import ubinascii
from gpio_lcd import GpioLcd
from time import sleep
from machine import Pin, WDT, Timer, UART, reset
import gc
from gps_simple import GPS_SIMPLE

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
current_lat = 0                           # Default value for current_lat
current_lon = 0                           # Default value for current_lon

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),   #Opsætning af LCD-skærm objekt
        d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
        num_lines=4, num_columns=20)

gps_port = 2                       # ESP32 UART port
gps_speed = 9600                   # UART fart
uart = UART(gps_port, gps_speed)   # Opretter UART objekt
gps = GPS_SIMPLE(uart)             # Opretter GPS objekt

wdt = WDT(timeout=180000)   #Opretter Watchdog objekt
timer_wdt = Timer(0)        #Opretter Timer objekt

### Definere funktioner

def reset_watchdog():   #Funktion til reset af watchdog timer
    wdt.feed()
    
def get_gps_data():
    lat = lon = course = None
    
    if gps.receive_nmea_data():
        if gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_course() != -999.0 and gps.get_validity() == "A":
            lat = str(gps.get_latitude())
            lon = str(gps.get_longitude())
            course = str(gps.get_course())
            return lat, lon, course
        else:
            return False
    else:
        return False

### Programmer

while True:
    try:
        gps_data = get_gps_data()   # Opretter lat, lon og course som tuple
        print(gps_data)
        
        if gps_data != None and gps_data != False:   # Viser nuværende lat, lon og course på LCD-skærm
            lcd.clear()
            lcd.move_to (0, 0)
            lcd.putstr(f"Lat: {str(gps_data[0])}")
            lcd.move_to (0, 1)
            lcd.putstr(f"Lon: {str(gps_data[1])}")
            lcd.move_to (0, 2)
            lcd.putstr(f"Course: {str(gps_data[2])}")

#         host, msg = e.recv()
#         if msg:                                #Printer besked i shell og gemmer besked som string
#             print(host, msg)
#             msg_decode = msg.decode('ascii')
#             current_data = str(msg_decode)
#             if msg == b'end':                  #Slutter programmet, hvis slutbesked modtages
#                 break
            
        if current_data != None:   #Viser nuværende data på LCD-skærm
            lcd.clear()
            lcd.move_to (1,0)
            lcd.putstr(current_data)
                        
        if gc.mem_free() < 2000:   #Frigør hukommelse, hvis der er mindre end 2000 bytes tilbage
            gc.collect()
        
        if gps_data != False:
            if gps_data[0] != current_lat or gps_data[1] != current_lon:
                print("Feeding the watchdog")
                reset_watchdog()
                current_lat = gps_data[0]
                current_lon = gps_data[1]
        
        sleep(5)
    
    except KeyboardInterrupt:
        reset()