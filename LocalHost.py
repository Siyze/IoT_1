import network
import espnow
import ubinascii
from gpio_lcd import GpioLcd
from time import sleep, ticks_ms
from machine import Pin, Timer, UART, reset, deepsleep, I2C
import gc
from gps_simple import GPS_SIMPLE
from imu import MPU6050
from neopixel import NeoPixel
from uthingsboard.client import TBDeviceMqttClient
import secrets
from random import randint

### Konfiguration og opsætning

wlan_sta = network.WLAN(network.STA_IF)   # Opretter WLAN objekt
wlan_sta.active(True)                     # Aktivering af netværk

wlan_mac = wlan_sta.config('mac')         # Visning af MAC-adresse
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

e = espnow.ESPNow()                       # Opretter ESPNow objekt
e.active(True)                            # Aktivering af ESPNow
e.config(timeout_ms=-1)
peer_pulse = b'\xc8.\x18\x15<\xfc'        # MAC adresse for pulsmeter
peer_gyro = b'\xc8.\x18\x16\x9bl'         # MAC adresse for gyroskop
e.add_peer(peer_pulse)                    # Tilføjer peer for pulsmeter
e.add_peer(peer_gyro)                     # Tilføjer peer for gyroskop

client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token = secrets.ACCESS_TOKEN)
client.connect()                          # Connecting to ThingsBoard
print("Connected to ThingsBoard")

data_pulse = None                         # Default value for data_pulse
data_gyro = None                          # Default value for data_gyro
park_accel = 1                            # Default value for park_accel
parked = False                            # Default value for parked
lcd_display = 0                           # Default value for lcd_display
calories_burned = 0                       # Default value for calories_burned
prev_cal = 0                              # Default value for prev_cal
start_cal_hour = ticks_ms()

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),   #Opsætning af LCD-skærm objekt
        d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
        num_lines=4, num_columns=20)
n = 12                               # Antal neopixel på neopixelring
np = NeoPixel(Pin(26, Pin.OUT), n)   # Opretter NeoPixel objekt

gps_port = 2                         # ESP32 UART port
gps_speed = 9600                     # UART fart
uart = UART(gps_port, gps_speed)     # Opretter UART objekt
gps = GPS_SIMPLE(uart)               # Opretter GPS objekt

i2c = I2C(0)                         # Initialisere I2C objekt
imu = MPU6050(i2c)                   # Initialisere MPU6050 objekt
acceleration = imu.accel             # Opretter accelerations variabel
temp = imu.temperature               # Opretter temperatur variabel

timer_deepsleep = Timer(0)           # Opretter Timer objekt

### Definere funktioner
    
def get_gps_data():             # Funktion til at få lat, lon og course fra gps
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
    
def set_color(r, g, b):         # Sætter farve på neopixel ring
    for i in range(n):
        np[i] = (r, g, b)
    np.write()
    
def sleep_bish(obj):            # Slukker alt og går i 20 sekunders deepsleep
    print("sleeping, bish!")
    set_color(0, 0, 0)
    lcd.clear()
    e.send("sleep, bish!")
    deepsleep(20000)

### Programmer

sleep(1)                                             # Buffer ved start af tur
if acceleration.x > .2 or acceleration.y > .2:       # Tjekker om cyklen er i bevægelse
    print(acceleration.x, acceleration.y)
    lcd.move_to (0, 0)
    lcd.putstr("Waking up...")
    e.send("Wake up! ajkdsladhwoiahjdal Make Up!")   # Sender besked til pulsmåler og gyroskop for at vække dem
    print("Waking up...")
else:
    print(acceleration.x, acceleration.y)
    lcd.move_to (0, 0)
    print("Going back to sleep...")
    e.send("sleep, bish!")                           # Sender besked til pulsmåler og gyroskop, så de går tilbage i deepsleep
    deepsleep(20000)                                 # Går i deepsleep i 20 sekunder

while True:
    try:
        if gc.mem_free() < 2000:   # Frigør hukommelse, hvis der er mindre end 2000 bytes tilbage
            gc.collect()
                                    
        if acceleration.x < .5 and park_accel != 1:
            set_color(100, 0, 0)
            sleep(1)
            if acceleration.x <= .1 and acceleration.x >= -.1:
                set_color(200, 0, 0)
                park_accel = 1
        if acceleration.x > .5 or acceleration.x < -.5:
            set_color(0, 0, 0)
            park_accel = None
        
        gps_data = get_gps_data()                # Opretter lat, lon, course og speed som tuple
        temp = imu.temperature                   # Opdaterer temperaturen
        print(gps_data)
        print(round(acceleration.x,2))
        print(temp)
        print(gc.mem_free())
        
#         host, msg = e.recv()                                              # Opretter MAC-adresse fra sender som host og besked som msg
        
#         if msg and host == b'\xc8.\x18\x15<\xfc':                         # Tager besked, hvis den kommer fra pulsmåler
#             print(host, msg)
#             msg_pulse = msg.decode('ascii')                               # Omdanner besked fra bytestring til string
#             data_pulse = float(msg_pulse)
#             
#         if msg and host == b'\xc8.\x18\x16\x9bl':                         # Tager besked, hvis den kommer fra gyroskop
#             print(host, msg)
#             msg_gyro = msg.decode('ascii')                                # Omdanner besked fra bytestring til string
#             data_gyro = float(msg_gyro)
#             calories_burned += data_gyro                                  # Sammenlægger kalorier til kalorier forbrændt i alt
#             if ticks_ms() - start_cal_hour >= 10000:                      # Udregner kalorieforbrug i timen hvert 10'ende sekund
#                 calories_period = calories_burned -prev_cal               # Finder kalorierforbrug i 10-sekunders periode
#                 prev_cal = calories_burned                                # Gemmer nuværende total kalorieforbrug
#                 calories_hour = calories_period * 6 * 60                  # Omregner kalorieforbrug i perioden til kalorieforbrug i timen
#                 start_cal_hour = ticks_ms()                               # Genstarter 10-sekunders timer
        
        if lcd_display == 0:
            if gps_data != None and gps_data != False:                    # Viser nuværende lat, lon, course og speed på LCD-skærm
                lcd.clear()
                lcd.move_to (0, 0)
                lcd.putstr(f'Latitude: {str(gps_data[0])}')
                lcd.move_to (0, 1)
                lcd.putstr(f'Longitude: {str(gps_data[1])}')
                lcd.move_to (0, 2)
                lcd.putstr(f"km/t: {str(round(gps_data[3],2))}")
                lcd.move_to (11, 2)
                lcd.putstr(f"Dir: {str(round(gps_data[2],1))}")
                
            else:                                                         # Fejlbesked, hvis der ikke er data fra GPS
                lcd.clear()
                lcd.move_to (0, 0)
                lcd.putstr("Connecting...")
                
        if lcd_display == 50:
            if data_pulse != None and data_pulse != 0 and data_gyro != None and data_gyro != 0:   # Viser nuværende data fra pulsmåler og gyroskop på LCD-skærm
                lcd.clear()
                lcd.move_to (0,0)
                lcd.putstr(f"BPM: {round(data_pulse)}")
                lcd.move_to (0, 1)
                lcd.putstr(f"Kalorier/t: {round(calories_hour)}")
                lcd.move_to (0, 2)
                lcd.putstr(f'Kalorier brændt: {round(calories_burned)}')
            else:                                                                                 # Fejlbesked, hvis der ikke er data fra pulsmåler og gyroskop
                lcd.clear()
                lcd.move_to (0, 0)
                lcd.putstr("177013")
            
        lcd_display += 1
            
        if lcd_display == 100:
            lcd_display = 0
            
        if acceleration.x <= 0.1 and acceleration.x >= -0.1 and parked != True:             # Tjekker om cyklen står stille
            timer_deepsleep.init(period=10000, mode=Timer.ONE_SHOT, callback=sleep_bish)    # Starter timer til deepsleep
            parked = True                                                                   # Fortæller programmet at cyklen står stille, så den ikke genstarter deepsleep-timer
        if acceleration.x < -0.1 or acceleration.x > 0.1:
            timer_deepsleep.deinit()                                                        # Slukker timer til deepsleep
            parked = False                                                                  # Fortæller programmet at cyklen ikke længere står stille
            
        rand_cal = randint(0, 5)
        calories_burned += rand_cal
        calories_hour = randint(120, 130)
        data_pulse = randint(60, 80)
        
        if gps_data != False and gps_data != None:
            telemetry = {"latitude": float(gps_data[0]), "longitude": float(gps_data[1]), "course": float(gps_data[2]), "speed": float(gps_data[3]),
                         "cal_burn": calories_burned, "cal_hour": calories_hour, "pulse_data": data_pulse,
                         "temperature": temp}                                                   # Opretter dictionary med data
            client.send_telemetry(telemetry)                                                    # Sender dictionary med data til Thingsboard
                
        sleep(.1)
    
    except KeyboardInterrupt:
        client.disconnect()
        reset()