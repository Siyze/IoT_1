import network
import espnow
import ubinascii
from gpio_lcd import GpioLcd
from time import sleep, ticks_ms
from machine import Pin, Timer, UART, reset, deepsleep, I2C, PWM, SPI
import gc
from gps_simple import GPS_SIMPLE
from imu import MPU6050
from neopixel import NeoPixel
from uthingsboard.client import TBDeviceMqttClient
import secrets
from ina219_lib import INA219
from sys import exit
from portExp_MCP23S08 import PortExp_MCP23S08

### Konfiguration og opsætning

wlan_sta = network.WLAN(network.STA_IF)   # Opretter WLAN objekt
wlan_sta.active(True)                     # Aktivering af netværk
wlan_sta.config(pm=wlan_sta.PM_NONE)

wlan_mac = wlan_sta.config('mac')         # Visning af MAC-adresse
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

e = espnow.ESPNow()                       # Opretter ESPNow objekt
e.active(True)                            # Aktivering af ESPNow
e.config(timeout_ms=-1)
peer_pulse = b'\xd4\x8a\xfch\x19\x0c'     # MAC adresse for pulsmeter
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
calories_hour = 0                         # Default value for calories_hour
prev_cal = 0                              # Default value for prev_cal
battery_capacity = 1800                   # mA i batteriet
alarm = False                             # Default value for alarm
start_cal_hour = ticks_ms()

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),   #Opsætning af LCD-skærm objekt
        d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
        num_lines=4, num_columns=20)

n = 12                                  # Antal neopixel på neopixelring
np = NeoPixel(Pin(26, Pin.OUT), n)      # Opretter NeoPixel objekt

gps_port = 2                            # ESP32 UART port
gps_speed = 9600                        # UART fart
uart = UART(gps_port, gps_speed)        # Opretter UART objekt
gps = GPS_SIMPLE(uart)                  # Opretter GPS objekt

i2c = I2C(0)                            # Initialisere I2C objekt
imu = MPU6050(i2c)                      # Initialisere MPU6050 objekt
acceleration = imu.accel                # Opretter accelerations variabel
temp = imu.temperature                  # Opretter temperatur variabel

ina_pin = i2c                           # INA219 bruger I2C som pin
ina_i2c_addr = 0x40                     # INA219 I2C adresse
ina219 = INA219(i2c, ina_i2c_addr)      # Opretter INA219 objekt
ina219.set_calibration_16V_400mA()      # Kalibrere INA219 til at være mere følsom
current = ina219.get_current()          # Opretter current variabel

buzzer = PWM(Pin(14, Pin.OUT), duty=0)  # Opretter buzzer objekt

timer_deepsleep = Timer(0)              # Opretter Timer objekt

### Definere funktioner
    
def get_gps_data():                     # Funktion til at få lat, lon og course fra gps
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
    
def set_color(r, g, b):                 # Sætter farve på neopixel ring
    for i in range(n):
        np[i] = (r, g, b)
    np.write()
    
def sleep_bish(obj):                    # Slukker alt og går i 20 sekunders deepsleep
    print("sleeping, bish!")
    set_color(0, 0, 0)
    lcd.clear()
    e.send("sleep, bish!")
    deepsleep(20000)
    
def handler(req_id, method, params):    # Skifter alarm til eller fra, hvis RPC kald fås fra Thingsboard
    global alarm
    try:
        print(f'Response {req_id}: {method}, params {params}')
        if method == "toggle_alarm":
            if params == True:
                alarm = True
                timer_deepsleep.deinit()
                lcd.clear()
                set_color(0, 0, 0)
                e.send("sleep, bish")
            else:
                alarm = False
                e.send("Wake up! ajkdsladhwoiahjdal Make Up!")
    except TypeError as e:
        print(e)
    

### Programmer

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
            
        client.set_server_side_rpc_request_handler(handler)
        client.check_msg()
        
        if alarm == False:                                # Kører programmet, hvis alarmen ikke er slået til
            gps_data = get_gps_data()                     # Opretter lat, lon, course og speed som tuple
            temp = imu.temperature                        # Opdaterer temperaturen
            current += ina219.get_current()
            battery_percentage = 100 - (current / battery_capacity) * 100
            print(gc.mem_free())
            print(round(acceleration.x,2))
                
            host, msg = e.recv()                                              # Opretter MAC-adresse fra sender som host og besked som msg
            
            if msg and host == b'\xc8.\x18\x15<\xfc':                         # Tager besked, hvis den kommer fra pulsmåler
                msg_pulse = msg.decode('ascii')                               # Omdanner besked fra bytestring til string
                data_pulse = float(msg_pulse)
                
            if msg and host == b'\xc8.\x18\x16\x9bl':                         # Tager besked, hvis den kommer fra gyroskop
                msg_gyro = msg.decode('ascii')                                # Omdanner besked fra bytestring til string
                data_gyro = float(msg_gyro)
                calories_burned += data_gyro                                  # Sammenlægger kalorier til kalorier forbrændt i alt
                if ticks_ms() - start_cal_hour >= 10000:                      # Udregner kalorieforbrug i timen hvert 10'ende sekund
                    calories_period = calories_burned - prev_cal              # Finder kalorierforbrug i 10-sekunders periode
                    prev_cal = calories_burned                                # Gemmer nuværende total kalorieforbrug
                    calories_hour = calories_period * 6 * 60                  # Omregner kalorieforbrug i perioden til kalorieforbrug i timen
                    start_cal_hour = ticks_ms()                               # Genstarter 10-sekunders timer
            
            if lcd_display == 0:
                if gps_data != None and gps_data != False:                    # Viser nuværende lat, lon, course og speed på LCD-skærm
                    lcd.clear()
                    lcd.move_to (0, 1)
                    lcd.putstr(f'Latitude: {str(gps_data[0])}')
                    lcd.move_to (0, 2)
                    lcd.putstr(f'Longitude: {str(gps_data[1])}')
                    lcd.move_to (0, 0)
                    lcd.putstr(f'{str(round(gps_data[3],2))} kph')
                    lcd.move_to (0, 3)
                    lcd.putstr(f"Dir: {str(round(gps_data[2],1))}")
                    lcd.move_to (15, 0)
                    lcd.putstr(f'{str(round(temp))}°C')
                    lcd.move_to (11, 3)
                    lcd.putstr(f'Battery: {str(round(battery_percentage))}%')
                    
                else:                                                         # Fejlbesked, hvis der ikke er data fra GPS
                    lcd.clear()
                    lcd.move_to (0, 0)
                    lcd.putstr("Connecting...")
                    
            if lcd_display == 50:
                if data_pulse != None and data_pulse != 0 and data_gyro != None and calories_hour != 0:   # Viser nuværende data fra pulsmåler og gyroskop på LCD-skærm
                    lcd.clear()
                    lcd.move_to (0,0)
                    lcd.putstr(f"BPM: {round(data_pulse)}")
                    lcd.move_to (0, 1)
                    lcd.putstr(f"Calories/h: {round(calories_hour)}")
                    lcd.move_to (0, 2)
                    lcd.putstr(f'Calories burned: {round(calories_burned)}')
                    lcd.move_to (11, 3)
                    lcd.putstr(f'Battery: {str(round(battery_percentage))}%')
                else:                                                                                 # Fejlbesked, hvis der ikke er data fra pulsmåler og gyroskop
                    lcd.clear()
                    lcd.move_to (0, 0)
                    lcd.putstr("177013")
                
            lcd_display += 1
                
            if lcd_display == 100:
                lcd_display = 0
                
            if acceleration.x < .5 and park_accel != 1:
                set_color(100, 0, 0)
                sleep(1)
                if acceleration.x <= .1 and acceleration.x >= -.1:
                    set_color(200, 0, 0)
                    park_accel = 1
            if acceleration.x > .5 or acceleration.x < -.5:
                set_color(0, 0, 0)
                park_accel = None
                
            if acceleration.x <= 0.1 and acceleration.x >= -0.1 and parked != True:             # Tjekker om cyklen står stille
                timer_deepsleep.init(period=180000, mode=Timer.ONE_SHOT, callback=sleep_bish)   # Starter timer til deepsleep
                parked = True                                                                   # Fortæller programmet at cyklen står stille, så den ikke genstarter deepsleep-timer
            if acceleration.x < -0.1 or acceleration.x > 0.1:
                timer_deepsleep.deinit()                                                        # Slukker timer til deepsleep
                parked = False                                                                  # Fortæller programmet at cyklen ikke længere står stille
            
            if gps_data != False and gps_data != None:
                telemetry = {"latitude": float(gps_data[0]), "longitude": float(gps_data[1]), "course": float(gps_data[2]), "speed": gps_data[3],
                             "cal_burn": calories_burned, "cal_hour": calories_hour, "pulse_data": data_pulse,
                             "temperature": temp, "battery": battery_percentage}                # Opretter dictionary med data
                client.send_telemetry(telemetry)                                                # Sender dictionary med data til Thingsboard
                
        if alarm == True:              # Slår alarmsystemet til
            if acceleration.x >= .5:
                set_color(255, 0, 0)
                buzzer.duty(512)
                buzzer.freq(2000)
                sleep(.25)
                set_color(0, 0, 0)
                buzzer.duty(0)
                sleep(.15)
            else:
                set_color(0, 0, 0)
                buzzer.duty(0)
                
        sleep(.1)
    
    except KeyboardInterrupt:
        client.disconnect()
        break