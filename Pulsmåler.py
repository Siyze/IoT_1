from machine import Pin, ADC, deepsleep
from time import sleep, ticks_ms
import network
import ubinascii
import espnow

### Konfiguuration

wlan_sta = network.WLAN(network.STA_IF)   # Opretter WLAN objekt
wlan_sta.active(True)                     # Aktivering af netværk

wlan_mac = wlan_sta.config('mac')         # Visning af MAC-adresse
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

e = espnow.ESPNow()                           # Opretter ESPNow objekt
e.active(True)                                # Aktivering af ESPNow
e.config(timeout_ms=-1)
peer_localhost = b'\xd4\x8a\xfch\x9f\x84'     # Tilføjer peer for localhost
e.add_peer(peer_localhost)

pulse_sensor = ADC(Pin(34, Pin.IN))   # Opretter pulsmåler objekt
pulse_sensor.atten(ADC.ATTN_0DB)      # Sætter range for pulsmåler til 0-3.3V
pulse_sensor.width(ADC.WIDTH_10BIT)   # Sætter værdier for pulsmåler mellem 0 og 1023

MAX_HISTORY = 250                     # Max længde på gemte puls-værdier
sample_rate = 0.05                    # Længde mellem sampling af værdier
last_value = 0                        # Default value for last_value
pulse_count = 0                       # Default value for pulse_count
heart_rate = 0                        # Default value for heart_rate
start_time = ticks_ms()

history = []                          # Opretter log til at bestemme min, max og threshold

### Funktioner

def detect_heartbeat():
    global pulse_count, last_value, history
    
    current_value = pulse_sensor.read()            # Læser værdi fra pulsmåler
    
    history.append(current_value)                  # Lægger værdi fra pulsmåler i listen
    history = history[-MAX_HISTORY:]               # Henter history op til MAX_HISTORY
    minima, maxima = min(history), max(history)    # Finder min og max fra history
    
    threshold_beat = (minima + maxima * 3) // 4    # Udregner et slag til at være 3/4 af maximum
    threshold_no_beat = (minima + maxima) // 2     # Udregner intet slag til at være halvdelen af maximum
    
    if current_value > threshold_beat and last_value <= threshold_no_beat:   # pulse_count bliver tillagt 1, hvis der opfanges et slag
        pulse_count += 1
        print(current_value)
        
    last_value = current_value
    
def calculate_heartrate():
    global pulse_count, start_time
    
    elapsed_time = (ticks_ms() - start_time) / 10000   # Udregner, hvor længe der er gået
    heart_rate = (pulse_count / elapsed_time) * 6      # Udregner BPM
    return heart_rate

### Program

while True:
    detect_heartbeat()                        # Måler hjerteslag
    if ticks_ms() - start_time >= 10000:      # Opdaterer BPM, hvert 10'ende sekund
        heart_rate = calculate_heartrate()
        print(f"BPM {round(heart_rate,2)}")
        
        pulse_count = 0                       # Resetter pulse_count efter BPM er udregnet
        start_time = ticks_ms()               # Resetter start_time
        
    sleep(sample_rate)                        # Venter til næste sample skal tages
    
    if heart_rate != 0:
        e.send(str(heart_rate))
        sleep(sample_rate)
        
    host, msg = e.recv()
    if msg:
        print(host, msg)
        if msg == b'Wake up! ajkdsladhwoiahjdal Make Up!':
            print("Waking up...")
            e.send("0")
        if msg == b'sleep, bish!':
            deepsleep(20000)