print("BOOT")

import gc
gc.collect()
import network
from machine import reset, Pin
from time import ticks_ms
import secrets

ssid = secrets.SSID
password = secrets.PASSWORD

def do_connect():
    wlan = network.WLAN(network.STA_IF)
    print('WLAN status:', wlan.status())
    wlan.active(True)
    try:
        if not wlan.isconnected():
            print('connecting to network...')
            wlan.connect(ssid, password)
            print('WLAN status:', wlan.status())
            start = ticks_ms()
            while not wlan.isconnected():
                if ticks_ms() - start > 10000:
                    print("Could not connect to wifi!")
                    break

    except Exception as e:
        print(f"WiFi error '{e}' occured, rebooting system")
        reset()
    finally:
        if wlan.isconnected():
            print("Connected to wifi!")
            print(f"wifi statuscode {wlan.status()}")
    return wlan    
    
# wlan = do_connect()