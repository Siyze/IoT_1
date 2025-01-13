import network
import espnow
import ubinascii
import gc
from time import sleep

wlan_sta = network.WLAN(network.STA_IF)
wlan_sta.active(True)

wlan_mac = wlan_sta.config('mac')
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

e = espnow.ESPNow()
e.active(True)
e.config(timeout_ms=-1)
peer = b'\xd4\x8a\xfch\x9f\x84'
e.add_peer(peer)