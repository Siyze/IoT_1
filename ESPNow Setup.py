import network
import espnow
import ubinascii

wlan_sta = network.WLAN(network.STA_IF)
wlan_sta.active(True)

wlan_mac = wlan_sta.config('mac')
print("MAC Address as bytestring:", wlan_mac)
print("MAC Address as Hex:", ubinascii.hexlify(wlan_mac).decode())

### If sending
# e = espnow.ESPNow()
# e.active(True)
# peer_gyro = b'\xc8.\x18\x15<\xfc'
# e.add_peer(peer_gyro)
# 
# e.send("Starting...")
# e.send("Test message")
# e.send(b'end')

### If recieving
# while True:
#     host, msg = e.recv()
#     if msg:
#         print(host, msg)
#         if msg == b'end':
#             break