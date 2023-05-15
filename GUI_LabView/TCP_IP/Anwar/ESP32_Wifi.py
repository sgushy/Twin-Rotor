import network
import socket
import binascii




import gc
gc.collect()

import esp
esp.osdebug(None)

wlan = network.WLAN(network.STA_IF) # create station interface
wlan.active(True)       # activate the interface
wlan.scan()             # scan for access points
wlan.isconnected()      # check if the station is connected to an AP
wlan.connect('Berkeley-IoT', 'A(uxgHF6')# connect to an AP
wlan.config('mac')      # get the interface's MAC address
print('this is IP address')
print(wlan.ifconfig())         # get the interface's IP/netmask/gw/DNS addresses

ap = network.WLAN(network.AP_IF) # create access-point interface
ap.active(True)
macaddress = (ap.config('mac'))

print(macaddress)

print('this is mac address')
mac_str = binascii.hexlify(ap.config('mac')).decode()
print(mac_str)

ap.config(essid='ESP32',password='ME135') # set the SSID of the access point
print(ap.config('essid'))
ap.config(max_clients=10) # set how many clients can connect to the network
ap.active(True)         # activate the interface

print(ap.ifconfig())

def web_page():
  html = """<html><head><meta name="viewport" content="width=device-width, initial-scale=1"></head>
  <body><h1>Hello, World! This is your ESP32 Talking</h1></body></html>"""
  return html

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(30)

while True:
  conn, addr = s.accept()
  print('Got a connection from %s' % str(addr))
  request = conn.recv(1024)
  print('Content = %s' % str(request))
  response = web_page()
  conn.send(response)
  conn.close()