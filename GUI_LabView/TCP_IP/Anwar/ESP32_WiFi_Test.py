import network

wlan = network.WLAN(network.STA_IF) # create station interface
wlan.active(True)       # activate the interface
wlan.scan()             # scan for access points
wlan.isconnected()      # check if the station is connected to an AP
wlan.connect('Berkeley-IoT', 'A(uxgHF6') # connect to an AP

wlan.config('mac')      # get the device MAC address
print('{:02x}.{:02x}.{:02x}.{:02x}'.format(*bytearray(wlan.config('mac'))))
print(wlan.ifconfig())         # get the interface's IP/netmask/gw/DNS addresses

