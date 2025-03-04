# Simple HTTP Server Example
# Control an LED and read a Button using a web browser

import time
import network
import socket
from machine import Pin,UART
from rotary_irq_rp2 import RotaryIRQ

rotary = RotaryIRQ(14, 15)
current_val = 0  # Track the last known value of the encoder
led = Pin(17, Pin.OUT)
ledState = 'LED State Unknown'
button = Pin(16, Pin.IN, Pin.PULL_UP)

ssid = ''
password = ''

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)
#ledx = Pin("LED", Pin.OUT)


# replace the "html" variable with the following to create a more user-friendly control panel
html = """<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
.buttonGreen { background-color: #4CAF50; border: 2px solid #000000;; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; }
.buttonRed { background-color: #D11D53; border: 2px solid #000000;; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; }
text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
</style></head>
<body><center><h1>Control Panel</h1></center><br><br>
<form><center>
<center> <button class="buttonGreen" name="led" value="on" type="submit">LED ON</button>
<br><br>
<center> <button class="buttonRed" name="led" value="off" type="submit">LED OFF</button>
</form>
<br><br>
<br><br>
<p>%s<p></body></html>
"""

# Wait for connect or fail
max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)
    
# Handle connection error
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('Connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )
    
    
# Open socket
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print('listening on', addr)

# Listen for connections, serve client
while True:
    try:       
        cl, addr = s.accept()
        print('client connected from', addr)
        request = cl.recv(1024)
        print("request:")
        print(request)
        request = str(request)
        led_on = request.find('led=on')
        led_off = request.find('led=off')
        
        print( 'led on = ' + str(led_on))
        print( 'led off = ' + str(led_off))
        
        
        while True:
            new_val = rotary.value()  # What is the encoder value right now?
            
            if current_val != new_val:  # The encoder value has changed!
                print('Encoder value:', new_val)  # Do something with the new value
                
                current_val = new_val  # Track this change as the last know value
                time.sleep(0.2)
            else:
                print("no encoder change")
                break
                
                
        


        if led_on == 8:
            print("led on")
            led.value(1)
        if led_off == 8:
            print("led off")
            led.value(0)
        
        ledState = "LED is OFF" if led.value() == 0 else "LED is ON" # a compact if-else statement
        
        if button.value() == 1: # button not pressed
            print("button NOT pressed")
            buttonState = "Button is NOT pressed"
        else:
            print("button pressed")
            buttonState = "Button is pressed"
        
        # Create and send response
        stateis = ledState + " and " + buttonState
        response = html % stateis
        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()
        
    except OSError as e:
        cl.close()
        print('connection closed')
