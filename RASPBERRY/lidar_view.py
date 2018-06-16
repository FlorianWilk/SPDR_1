import subprocess
import serial
import time
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

RST = None
DC = 23
x=0
top=0
SPI_PORT = 0
SPI_DEVICE = 0
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
disp.begin()
disp.clear()
 
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
 
draw = ImageDraw.Draw(image)
draw.rectangle((0,0,width,height), outline=0, fill=0)
font = ImageFont.load_default()

if True:
    draw.rectangle((0,0,width,height), outline=0, fill=0)
 
    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell = True )
    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    MemUsage = subprocess.check_output(cmd, shell = True )
    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    Disk = subprocess.check_output(cmd, shell = True )
 
    # Write two lines of text.
 
    draw.text((x, top),       str(IP) + " "+str(CPU),  font=font, fill=255)
//    draw.text((x, top+16),    str(MemUsage),  font=font, fill=255)
//    draw.text((x, top+25),    str(Disk),  font=font, fill=255)
 
    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(.1)

// TODO: try 38400 Baud
s = serial.Serial('/dev/ttyS0', 9600) # Connect to our Arduino Mega 
s.close()
s.open()
time.sleep(5) # Arduino does a reset after a serial connection, so lets wait some secs
 
s.write("S")
try:
    while True:
        response = s.readline()
        print(response)
	if response == "*UP\r\n":
		s.write("S")
		print("IS UP")
except KeyboardInterrupt:
    s.close()
