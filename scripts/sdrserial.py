import serial
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
PORT="/dev/ttyACM0"

ser = serial.Serial()
ser.port = PORT 
ser.baudrate = 115200
ser.timeout = 5


ser.open()
while (1):
    line = ser.readline().decode()
    print(line, end=None)
    if line == "{{\r\n":
        break 

data_lines = []
line = ser.readline().decode()
while (line != "}}\r\n"):
    data_lines.append(line)
    line = ser.readline().decode()
ser.close()

data = []
for line in data_lines:
    print(line, end="")
    strvals = line.strip(',\r\n').split(',')
    for val in strvals:
        data.append(int(val))

plt.figure()
plt.plot(data, '-')
plt.show()

plt.stem(abs(fft(data)) / 400)
plt.show()

#while ser.readline().decode("utf-8") != "{{\n":
    