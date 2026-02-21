import time, serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
ser = serial.Serial('/dev/ttyUSB0', 115200)

plt.ion()

tempData = []
xData = []
yData = []
zData = []
time_ms = []
cnt = 0


# === Plot Function ===
def makeFig():
    plt.clf()
    plt.title('Live Gyroscope Data')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Value')

    plt.plot(time_ms, tempData, 'k.-', label='Temp')
    plt.plot(time_ms, xData, 'r.-', label='X dps')
    plt.plot(time_ms, yData, 'g.-', label='Y dps')
    plt.plot(time_ms, zData, 'b.-', label='Z dps')

    plt.legend(loc='upper left')

while True:
    try:
        ser = serial.Serial('/dev/ttyUSB0',115200)
        break
    except:
        print("Waiting for port...")
        time.sleep(1)
        
# === Main Loop ===
while True:
    while ser.inWaiting() == 0:
        pass

    try:
        line = ser.readline().decode().strip()
        values = line.split(',')

        if len(values) == 4:
            temp = float(values[0])
            x = float(values[1])
            y = float(values[2])
            z = float(values[3])

            tempData.append(temp)
            xData.append(x)
            yData.append(y)
            zData.append(z)

            time_ms.append(cnt)
            cnt += 1

            drawnow(makeFig)
            plt.pause(0.0001)

            # keep last 500 samples
            if len(tempData) > 500:
                tempData.pop(0)
                xData.pop(0)
                yData.pop(0)
                zData.pop(0)
                time_ms.pop(0)

    except Exception as e:
        print("Error:", e)