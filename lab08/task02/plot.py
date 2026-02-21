import serial
import matplotlib.pyplot as plt
from drawnow import drawnow
import time

PORT = '/dev/ttyUSB0'   # change if needed
BAUD = 115200

# === Auto connect ===
while True:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print("Connected to", PORT)
        break
    except:
        print("Waiting for port...")
        time.sleep(1)

plt.ion()

tempData = []
timeData = []
cnt = 0


def makeFig():
    plt.clf()
    plt.title("Live Temperature")
    plt.grid(True)
    plt.xlabel("Sample")
    plt.ylabel("Temperature")
    plt.plot(timeData, tempData, label="Temp")
    plt.legend()


while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        # case 1 → only temp sent
        if "," not in line:
            temp = float(line)

        # case 2 → CSV format (temp first)
        else:
            temp = float(line.split(",")[0])

        tempData.append(temp)
        timeData.append(cnt)
        cnt += 1

        drawnow(makeFig)
        plt.pause(0.001)

        if len(tempData) > 300:
            tempData.pop(0)
            timeData.pop(0)

    except Exception:
        pass