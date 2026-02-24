import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200)
plt.ion()

# === Plot Styles to Rotate Through ===
plot_styles = ['r.-', 'b.-', 'g.-', 'm.-', 'c.-', 'y.-', 'k.-']

# === Data Storage ===
NUM_SAMPLES = 100
time_ms = []
cnt = 0
all_signals = []  # List of lists. all_signals[0] = i values, all_signals[1] = j values, etc.
signals = ["Temperature", "X", "Y", "Z"]



# === Plotting Function ===
def makeFig(title, xLabel, yLabel, yLimit, *args):
    plt.clf()
    plt.title(title)
    plt.grid(True)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.ylim(yLimit[0], yLimit[1])  

    for data, style, label in args:
        plt.plot(time_ms, data, style, label=label)

    plt.legend(loc='upper left')


# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass  # Wait for data

    try:
        line = sinWaveData.readline().decode().strip()
        # Replace multiple spaces/tabs with single space
        values = line.replace('\t', ' ').split()
        # Convert to int
        values = [int(v) for v in values]

        # Initialize lists if first time
        if len(all_signals) != len(values):
            all_signals = [[] for _ in range(len(values))]

        # Append values to corresponding lists
        for i, val in enumerate(values):
            all_signals[i].append(val)

        time_ms.append(cnt)
        cnt += 1

        # Build arguments for plotting
        args = []
        for i, signal_data in enumerate(all_signals):
            style = plot_styles[i % len(plot_styles)]
            label = signals[i] if i < len(signals) else f"Signal {i}"
            args.append((signal_data, style, label))

        drawnow(lambda: makeFig("Gyroscope Plot", "Time (ms)", "Value", [-255, 255], *args))
        plt.pause(0.0001)

        # Keep only last NUM_SAMPLES samples
        if len(time_ms) > NUM_SAMPLES:
            time_ms.pop(0)
            for signal in all_signals:
                signal.pop(0)

    except Exception as e:
        print("Error:", e)