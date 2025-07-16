import serial
import csv
import time
import keyboard  # pip install keyboard
import time

# --- Config ---
PORT = 'COM8'
BAUD = 115200
CSV_FILE = 'live_data_1.csv'

# --- Start serial ---
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("Logging started. Press 'u' for upright, 's' for slouch. Ctrl+C to stop.")

current_label = 'upright'
time_bound = 1

with open(CSV_FILE, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    transition = 0
    start_time = time.time()
    writer.writerow(['Time','Roll', 'Pitch', 'Label'])
    transition_time = time.time()

    try:
        while True:
            # Check keyboard input
            if (time.time() - transition_time) > time_bound and transition == 1:
                transition = 0
            if keyboard.is_pressed('u'):
                current_label = 'upright'
                transition = 1
                transition_time = time.time()
            elif keyboard.is_pressed('s'):
                current_label = 'slouch'
                transition = 1
                transition_time = time.time()

            # Read from serial
            line = ser.readline().decode('utf-8').strip()
            print("RAW LINE:", line)
            if not line:
                continue

            try:
                roll, pitch = map(float, line.split(",")[:2])
                timestamp = time.time() - start_time
                writer.writerow([timestamp, roll, pitch, current_label, transition])
                print(f"{timestamp},{roll:.2f}, {pitch:.2f}, {current_label}, {transition}")
            except ValueError:
                continue  # skip bad lines
    except KeyboardInterrupt:
        print("\nLogging stopped.")
    finally:
        ser.close()
