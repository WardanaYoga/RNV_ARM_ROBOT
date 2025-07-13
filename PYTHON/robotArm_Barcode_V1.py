import cv2
from pyzbar.pyzbar import decode
import serial
import time
import os
import numpy as np

# === SETUP SERIAL ===
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)
ser.flushInput()
ser.flushOutput()

# === SETUP KAMERA ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# === FOLDER SIMPAN BARCODE ===
folder = "barcode_log"
os.makedirs(folder, exist_ok=True)

# === G-CODE ===
cmdList = [
    "G0 X0.00 Y216.90 Z138.00 E0.00 F0.00",
    "G0 X-200.00 Y216.90 Z138.00 E0.00 F0.00",
    "G0 X-200.00 Y90.90 Z138.00 E0.00 F0.00",
    "G0 X-200.00 Y90.90 Z110.00 E0.00 F0.00",
    "G0 X-200.00 Y90.90 Z110.00 E250.00 F0.00",
    "G0 X0.00 Y216.90 Z138.00 E250.00 F0.00",
    "G0 X0.00 Y216.90 Z0.00 E250.00 F0.00",
    "G0 X0.00 Y216.90 Z-90.00 E250.00 F0.00",
    "M5",
    "G4 S0.05",
    "M3",
    "G4 S0.05",
    "G0 X0.00 Y216.90 Z200.00 E250.00 F0.00",
    "G0 X0.00 Y216.90 Z138.00 E250.00 F0.00",
    "G0 X200.00 Y9.90 Z138.00 E250.00 F0.00",
    "G0 X200.00 Y9.90 Z138.00 E0.00 F0.00",
    "G0 X0.00 Y216.90 Z138.00 E0.00 F0.00",
    "G0 X0.00 Y216.90 Z-90.00 E0.00 F0.00",
    "M5",
    "G0 X0.00 Y216.90 Z200.00 E0.00 F0.00",
    "M3",
    "G0 X0.00 Y216.90 Z138.00 E0.00 F0.00"
]

bCmdList = [(cmd + '\r').encode('utf-8') for cmd in cmdList]

def tunggu_selesai(max_wait=20):
    start_time = time.time()
    while True:
        if time.time() - start_time > max_wait:
            print("?? Timeout respon Arduino.")
            break
        if ser.in_waiting:
            response = ser.readline().decode("utf-8", errors="ignore").strip()
            if response:
                print(f"[{time.strftime('%H:%M:%S')}] RESPON: '{response}'")
                if "ok" in response.lower():
                    break

def kirim_gcode():
    print("? Barcode valid. Menjalankan G-code...")
    for i, cmd in enumerate(bCmdList):
        print(f"? Kirim perintah {i+1}: {cmd.decode().strip()}")
        ser.write(cmd)
        tunggu_selesai()
        time.sleep(0.3)

# === Homing sebelum mulai ===
print("?? Homing...")
ser.write(b'G28\r')
tunggu_selesai()
time.sleep(1)

print("? Sistem aktif. Arahkan barcode ke kamera...")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    barcodes = decode(frame)

    if barcodes:
        barcode = barcodes[0]
        data = barcode.data.decode('utf-8')
        filename = f"{folder}/barcode_{data}.jpg"

        # Jika file belum ada ? simpan gambar & jalankan G-code
        if not os.path.exists(filename):
            print(f"?? Barcode baru: {data}")
            cv2.imwrite(filename, frame)
            kirim_gcode()
        else:
            print(f"? Barcode '{data}' sudah pernah dijalankan.")

        # Gambar kotak hijau di sekitar barcode
        pts = barcode.polygon
        if len(pts) > 3:
            pts = [(pt.x, pt.y) for pt in pts]
            pts = cv2.convexHull(np.array(pts)).astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    # Tampilkan hasil kamera
    cv2.imshow("Barcode Scanner", frame)
    if cv2.waitKey(1) == ord('q'):
        break

# === Cleanup ===
cap.release()
cv2.destroyAllWindows()
ser.close()
print("? Program selesai dan koneksi ditutup.")
