# pip install pyserial
import csv
import sys
import time
from datetime import datetime
import serial

# --- CONFIG --- (ajustá estos valores)
PORT = "COM5"
BAUD = 9600
OUTFILE = "log_serial.csv"

def main():

    with open(OUTFILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        if f.tell() == 0:
            writer.writerow(["iso_time", "raw_line"])

        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            print(f"Grabando desde {PORT} @ {BAUD} → {OUTFILE} ")
            try:
                while True:
                    line = ser.readline()  # lee hasta \n
                    if not line:
                        continue
                    try:
                        text = line.decode("utf-8", errors="replace").strip()
                    except Exception:
                        text = str(line).strip()
                    writer.writerow([datetime.now().isoformat(timespec="seconds"), text])
                    f.flush()  # asegura que se escriba al disco
            except KeyboardInterrupt:
                print("\nDetenido por el usuario.")
            except serial.SerialException as e:
                print(f"\nError de puerto: {e}")
            except Exception as e:
                print(f"\nError: {e}")

if __name__ == "__main__":
    # Permite pasar COM/baud por argumentos: python logger_serial.py COM7 230400
    if len(sys.argv) >= 2:
        PORT = sys.argv[1]
    if len(sys.argv) >= 3:
        BAUD = int(sys.argv[2])
    main()

## python logger_serial.py           # usa COM5 @115200 (según config del script)
## python logger_serial.py COM7 9600 # ejemplo pasando COM y baud por parámetros
