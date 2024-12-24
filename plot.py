import serial
import matplotlib.pyplot as plt
from collections import deque
import re

# Funkcja do parsowania danych
def parse_line(line):
    match = re.search(r"angle: ([\d.-]+), ([\d.-]+) kalman angle: ([\d.-]+)", line)
    if match:
        return float(match.group(1)), float(match.group(2)), float(match.group(3))
    return None

def main():
    # Ustawienia portu szeregowego
    port = "COM1"
    baud_rate = 115200

    # Inicjalizacja list na dane
    max_samples = 1000
    x_data = deque(maxlen=max_samples)
    z_data = deque(maxlen=max_samples)

    # Połączenie z portem szeregowym
    try:
        ser = serial.Serial(port, baud_rate, timeout=0.1)  # Mniejszy timeout dla szybszego odczytu
        print(f"Połączono z {port} z prędkością {baud_rate}.")
    except serial.SerialException as e:
        print(f"Błąd połączenia z portem szeregowym: {e}")
        return

    # Przygotowanie wykresu
    plt.ion()
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], label="angle x")
    line3, = ax.plot([], [], label="kalman angle")
    ax.legend()
    ax.set_title("Real-time data from COM4")
    ax.set_xlabel("Samples")
    ax.set_ylabel("Values")

    # Licznik iteracji do rzadszej aktualizacji wykresu
    update_interval = 10  # Aktualizuj wykres co 10 iteracji
    iteration_count = 0

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                parsed = parse_line(line)

                if parsed:
                    x, _, z = parsed
                    x_data.append(x)
                    z_data.append(z)

                    iteration_count += 1
                    if iteration_count % update_interval == 0:
                        # Aktualizacja wykresu co kilka iteracji
                        line1.set_data(range(len(x_data)), x_data)
                        line3.set_data(range(len(z_data)), z_data)

                        ax.relim()
                        ax.autoscale_view()
                        plt.pause(0.001)

    except KeyboardInterrupt:
        print("Przerwano przez użytkownika.")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
