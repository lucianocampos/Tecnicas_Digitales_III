import time
import sys

import serial

def main():
    # Parámetros configurables
    puerto = 'COM6'
    velocidad = 19200  # Cambia aquí si querés otro baudrate
    bytesize = serial.EIGHTBITS
    parity   = serial.PARITY_NONE
    stopbits = serial.STOPBITS_ONE
    timeout  = 1       # Timeout de lectura/escritura en segundos

    try:
        ser = serial.Serial(
            port=puerto,
            baudrate=velocidad,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )
    except serial.SerialException as e:
        print(f"ERROR: No se pudo abrir {puerto}: {e}")
        sys.exit(1)

    print(f"Puerto {puerto} abierto a {velocidad} bps. Enviando trama cada 1 s…")

    # Trama de bytes a enviar
    trama = bytes([0x02, 0x07, 0x93, 0x01, 0x00, 0x00, 0x7C])

    try:
        while True:
            ser.write(trama)
            ser.flush()
            print(f"Enviado: {trama.hex(' ').upper()}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
    finally:
        ser.close()
        print(f"Puerto {puerto} cerrado.")

if __name__ == '__main__':
    main()
