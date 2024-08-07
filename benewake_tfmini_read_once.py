import serial
import sys

def find_message(ser):
    while True:
        byte1 = ser.read(1)
        if byte1 == b'\x59':
            byte2 = ser.read(1)
            if byte2 == b'\x59':
                message = ser.read(7)
                return b'\x59\x59' + message

def is_valid(message):
    if len(message) != 9:
        raise ValueError("Input must be exactly 9 bytes.")
    
    # Calculate the truncated sum of the first eight bytes
    sum_value = 0x00
    for i in range(8):
        sum_value += message[i]
        sum_value &= 0xFF
        
    # Check if the truncated sum equals the 9th byte
    return sum_value == message[8]

def main():
    port = '/dev/ttyACM0'  # Replace with your serial port
    baudrate = 115200
    start = int(sys.argv[1])

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud rate.")
        print("Truth(cm)\tDistance(cm)\tStrength\tTemperature(℃)")
        while True:
            # print("Waiting for a valid message...")
            message = find_message(ser)
            # print("Received message:", message.hex(' '))
            if is_valid(message):
                distance = message[2] + message[3]*256
                strength = message[4] + message[5]*256
                temperature = message[6] + message[7]*256
                temperature = (temperature/8) - 256
                print(str(start) + "\t" + str(distance) + "\t" + str(strength) + "\t" + str(temperature))
                input("")
                start+= 1
                ser.reset_input_buffer()

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nProgram interrupted. Exiting.")
        sys.exit(0)
    finally:
        if ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
