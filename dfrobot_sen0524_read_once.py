import serial
import sys

def find_message(ser):
    byte1 = ser.read(1)
    if byte1 == b'\x57':
        byte2 = ser.read(1)
        if byte2 == b'\x00':
            byte3 = ser.read(1)
            if byte3 == b'\xFF':
                byte4 = ser.read(1)
                if byte4 == b'\x00':
                    message = ser.read(12)
                    return b'\x57\x00\xFF\x00' + message

def is_valid(message):
    if len(message) != 16:
        raise ValueError("Input must be exactly 16 bytes.")
    
    # Calculate the truncated sum of the first 15 bytes
    sum_value = 0x00
    for i in range(15):
        sum_value += message[i]
        sum_value &= 0xFF
        
    # Check if the truncated sum equals the 16th byte
    return sum_value == message[15]

def main():
    port = '/dev/ttyACM0'  # Replace with your serial port
    baudrate = 115200
    start = int(sys.argv[1])

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud rate.")
        print("Truth(cm)\tDistance(mm)\tStrength\tPrecision(cm)")
        while True:
            # print("Waiting for a valid message...")
            message = find_message(ser)
            # print("Received message:", message.hex(' '))
            if is_valid(message):
                strength = message[12] + message[13]*256

                if strength == 0:
                    print(str(start) + "\tOut of range!")
                else:
                    distance = message[8] + message[9]*256
                    precision = message[14]
                    print(str(start) + "\t" + str(distance) + "\t" + str(strength) + "\t" + str(precision), end="")

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
