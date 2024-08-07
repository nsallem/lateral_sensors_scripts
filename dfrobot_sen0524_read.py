import serial
import time

ser = serial.Serial("/dev/ttyUSB0", 115200)

# we define a new function that will get the data from LiDAR and publish it
def read_data():
    time.sleep(1)
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            data = ser.read(16)
            ser.reset_input_buffer()

            if data[0] == 0x57 and data[1] == 0x00 and data[2] == 0xFF and data[1] == 0x00:
                strength = data[12] + data[13]*256

                if strength > 0:
                    distance = (data[8] + data[9]*256) / 1000.0
                    precision = data[14] / 100.0
                    print("Distance: "+ str(distance) + "m, Precision: " + str(precision)+ "m，Strength: " + str(strength))
                    ser.reset_input_buffer()

if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        read_data()
    except KeyboardInterrupt:
        if ser != None:
            ser.close()
            print("program interrupted by the user")
