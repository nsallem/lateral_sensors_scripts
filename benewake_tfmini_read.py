import serial
import time
ser = serial.Serial("/dev/ttyACM0", 115200)
# we define a new function that will get the data from LiDAR and publish it
def read_data():
    time.sleep(1)
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # python3
                distance = (bytes_serial[2] + bytes_serial[3]*256) / 100.0
                strength = bytes_serial[4] + bytes_serial[5]*256
                temperature = bytes_serial[6] + bytes_serial[7]*256
                temperature = (temperature/8) - 256
                print("Distance: "+ str(distance) + "m，Strength: " + str(strength) + ", Temperature: " + str(temperature)+ "℃")
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
