/**************************************************************************************************************
     This code tests the range finder function of the URM14 ultrasonic sensor
     @ author : roker.wang@dfrobot.com
     @ data   : 11.08.2020
     @ version: 1.0
**************************************************************************************************************/
#include <modbus.h>
#include <iostream>
#include <limits>

#define   SLAVE_ADDR                ((uint16_t)0x0C)

#define   TEMP_CPT_SEL_BIT          ((uint16_t)0x01)
#define   TEMP_CPT_ENABLE_BIT       ((uint16_t)0x01 << 1)
#define   MEASURE_MODE_BIT          ((uint16_t)0x01 << 2)
#define   MEASURE_TRIG_BIT          ((uint16_t)0x01 << 3)

typedef enum{
  ePid,
  eVid,
  eAddr,
  eComBaudrate,
  eComParityStop,
  eDistance,
  eInternalTempreture,
  eExternTempreture,
  eControl,
  eNoise
}eRegIndex_t;//Sensor register index

/*
 *@brief Read data from holding register of client
 *
 *@param addr ： Address of Client
 *@param reg: Reg index
 *@return data if execute successfully, false oxffff.
 */
uint16_t readData(modbus_t *mb, eRegIndex_t reg)
{
  uint16_t data = 0xffff;

  // Set the Modbus slave ID
  modbus_set_slave(mb, SLAVE_ADDR);
  if (modbus_read_registers(mb, reg, 1, &data) < 0) {
    std::cerr << "Failed to write coil: " << modbus_strerror(errno) << std::endl;
    data = 0xffff;
  }

  return data;
}

/*
 *@brief write data to holding register of client
 *
 *@param addr ： Address of Client
 *@param reg: Reg index
 *@param data: The data to be written
 *@return 1 if execute successfully, false 0.
 */
uint16_t writeData(modbus_t *mb, eRegIndex_t reg, uint16_t data)
{
  // Set the Modbus slave ID
  modbus_set_slave(mb, SLAVE_ADDR);

  if (modbus_write_register(mb, reg, data) < 0) {
    std::cerr << "Failed to write coil: " << modbus_strerror(errno) << std::endl;
    return 0;
  }
  return 1;
}

int main () {
  // Parameters
  const char* device = "/dev/ttyACM0"; // Serial device file (adjust as needed)
  const int baudrate = 19200;          // Baud rate
  const unsigned long timeout = 1000;  // Default timeout

  // Create a new Modbus RTU context
  modbus_t* mb = modbus_new_rtu(device, baudrate, 'N', 8, 1);
  if (mb == nullptr) {
    std::cerr << "Failed to create the Modbus RTU context." << std::endl;
    return -1;
  }

  // Connect to the Modbus device
  if (modbus_connect(mb) == -1) {
    std::cerr << "Failed to connect to the Modbus device: " << modbus_strerror(errno) << std::endl;
    modbus_free(mb);
    return -1;
  }

  modbus_set_error_recovery(mb, MODBUS_ERROR_RECOVERY_PROTOCOL);

  modbus_set_response_timeout(mb, timeout / 1000, (timeout % 1000) * 1000);


  volatile uint16_t cr = 0;
  cr |= MEASURE_MODE_BIT;//Set bit2 , Set to trigger mode
  cr &= ~(uint16_t)TEMP_CPT_SEL_BIT;//Select internal temperature compensation
  cr &= ~(uint16_t)TEMP_CPT_ENABLE_BIT;//enable temperature compensation

  writeData(mb, eControl, cr); //Writes the setting value to the control register
  usleep(100*1000);

  float  dist = std::numeric_limits<float>::quiet_NaN();
  
  while(true) {
    cr |= MEASURE_TRIG_BIT;//Set trig bit
    writeData(mb, eControl, cr); //Write the value to the control register and trigger a ranging
    usleep(300*1000);//Delay of 300ms(minimum delay should be greater than 30ms) is to wait for the completion of ranging
    dist = static_cast<float> (readData(mb, eDistance)) / 10.0;//Read distance register, one LSB is 0.1mm

    std::cout << "distance: " << dist << "mm" << "\n";
  }

  // Close the connection and free resources
  modbus_close(mb);
  modbus_free(mb);

  return 0;
}
