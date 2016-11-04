/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include "Balloonduino_lib.h"



// Base library type
Balloonduino::Balloonduino()
{
    milliseconds = 0;
    delayMilliseconds = 1000;
    hours = 0;
    minutes = 0;
    seconds = 0;
    launchTolerance = 0;
    altitude = 0;
    temperature = 0;
    pressure = 0;
    isLaunched = false;
}

// TODO other Balloonduino sensors

// Given a meter value, prints meters and feet equivalent to the console without newline
// for example, printMetersAndFeet(20000) prints "20000 meters (65616.96 feet)"
void Balloonduino::printMetersAndFeet(double meters)
{
    Serial.print(meters);
    Serial.print(" meters (");
    Serial.print(meters * 3.28084);
    Serial.print(" feet)");
}

// Given a Celsius value, prints Celsius and Fahrenheit equivalent to the console without newline
// for example, printCelsiusAndFahrenheit(20.0) prints "20.0 Celsius (68.0 Fahrenheit)"
void Balloonduino::printCelsiusAndFahrenheit(double celsius)
{
    Serial.print(celsius);
    Serial.print(" Celsius (");
    Serial.print(celsius * 1.8 + 32);
    Serial.print(" Fahrenheit)");
}

// Given a Pascal value, prints Pascals and atmospheres equivalent to the console without newline
// for example, printPascalsAndAtmospheres(101325) prints "101325 Pascals (1 atm)"
void Balloonduino::printPascalsAndAtmospheres(double pascals)
{
    Serial.print(pascals);
    Serial.print(" Pascals (");
    Serial.print(pascals / 101325);
    Serial.print(" atm)");
}

// prints current altitude in meters and feet
void Balloonduino::printAltitude()
{
    Serial.print("Altitude is ");
    printMetersAndFeet (altitude);
    Serial.print(" above launch site. ");
}

// prints current temperature in Celsius and Fahrenheit
void Balloonduino::printTemperature()
{
    Serial.print("Temperature is ");
    printCelsiusAndFahrenheit (temperature);
    Serial.print(". ");
}

// prints current pressure in millibars
void Balloonduino::printPressure()
{
    Serial.print("Pressure is ");
    printPascalsAndAtmospheres (pressure);
    Serial.print(". ");
}

// Prints current millisecond time in [HH:MM:SS] without newline
void Balloonduino::printFormattedTime()
{
    milliseconds = millis() / 1000;    // convert from milliseconds to seconds
    seconds = milliseconds % 60;
    minutes = milliseconds / 60;
    hours = minutes / 60;

    Serial.print("[");
    if (hours < 10)
    {
        Serial.print("0");
    }
    Serial.print(hours);
    Serial.print(":");
    if (minutes < 10)
    {
        Serial.print("0");
    }
    Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10)
    {
        Serial.print("0");
    }
    Serial.print(seconds);
    Serial.print("] ");
}

void Balloonduino::read_imu(){
  
  #ifndef BALLONDUINO_NO_BNO
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // get measurements
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); // (values in uT, micro Teslas)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // (values in rps, radians per second)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // (values in m/s^2)

  // assign them into structure fields
  IMUData.system_cal = system_cal;
  IMUData.accel_cal = accel_cal;
  IMUData.gyro_cal = gyro_cal;
  IMUData.mag_cal = mag_cal;
  IMUData.accel_x = accel.x();
  IMUData.accel_y = accel.y();
  IMUData.accel_z = accel.z();
  IMUData.gyro_x = gyro.x();
  IMUData.gyro_y = gyro.y();
  IMUData.gyro_z = gyro.z();
  IMUData.mag_x = mag.x();
  IMUData.mag_y = mag.y();
  IMUData.mag_z = mag.z();
  #endif
}

void Balloonduino::read_pwr(){
  
  #ifndef BALLONDUINO_NO_ADS
  PWRData.batt_volt = ((float)ads.readADC_SingleEnded(2)) * 0.002 * 3.0606; // V
  PWRData.i_consump = (((float)ads.readADC_SingleEnded(3)) * 0.002 - 2.5) * 10;
  #endif
}

void Balloonduino::read_env(){
  
  #ifndef BALLONDUINO_NO_BME
  //BME280
  ENVData.bme_pres = bme.readPressure() / 100.0F; // hPa
  ENVData.bme_temp = bme.readTemperature(); // degC
  ENVData.bme_humid = bme.readHumidity(); // %
  #else
  ENVData.bme_pres = 0.0F; // hPa
  ENVData.bme_temp = 0.0F; // degC
  ENVData.bme_humid = 0.0F; // %
  #endif

  //  SSC
  #ifndef BALLONDUINO_NO_BME
  ssc.update();
  ENVData.ssc_pres = ssc.pressure(); // PSI
  ENVData.ssc_temp = ssc.temperature(); // degC
  #else
  ENVData.ssc_pres = 0.0F; // PSI
  ENVData.ssc_temp = 0.0F; // degC
  #endif
  
  // BNO
  #ifndef BALLONDUINO_NO_BNO
  ENVData.bno_temp = bno.getTemp();
  #else
  ENVData.bno_temp = 0.0F;
  #endif
  
  //MCP9808
  #ifndef BALLONDUINO_NO_MCP
  ENVData.mcp_temp = tempsensor.readTempC(); // degC
  #else
  ENVData.mcp_temp = 0.0F;
  #endif
}

void Balloonduino::log_imu(File IMULogFile){
  
  #ifndef BALLONDUINO_NO_BNO
  // print the time to the file
  print_time(IMULogFile);

  // print the sensor values
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.system_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.mag_cal);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_x);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_y);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.accel_z);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_x);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_y);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.gyro_z);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.mag_x);
  IMULogFile.print(", ");
  IMULogFile.print(IMUData.mag_y);
  IMULogFile.print(", ");
  IMULogFile.println(IMUData.mag_z);

  IMULogFile.flush();
  #endif
}

void Balloonduino::log_env(File ENVLogFile){
  // print the time to the file
  print_time(ENVLogFile);

  // print the sensor values
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bme_pres);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bme_temp);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bme_humid);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.ssc_pres);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.ssc_temp);
  ENVLogFile.print(", ");
  ENVLogFile.print(ENVData.bno_temp);
  ENVLogFile.print(", ");
  ENVLogFile.println(ENVData.mcp_temp);

  ENVLogFile.flush();
}

void Balloonduino::log_pwr(File PWRLogFile){
  
  #ifndef BALLONDUINO_NO_ADS
  // print the time to the file
  print_time(PWRLogFile);
  
  // print the sensor values
  PWRLogFile.print(", ");
  PWRLogFile.print(PWRData.batt_volt);
  PWRLogFile.print(", ");
  PWRLogFile.println(PWRData.i_consump);

  PWRLogFile.flush();
  #endif
}

uint16_t Balloonduino::create_HK_pkt(){
/*  create_HK_pkt()
 * 
 *  Creates an HK packet containing the values of all the interface counters. 
 *  Packet data is filled into the memory passed in as the argument
 *  
 */
  // get the current time from the RTC
  DateTime now;
  #ifndef BALLONDUINO_NO_RTC
  now = rtc.now();
  #else
  uint32_t mils = millis();
  now = makeTime(numberOfSeconds(mils), numberOfMinutes(mils), numberOfHours(mils), elapsedDays(mils), 0U, 0U )
  #endif
  
  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // add length of primary header
  payloadSize += sizeof(CCSDS_PriHdr_t);

  // Populate primary header fields:
  setAPID(OutPktBuf, HK_STAT_APID);
  setSecHdrFlg(OutPktBuf, 1);
  setPacketType(OutPktBuf, 0);
  setVer(OutPktBuf, 0);
  setSeqCtr(OutPktBuf, 0);
  setSeqFlg(OutPktBuf, 0);

  // add length of secondary header
  payloadSize += sizeof(CCSDS_TlmSecHdr_t);

  // Populate the secondary header fields:
  setTlmTimeSec(OutPktBuf, now.unixtime()/1000L);
  setTlmTimeSubSec(OutPktBuf, now.unixtime() % 1000L);

  // Add counter values to the pkt
  payloadSize = addIntToTlm(IntCtr.CmdExeCtr, OutPktBuf, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(IntCtr.CmdRejCtr, OutPktBuf, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(IntCtr.XbeeRcvdByteCtr, OutPktBuf, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(IntCtr.XbeeSentByteCtr, OutPktBuf, payloadSize); // Add counter of sent packets to message
  payloadSize = addIntToTlm(millis()/1000L, OutPktBuf, payloadSize); // Timer

  // fill the length field
  setPacketLength(OutPktBuf, payloadSize);

  return payloadSize;

}

void Balloonduino::begin(){
  debug_serial.begin(250000);
  xbee_serial.begin(9600);
  
  debug_serial.println("GoGoGadget Camera payload!");

  //// BNO
  #ifndef BALLONDUINO_NO_BNO
  if(!bno.begin()){
    debug_serial.println("BNO055 NOT detected.");
  }
  else{
    debug_serial.println("BNO055 detected!");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  #endif
  
  //// MCP9808
  #ifndef BALLONDUINO_NO_MCP
  if (!tempsensor.begin(0x18)) {
    debug_serial.println("MCP9808 NOT detected.");
  }
  else{
    debug_serial.println("MCP9808 detected!");
  }
  #endif

  //// RTC
  /* The RTC is used so that the log files contain timestamps. If the RTC
   *  is not running (because no battery is inserted) the RTC will be initalized
   *  to the time that this sketch was compiled at.
   */
  #ifndef BALLONDUINO_NO_RTC
  if (! rtc.begin()) {
    debug_serial.println("DS1308 NOT detected.");
  }
  else{
    debug_serial.println("DS1308 detected!");
  }

  if (! rtc.isrunning()) {
    debug_serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  //// SoftRTC (for subsecond precision)
  softrtc.begin(rtc.now());  // Initialize SoftRTC to the current time
  start_millis = millis();  // get the current millisecond count
  #endif
  
  //// Init BME
  #ifndef BALLONDUINO_NO_BME
  if (!bme.begin(0x76)) {
    debug_serial.println("BME280 NOT detected.");
  }
  else{
    debug_serial.println("BME280 detected!");
  }
  #endif

  //// Init ADS
  #ifndef BALLONDUINO_NO_ADS
  ads.begin();
  ads.setGain(GAIN_ONE);
  debug_serial.println("Initialized ADS1015");
  #endif

  //// Init SD card
  SPI.begin();
  pinMode(53,OUTPUT);
  if (!SD.begin(53)) {
    debug_serial.println("SD Card NOT detected.");
  }
  else{
    debug_serial.println("SD Card detected!");
  }

  // xbee
  debug_serial.println("Beginning xbee init");
  
  int xbeeStatus = InitXBee(XBEE_ADDR, XBEE_ID, xbee_serial, false);
  if(!xbeeStatus) {
    debug_serial.println("XBee Initialized!");
  } else {
    debug_serial.print("XBee Failed to Initialize with Error Code: ");
    debug_serial.println(xbeeStatus);
  }
  
  //// Init SSC
  //  set min / max reading and pressure, see datasheet for the values for your 
  //  sensor
  #ifndef BALLONDUINO_NO_SSC
  ssc.setMinRaw(0);
  ssc.setMaxRaw(16383);
  ssc.setMinPressure(0.0);
  ssc.setMaxPressure(30);
  
  //  start the sensor
  debug_serial.print("SSC start: ");
  debug_serial.println(ssc.start());
  #endif
  

}

uint16_t Balloonduino::create_ENV_pkt(){
/*  create_ENV_pkt()
 * 
 *  Creates an HK packet containing the values of all the interface counters. 
 *  Packet data is filled into the memory passed in as the argument
 *  
 */
  // get the current time from the RTC
  DateTime now;
  #ifndef BALLONDUINO_NO_RTC
  now = rtc.now();
  #else
  uint32_t mils = millis();
  now = makeTime(numberOfSeconds(mils), numberOfMinutes(mils), numberOfHours(mils), elapsedDays(mils), 0U, 0U )
  #endif

  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // add length of primary header
  payloadSize += sizeof(CCSDS_PriHdr_t);

  // Populate primary header fields:
  setAPID(OutPktBuf, ENV_STAT_APID);
  setSecHdrFlg(OutPktBuf, 1);
  setPacketType(OutPktBuf, 0);
  setVer(OutPktBuf, 0);
  setSeqCtr(OutPktBuf, 0);
  setSeqFlg(OutPktBuf, 0);

  // add length of secondary header
  payloadSize += sizeof(CCSDS_TlmSecHdr_t);

  // Populate the secondary header fields:
  setTlmTimeSec(OutPktBuf, now.unixtime()/1000L);
  setTlmTimeSubSec(OutPktBuf, now.unixtime() % 1000L);

  // Add counter values to the pkt
  payloadSize = addFloatToTlm(ENVData.bme_pres, OutPktBuf, payloadSize); // Add bme pressure to message [Float]
  payloadSize = addFloatToTlm(ENVData.bme_temp, OutPktBuf, payloadSize); // Add bme temperature to message [Float]
  payloadSize = addFloatToTlm(ENVData.bme_humid, OutPktBuf, payloadSize); // Add bme humidity to message [Float]
  payloadSize = addFloatToTlm(ENVData.ssc_pres, OutPktBuf, payloadSize); // Add ssc pressure to message [Float]
  payloadSize = addFloatToTlm(ENVData.ssc_temp, OutPktBuf, payloadSize); // Add ssc temperature to messsage [Float]
  payloadSize = addFloatToTlm(ENVData.bno_temp, OutPktBuf, payloadSize); // Add bno temperature to message [Float]
  payloadSize = addFloatToTlm(ENVData.mcp_temp, OutPktBuf, payloadSize); // Add mcp temperature to message [Float]
  
  // fill the length field
  setPacketLength(OutPktBuf, payloadSize);

  return payloadSize;

}

uint16_t Balloonduino::create_PWR_pkt(){
/*  create_ENV_pkt()
 * 
 *  Creates an HK packet containing the values of all the interface counters. 
 *  Packet data is filled into the memory passed in as the argument
 *  
 */
  // get the current time from the RTC
  DateTime now;
  #ifndef BALLONDUINO_NO_RTC
  now = rtc.now();
  #else
  uint32_t mils = millis();
  now = makeTime(numberOfSeconds(mils), numberOfMinutes(mils), numberOfHours(mils), elapsedDays(mils), 0U, 0U )
  #endif
  
  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // add length of primary header
  payloadSize += sizeof(CCSDS_PriHdr_t);

  // Populate primary header fields:
  setAPID(OutPktBuf, PWR_STAT_APID);
  setSecHdrFlg(OutPktBuf, 1);
  setPacketType(OutPktBuf, 0);
  setVer(OutPktBuf, 0);
  setSeqCtr(OutPktBuf, 0);
  setSeqFlg(OutPktBuf, 0);

  // add length of secondary header
  payloadSize += sizeof(CCSDS_TlmSecHdr_t);

  // Populate the secondary header fields:
  setTlmTimeSec(OutPktBuf, now.unixtime()/1000L);
  setTlmTimeSubSec(OutPktBuf, now.unixtime() % 1000L);

  // Add counter values to the pkt
  payloadSize = addFloatToTlm(PWRData.batt_volt, OutPktBuf, payloadSize); // Add battery voltage to message [Float]
  payloadSize = addFloatToTlm(PWRData.i_consump, OutPktBuf, payloadSize); // Add current consumption to message [Float]

  // fill the length field
  setPacketLength(OutPktBuf, payloadSize);

  return payloadSize;

}

uint16_t Balloonduino::create_IMU_pkt(){
/*  create_IMU_pkt()
 * 
 *  Creates an HK packet containing the values of all the interface counters. 
 *  Packet data is filled into the memory passed in as the argument
 *  
 */
  // get the current time from the RTC
  DateTime now;
  #ifndef BALLONDUINO_NO_RTC
  now = rtc.now();
  #else
  uint32_t mils = millis();
  now = makeTime(numberOfSeconds(mils), numberOfMinutes(mils), numberOfHours(mils), elapsedDays(mils), 0U, 0U )
  #endif
  
  // initalize counter to record length of packet
  uint16_t payloadSize = 0;

  // add length of primary header
  payloadSize += sizeof(CCSDS_PriHdr_t);

  // Populate primary header fields:
  setAPID(OutPktBuf, IMU_STAT_APID);
  setSecHdrFlg(OutPktBuf, 1);
  setPacketType(OutPktBuf, 0);
  setVer(OutPktBuf, 0);
  setSeqCtr(OutPktBuf, 0);
  setSeqFlg(OutPktBuf, 0);

  // add length of secondary header
  payloadSize += sizeof(CCSDS_TlmSecHdr_t);

  // Populate the secondary header fields:
  setTlmTimeSec(OutPktBuf, now.unixtime()/1000L);
  setTlmTimeSubSec(OutPktBuf, now.unixtime() % 1000L);

  // Add counter values to the pkt
  payloadSize = addIntToTlm(IMUData.system_cal, OutPktBuf, payloadSize); // Add system cal status to message [uint8_t]
  payloadSize = addIntToTlm(IMUData.accel_cal, OutPktBuf, payloadSize); // Add accelerometer cal status to message [uint8_t]
  payloadSize = addIntToTlm(IMUData.gyro_cal, OutPktBuf, payloadSize); // Add gyro cal status to message [uint8_t]
  payloadSize = addIntToTlm(IMUData.mag_cal, OutPktBuf, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
  payloadSize = addFloatToTlm(IMUData.accel_x, OutPktBuf, payloadSize); // Add battery accelerometer x to message [Float]
  payloadSize = addFloatToTlm(IMUData.accel_y, OutPktBuf, payloadSize); // Add battery accelerometer y to message [Float]
  payloadSize = addFloatToTlm(IMUData.accel_z, OutPktBuf, payloadSize); // Add battery accelerometer z to message [Float]
  payloadSize = addFloatToTlm(IMUData.gyro_x, OutPktBuf, payloadSize); // Add battery accelerometer x to message [Float]
  payloadSize = addFloatToTlm(IMUData.gyro_y, OutPktBuf, payloadSize); // Add battery accelerometer y to message [Float]
  payloadSize = addFloatToTlm(IMUData.gyro_z, OutPktBuf, payloadSize); // Add battery accelerometer z to message [Float]
  payloadSize = addFloatToTlm(IMUData.mag_x, OutPktBuf, payloadSize); // Add battery accelerometer x to message [Float]
  payloadSize = addFloatToTlm(IMUData.mag_y, OutPktBuf, payloadSize); // Add battery accelerometer y to message [Float]
  payloadSize = addFloatToTlm(IMUData.mag_z, OutPktBuf, payloadSize); // Add battery accelerometer z to message [Float]

  // fill the length field
  setPacketLength(OutPktBuf, payloadSize);
  
  return payloadSize;

}

void Balloonduino::command_response(uint8_t data[], uint8_t data_len, File file) {
  /*  command_response()
   * 
   *  given an array of data (presumably containing a CCSDS packet), check if the
   *  packet is a CAMERA command packet, and if so, process it
   */

  debug_serial.print("Rcvd: ");
  for (int i =0; i<8;i++){
    debug_serial.print(data[i],HEX);
    debug_serial.print(", ");
  }
  debug_serial.println();

  // get the APID (the field which identifies the type of packet)
  uint16_t _APID = getAPID(data);
    
  // check if the data is a command packet with the Camera command APID
  if(getPacketType(data) && _APID == CMD_APID){

    uint8_t FcnCode = getCmdFunctionCode(data);
    uint16_t pktLength = 0;
    uint8_t Pkt_Buff[100];
    uint8_t destAddr = 0;

    // respond to the command depending on what type of command it is
    switch(FcnCode){

      // NoOp Cmd
      case NOOP_FCNCODE:
        // No action other than to increment the interface counters
        
        debug_serial.println("Received NoOp Cmd");

        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;
        
      // HK_Req
      case HKREQ_FCNCODE:
        // Requests that an HK packet be sent to the specified xbee address
        /*  Command format:
         *   CCSDS Command Header (8 bytes)
         *   Xbee address (1 byte)
         */
        
        debug_serial.println("Received HK_Req Cmd");

        // extract the desintation address from the command
        extractFromTlm(destAddr, data, 8);
        
        // create a HK pkt
        pktLength = this->create_HK_pkt();

        // send the HK packet via xbee and log it
        xbee_send_and_log(destAddr, Pkt_Buff, pktLength, file);
          
        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;

      // ResetCtr
      case RESETCTR_FCNCODE:
        // Requests that an HK packet be sent to the specified xbee address
        /*  Command format:
         *   CCSDS Command Header (8 bytes)
         *   Xbee address (1 byte)
         */
        
        debug_serial.println("Received ResetCtr Cmd");
        
        IntCtr.CmdExeCtr = 0;
        IntCtr.CmdRejCtr = 0;
        IntCtr.XbeeRcvdByteCtr = 0;
        IntCtr.XbeeSentByteCtr = 0;
        
        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;
                
      // ENV_Req
      case REQENV_FCNCODE:
        // Requests that an HK packet be sent to the specified xbee address
        /*  Command format:
         *   CCSDS Command Header (8 bytes)
         *   Xbee address (1 byte)
         */
        
        debug_serial.println("Received ENV_Req Cmd");

        // extract the desintation address from the command
        extractFromTlm(destAddr, data, 8);
        
        // create a HK pkt
        pktLength = this->create_ENV_pkt();

        // send the HK packet via xbee and log it
        xbee_send_and_log(destAddr, Pkt_Buff, pktLength, file);
        
        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;
        
      // PWR_Req
      case REQPWR_FCNCODE:
        // Requests that an HK packet be sent to the specified xbee address
        /*  Command format:
         *   CCSDS Command Header (8 bytes)
         *   Xbee address (1 byte)
         */
        
        debug_serial.println("Received PWR_Req Cmd");

        // extract the desintation address from the command
        extractFromTlm(destAddr, data, 8);
        
        // create a HK pkt
        pktLength = this->create_PWR_pkt();

        // send the HK packet via xbee and log it
        xbee_send_and_log(destAddr, Pkt_Buff, pktLength, file);
        
        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;
        
      // IMU_Req
      case REQIMU_FCNCODE:
        // Requests that an HK packet be sent to the specified xbee address
        /*  Command format:
         *   CCSDS Command Header (8 bytes)
         *   Xbee address (1 byte)
         */
        
        debug_serial.println("Received IMU_Req Cmd");

        // extract the desintation address from the command
        extractFromTlm(destAddr, data, 8);
        
        // create a HK pkt
        pktLength = this->create_IMU_pkt();

        // send the HK packet via xbee and log it
        xbee_send_and_log(destAddr, Pkt_Buff, pktLength, file);
        
        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;
        
      // Reboot
      case REBOOT_FCNCODE:
        // Requests that Link reboot

        debug_serial.println("Received Reboot Cmd");

        // set the reboot timer
        wdt_enable(WDTO_1S);

        // increment the cmd executed counter
        IntCtr.CmdExeCtr++;
        break;  
        
      // unrecognized fcn code
      default:
        debug_serial.print("unrecognized fcn code ");
        debug_serial.println(FcnCode, HEX);
        
        // reject command
        IntCtr.CmdRejCtr++;
    } // end switch(FcnCode)
    
  } // if(getPacketType(data) && _APID == CAM_CMD_APID)
  else{
    debug_serial.print("Unrecognized ");
    debug_serial.print(getPacketType(data));
    debug_serial.print(" pkt apid 0x");
    debug_serial.println(_APID, HEX);
  }
}

void Balloonduino::print_time(File file){
/*  print_time()
 * 
 *  Prints the current time to the given log file
 */

  // get the current time from the RTC
  DateTime now = rtc.now();

  // print a datestamp to the file
  char buf[50];
  sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
  file.print(buf);
}

void Balloonduino::xbee_send_and_log(uint8_t dest_addr, uint8_t data[], uint8_t data_len, File file){
/*  xbee_send_and_log()
 * 
 *  Sends the given data out over the xbee and adds an entry to the xbee log file.
 *  Also updates the radio sent counter.
 */
 
  // send the data via xbee
  _sendData(dest_addr, data, data_len);

  debug_serial.print("Forwarding: ");
  for(int i = 0; i <= data_len; i++){
    debug_serial.print(data[i], HEX);
    debug_serial.print(", ");
  }
  debug_serial.println();

  // log the sent data
  logPkt(file, data, sizeof(data), 0);

  // update the xbee send ctr
  IntCtr.XbeeSentByteCtr += data_len;
}

void Balloonduino::logPkt(File file, uint8_t data[], uint8_t len, uint8_t received_flg){
/*  logPkt()
 * 
 *  Prints an entry in the given log file containing the given data. Will prepend an
 *  'S' if the data was sent or an 'R' is the data was received based on the value
 *  of the received_flg.
 */

  // if the file is open
  if (file) {

    // prepend an indicator of if the data was received or sent
    // R indicates this was received data
    if(received_flg){
      file.print("R ");
    }
    else{
      file.print("S ");
    }
    
    // Print a timestamp
    print_time(file);

   char buf[50];

    // print the data in hex
    file.print(": ");
    for(int i = 0; i < len; i++){
        sprintf(buf, "%02x, ", data[i]);
        file.print(buf);
     }
     file.println();
     
     // ensure the data gets written to the file
     file.flush();
   }
}
