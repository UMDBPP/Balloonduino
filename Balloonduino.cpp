/*
 * Balloonduino.cpp
 * balloonduino: modified Arduino Mega
 */

#include "Balloonduino.h"

// Base library type
Balloonduino::Balloonduino(void)
{
    // do nothing
}

bool Balloonduino::begin(uint8_t xbee_address = 0x0006, uint8_t cmd_xbee_address = 0x0002, uint8_t xbee_pan_id = 0x0B0B,
        bool use_xbee = true, int cmd_apid = 100, int hk_stat_apid = 101,
        int packet_counter_apid = 110, int payload_name_apid = 119,
        int env_stat_apid = 120, int imu_stat_apid = 130, int pwr_stat_apid =
                140,
        bool use_rtc = true,
        bool use_bno = true,
        bool use_mcp = true,
        bool use_bme = true,
        bool use_ads = true,
        bool use_ssc = true)
{
    /*
     * Disables watchdog timer (in case its on)
     * Initalizes all the board hardware/software including:
     *   Serial
     *   Xbee
     *   RTC
     *   SoftRTC
     *   BNO
     *   MCP
     *   BME
     *   SSC
     *   ADS
     *   SD card
     *   Log files
     */

    xbee_enable = use_xbee;
    bno_enable = use_bno;
    mcp_enable = use_mcp;
    rtc_enable = use_rtc;
    bme_enable = use_bme;
    ads_enable = use_ads;
    ssc_enable = use_ssc;

    COMMAND_APID = cmd_apid;
    STATUS_APID = hk_stat_apid;
    PACKET_COUNTER_APID = packet_counter_apid;
    PAYLOAD_NAME_APID = payload_name_apid;
    IMU_PACKET_APID = imu_stat_apid;
    ENVIRONMENTAL_PACKET_APID = env_stat_apid;
    POWER_PACKET_APID = pwr_stat_apid;

    // disable the watchdog timer immediately in case it was on because of a
    // commanded reboot
    wdt_disable();

    //// Init serial ports:
    /*  aliases defined above are used to reduce confusion about which serial
     *    is connected to what interface
     *  xbee serial is lower baud rates because the hardware
     *    defaults to that baud rate. higher baud rates need to be tested
     *    before they're used with those devices
     */
    debug_serial.begin(250000);
    xbee_serial.begin(9600);

    bool initialized = true;

    //// Init DS1308 RTC
    /* The RTC is used so that the log files contain timestamps. If the RTC
     *  is not running (because no battery is inserted) the RTC will be initalized
     *  to the time that this sketch was compiled at.
     */
    if (rtc_enable)
    {
        InitStat.rtc_start = rtc.begin();
        if (!InitStat.rtc_start)
        {
            Serial.println("RTC NOT detected.");
        }
        else
        {
            Serial.println("RTC detected!");
            InitStat.rtc_running = rtc.isrunning();
            if (!InitStat.rtc_running)
            {
                debug_serial.println("RTC is NOT running!");
                // following line sets the RTC to the date & time this sketch was compiled
                rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
                // To set the RTC with an explicit date & time:
                // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
            }
        }
    }

    //// SoftRTC (for subsecond precision)
    SoftRTC.begin(rtc.now());  // Initialize SoftRTC to the current time
    start_millis = millis();  // get the current millisecond count

    //// Init SD card
    /* The SD card is used to store all of the log files.
     */
    SPI.begin();
    pinMode(53, OUTPUT);
    InitStat.SD_detected = SD.begin(53);
    if (!InitStat.SD_detected)
    {
        debug_serial.println("SD Card NOT detected.");
    }
    else
    {
        debug_serial.println("SD Card detected!");
    }

    //// Open log files
    /* Link will log to 3 files, one for I/O to the xbee, one for I/O to the radio,
     *  and one for recording its initialization status each time it starts up.
     *  NOTE: Filenames must be shorter than 8 characters
     */

    if (xbee_enable)
    {
        xbeeLogFile = SD.open("XBEE_LOG.txt", FILE_WRITE);
        delay(10);
    }

    // for data files, write a header
    initLogFile = SD.open("INIT_LOG.txt", FILE_WRITE);
    initLogFile.println("DateTime,RTCStart,RTCRun,BNO,BME,MCP,SSC,Xbee");
    initLogFile.flush();
    delay(10);

    if (bno_enable)
    {
        IMULogFile = SD.open("IMU_LOG.txt", FILE_WRITE);
        IMULogFile.println(
                "DateTime,SystemCal[0-3],AccelCal[0-3],GyroCal[0-3],MagCal[0-3],AccelX[m/s^2],AccelY[m/s^2],AccelZ[m/s^2],GyroX[rad/s],GyroY[rad/s],GyroZ[rad/s],MagX[uT],MagY[uT],MagZ[uT]");
        IMULogFile.flush();
        delay(10);
    }

    if (ads_enable)
    {
        PWRLogFile = SD.open("PWR_LOG.txt", FILE_WRITE);
        PWRLogFile.println("DateTime,BatteryVoltage[V],CurrentConsumption[A]");
        PWRLogFile.flush();
        delay(10);
    }

    if (bme_enable || ssc_enable || bno_enable || mcp_enable)
    {
        ENVLogFile = SD.open("ENV_LOG.txt", FILE_WRITE);
        ENVLogFile.println(
                "DateTime,BMEPressure[hPa],BMETemp[degC],BMEHumidity[%],SSCPressure[PSI],SSCTemp[degC],BNOTemp[degC],MCPTemp[degC]");
        ENVLogFile.flush();
        delay(10);
    }

    //// Init XBee
    /* InitXbee() will configure the attached xbee so that it can talk to
     *   xbees which also use this library. It also handles the initalization
     *   of the adafruit xbee library
     */
    if (xbee_enable)
    {
        InitStat.xbeeStatus = ccsds_xbee.init(XBEE_ADDR, XBEE_PAN_ID,
        xbee_serial, debug_serial);
        if (!InitStat.xbeeStatus)
        {
            debug_serial.println(F("XBee Initialized!"));
        }
        else
        {
            debug_serial.print(
                    F("XBee Failed to Initialize with Error Code: "));
            debug_serial.println(InitStat.xbeeStatus);
        }

        ccsds_xbee.add_rtc(rtc);
        ccsds_xbee.start_logging(xbeeLogFile);
    }

    //// Init BNO IMU
    if (bno_enable)
    {
        InitStat.BNO_init = bno.begin();
        if (!InitStat.BNO_init)
        {
            debug_serial.println("WARNING: BNO055 initialization failure.");
            initialized = false;
        }
        else
        {
            debug_serial.println("BNO055 initialized.");
        }
        delay(1000);
        bno.setExtCrystalUse(true);
    }

    //// Init MCP9808 temperature sensor
    if (mcp_enable)
    {
        InitStat.MCP_init = tempsensor.begin(0x18);
        if (!InitStat.MCP_init)
        {
            debug_serial.println("WARNING: MCP9808 initialization failure.");
            initialized = false;
        }
        else
        {
            debug_serial.println("MCP9808 initialized.");
        }
    }

    //// Init BME environmental sensor
    if (bme_enable)
    {
        InitStat.BME_init = bme.begin(0x76);
        if (!InitStat.BME_init)
        {
            debug_serial.println("WARNING: BME280 initialization failure.");
            initialized = false;
        }
        else
        {
            debug_serial.println("BME280 initialized.");
        }
    }

    //// Init ADS power monitor
    // ADC, used for current consumption/battery voltage
    if (ads_enable)
    {
        ads.begin();
        ads.setGain(GAIN_ONE);
        debug_serial.println("Initialized ADS1015");
    }

    //// Init SSC pressure sensor
    //  set min / max reading and pressure, see datasheet for the values for your
    //  sensor
    if (ssc_enable)
    {
        ssc.setMinRaw(0);
        ssc.setMaxRaw(16383);
        ssc.setMinPressure(0.0);
        ssc.setMaxPressure(30);
        //  start the sensor
        InitStat.SSC_init = ssc.start();
        if (!InitStat.SSC_init)
        {
            debug_serial.println("SSC started ");
        }
        else
        {
            debug_serial.println("SSC failed!");
        }
    }

    //// set interface counters to zero
    CmdExeCtr = 0;
    CmdRejCtr = 0;
    XbeeRcvdByteCtr = 0;
    XbeeSentByteCtr = 0;

    // write entry in init log file
    print_time(initLogFile);
    initLogFile.print(", ");
    initLogFile.print(InitStat.rtc_start);
    initLogFile.print(", ");
    initLogFile.print(InitStat.rtc_running);
    initLogFile.print(", ");
    initLogFile.print(InitStat.BNO_init);
    initLogFile.print(", ");
    initLogFile.print(InitStat.BME_init);
    initLogFile.print(", ");
    initLogFile.print(InitStat.MCP_init);
    initLogFile.print(", ");
    initLogFile.print(InitStat.SSC_init);
    initLogFile.print(", ");
    initLogFile.print(InitStat.xbeeStatus);
    initLogFile.print(", ");
    initLogFile.print(InitStat.SD_detected);
    initLogFile.println();
    initLogFile.close();

    return initialized;
}

void Balloonduino::read_env(struct ENVData_s *ENVData)
{
    /*  read_env()
     *
     *  Reads all of the environmental sensors and stores data in
     *  a structure.
     *
     */

    //BME280
    ENVData->bme_pres = bme.readPressure() / 100.0F; // hPa
    ENVData->bme_temp = bme.readTemperature(); // degC
    ENVData->bme_humid = bme.readHumidity(); // %
    /*
     * This is causing LINK to not respond to commands... not sure why
     //  SSC
     ssc.update();
     ENVData->ssc_pres = ssc.pressure(); // PSI
     ENVData->ssc_temp = ssc.temperature(); // degC
     */
    // BNO
    ENVData->bno_temp = bno.getTemp();

    //MCP9808
    ENVData->mcp_temp = tempsensor.readTempC(); // degC
}

void Balloonduino::read_pwr(struct PWRData_s *PWRData)
{
    /*  read_pwr()
     *
     *  Reads all of the power sensors and stores data in
     *  a structure.
     *
     */
    PWRData->batt_volt = ((float) ads.readADC_SingleEnded(2)) * 0.002 * 3.0606; // V
    PWRData->i_consump = (((float) ads.readADC_SingleEnded(3)) * 0.002 - 2.5)
            * 10;
}

void Balloonduino::read_imu(struct IMUData_s *IMUData)
{
    /*  read_imu()
     *
     *  Reads all of the IMU sensors and stores data in
     *  a structure.
     *
     */
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

    // get measurements
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); // (values in uT, micro Teslas)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // (values in rps, radians per second)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // (values in m/s^2)

    // assign them into global variables
    IMUData->system_cal = system_cal;
    IMUData->accel_cal = accel_cal;
    IMUData->gyro_cal = gyro_cal;
    IMUData->mag_cal = mag_cal;
    IMUData->accel_x = accel.x();
    IMUData->accel_y = accel.y();
    IMUData->accel_z = accel.z();
    IMUData->gyro_x = gyro.x();
    IMUData->gyro_y = gyro.y();
    IMUData->gyro_z = gyro.z();
    IMUData->mag_x = mag.x();
    IMUData->mag_y = mag.y();
    IMUData->mag_z = mag.z();

}

void Balloonduino::log_imu(struct IMUData_s IMUData, File IMULogFile)
{
    /*  log_imu()
     *
     *  Writes the IMU data to a log file with a timestamp.
     *
     */

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
}

void Balloonduino::log_env(struct ENVData_s ENVData, File ENVLogFile)
{
    /*  log_env()
     *
     *  Writes the ENV data to a log file with a timestamp.
     *
     */

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

void Balloonduino::log_pwr(struct PWRData_s PWRData, File PWRLogFile)
{
    /*  log_pwr()
     *
     *  Writes the PWR data to a log file with a timestamp.
     *
     */

    // print the time to the file
    print_time(PWRLogFile);

    // print the sensor values
    PWRLogFile.print(", ");
    PWRLogFile.print(PWRData.batt_volt, 4);
    PWRLogFile.print(", ");
    PWRLogFile.println(PWRData.i_consump, 4);

    PWRLogFile.flush();
}

uint16_t Balloonduino::create_HK_payload(uint8_t Pkt_Buff[])
{
    /*  create_HK_pkt()
     *
     *  Creates an HK packet containing the values of all the interface counters.
     *  Packet data is filled into the memory passed in as the argument. This function
     *  assumes that the buffer is large enough to hold this packet.
     *
     */

    // initalize counter to record length of packet
    uint16_t payloadSize = 0;

    // Add counter values to the pkt
    payloadSize = addIntToTlm(CmdExeCtr, Pkt_Buff, payloadSize); // Add counter of sent packets to message
    payloadSize = addIntToTlm(CmdRejCtr, Pkt_Buff, payloadSize); // Add counter of sent packets to message
    payloadSize = addIntToTlm(ccsds_xbee.getRcvdByteCtr(), Pkt_Buff,
            payloadSize); // Add counter of sent packets to message
    payloadSize = addIntToTlm(ccsds_xbee.getSentByteCtr(), Pkt_Buff,
            payloadSize); // Add counter of sent packets to message
    payloadSize = addIntToTlm(millis() / 1000L, Pkt_Buff, payloadSize); // Timer

    return payloadSize;
}

uint16_t Balloonduino::create_ENV_payload(uint8_t payload[],
        struct ENVData_s ENVData)
{
    /*  create_ENV_pkt()
     *
     *  Creates an ENV packet containing the values of all environmental sensors.
     *  Packet data is filled into the memory passed in as the argument. This function
     *  assumes that the buffer is large enough to hold this packet.
     *
     */

    uint16_t payloadSize = 0;
    // Add counter values to the pkt
    payloadSize = addFloatToTlm(ENVData.bme_pres, payload, payloadSize); // Add bme pressure to message [Float]
    payloadSize = addFloatToTlm(ENVData.bme_temp, payload, payloadSize); // Add bme temperature to message [Float]
    payloadSize = addFloatToTlm(ENVData.bme_humid, payload, payloadSize); // Add bme humidity to message [Float]
    payloadSize = addFloatToTlm(ENVData.ssc_pres, payload, payloadSize); // Add ssc pressure to message [Float]
    payloadSize = addFloatToTlm(ENVData.ssc_temp, payload, payloadSize); // Add ssc temperature to messsage [Float]
    payloadSize = addFloatToTlm(ENVData.bno_temp, payload, payloadSize); // Add bno temperature to message [Float]
    payloadSize = addFloatToTlm(ENVData.mcp_temp, payload, payloadSize); // Add mcp temperature to message [Float]

    return payloadSize;

}

uint16_t Balloonduino::create_PWR_payload(uint8_t Pkt_Buff[],
        struct PWRData_s PWRData)
{
    /*  create_PWR_pkt()
     *
     *  Creates an PWR packet containing the values of all the power/battery sensors.
     *  Packet data is filled into the memory passed in as the argument. This function
     *  assumes that the buffer is large enough to hold this packet.
     *
     */

    // initalize counter to record length of packet
    uint16_t payloadSize = 0;

    // Add counter values to the pkt
    payloadSize = addFloatToTlm(PWRData.batt_volt, Pkt_Buff, payloadSize); // Add battery voltage to message [Float]
    payloadSize = addFloatToTlm(PWRData.i_consump, Pkt_Buff, payloadSize); // Add current consumption to message [Float]

    return payloadSize;

}

uint16_t Balloonduino::create_IMU_payload(uint8_t Pkt_Buff[],
        struct IMUData_s IMUData)
{
    /*  create_IMU_pkt()
     *
     *  Creates an IMU packet containing the values of all the IMU sensors.
     *  Packet data is filled into the memory passed in as the argument. This function
     *  assumes that the buffer is large enough to hold this packet.
     *
     */

    // initalize counter to record length of packet
    uint16_t payloadSize = 0;

    // Add counter values to the pkt
    payloadSize = addIntToTlm(IMUData.system_cal, Pkt_Buff, payloadSize); // Add system cal status to message [uint8_t]
    payloadSize = addIntToTlm(IMUData.accel_cal, Pkt_Buff, payloadSize); // Add accelerometer cal status to message [uint8_t]
    payloadSize = addIntToTlm(IMUData.gyro_cal, Pkt_Buff, payloadSize); // Add gyro cal status to message [uint8_t]
    payloadSize = addIntToTlm(IMUData.mag_cal, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
    payloadSize = addFloatToTlm(IMUData.accel_x, Pkt_Buff, payloadSize); // Add battery accelerometer x to message [Float]
    payloadSize = addFloatToTlm(IMUData.accel_y, Pkt_Buff, payloadSize); // Add battery accelerometer y to message [Float]
    payloadSize = addFloatToTlm(IMUData.accel_z, Pkt_Buff, payloadSize); // Add battery accelerometer z to message [Float]
    payloadSize = addFloatToTlm(IMUData.gyro_x, Pkt_Buff, payloadSize); // Add battery accelerometer x to message [Float]
    payloadSize = addFloatToTlm(IMUData.gyro_y, Pkt_Buff, payloadSize); // Add battery accelerometer y to message [Float]
    payloadSize = addFloatToTlm(IMUData.gyro_z, Pkt_Buff, payloadSize); // Add battery accelerometer z to message [Float]
    payloadSize = addFloatToTlm(IMUData.mag_x, Pkt_Buff, payloadSize); // Add battery accelerometer x to message [Float]
    payloadSize = addFloatToTlm(IMUData.mag_y, Pkt_Buff, payloadSize); // Add battery accelerometer y to message [Float]
    payloadSize = addFloatToTlm(IMUData.mag_z, Pkt_Buff, payloadSize); // Add battery accelerometer z to message [Float]

    return payloadSize;

}

uint16_t Balloonduino::create_INIT_payload(uint8_t Pkt_Buff[],
        struct InitStat_s InitStat)
{
    /*  create_IMU_pkt()
     *
     *  Creates an IMU packet containing the values of all the IMU sensors.
     *  Packet data is filled into the memory passed in as the argument. This function
     *  assumes that the buffer is large enough to hold this packet.
     *
     */

    // initalize counter to record length of packet
    uint16_t payloadSize = 0;

    // Add counter values to the pkt
    payloadSize = addIntToTlm(InitStat.xbeeStatus, Pkt_Buff, payloadSize); // Add system cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.rtc_running, Pkt_Buff, payloadSize); // Add accelerometer cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.rtc_start, Pkt_Buff, payloadSize); // Add gyro cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.BNO_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.MCP_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.BME_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.SSC_init, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]
    payloadSize = addIntToTlm(InitStat.SD_detected, Pkt_Buff, payloadSize); // Add mnagnetomter cal status to message [uint8_t]

    return payloadSize;

}

void Balloonduino::command_response(uint8_t data[], uint8_t data_len, File file)
{
    /*  command_response()
     *
     *  given an array of data (presumably containing a CCSDS packet), check if:
     *    the packet is a command packet
     *    the APID is the LINK command packet APID
     *    the checksum in the header is correct
     *  if so, process it
     *  otherwise, reject it
     */

    // get the APID (the field which identifies the type of packet)
    uint16_t _APID = getAPID(data);

    if (_APID != COMMAND_APID)
    {
        debug_serial.print("Unrecognized apid 0x");
        debug_serial.println(_APID, HEX);
        return;
    }
    if (!getPacketType(data))
    {
        debug_serial.print("Not a command packet");
        return;
    }

    // validate command checksum
    /*if (!validateChecksum(data))
    {
     Serial.println("Command checksum doesn't validate");
     CmdRejCtr++;
     return;
     }*/

    uint8_t destAddr = 0;
    uint16_t pktLength = 0;
    uint8_t payloadLength = 0;
    uint8_t Pkt_Buff[100];
    uint8_t payload_buff[100];

    // respond to the command depending on what type of command it is
    switch (getCmdFunctionCode(data))
    {

        // NoOp Cmd
        case COMMAND_NOOP:
        {
            // No action other than to increment the interface counters

            debug_serial.println("Received NoOp Cmd");

            // increment the cmd executed counter
            CmdExeCtr++;
            break;
        }
            // REQ_Name Cmd
        case REQUEST_PAYLOAD_NAME:
        {
            /*
             * This command requests that a packet containing the payload's
             * name be sent to a specific xbee. The format of the command is:
             *   CCSDS Command Header (8 bytes)
             *   Xbee address (uint8_t)
             * There is one parameter associated with this command, the address
             * of the xbee to send the Name message to. The format of the Name message
             * which is sent out is:
             *   CCSDS Telemetry Header (12 bytes)
             *   Payload Name (string, 8 bytes)
             */
            debug_serial.print("Received Name Cmd to addr ");

            // define variables to process the command
            uint8_t NamedestAddr = 0;
            uint16_t pktLength = 0;
            uint8_t payloadLength = 0;
            int success_flg = 0;

            // define buffer to create the response in
            uint8_t Name_Payload_Buff[PKT_MAX_LEN];

            // extract the desintation address from the command
            extractFromTlm(NamedestAddr, data, 8);
            debug_serial.println(NamedestAddr);

            // Use sprintf to pad/trim the string if the payload name isn't
            // exactly 8 characters
            char payloadname[8];
            sprintf(payloadname, "%8.8s", PAYLOAD_NAME);

            // print the name to debug
            debug_serial.println(payloadname);

            // add the information to the buffer
            payloadLength = addStrToTlm(payloadname, Name_Payload_Buff,
                    payloadLength);

            // send the telemetry message by adding the buffer to the header
            success_flg = ccsds_xbee.sendTlmMsg(NamedestAddr, PAYLOAD_NAME_APID,
                    Name_Payload_Buff, payloadLength);

            if (success_flg > 0)
            {
                // increment the cmd executed counter
                CmdExeCtr++;
            }
            else
            {
                CmdRejCtr++;
            }
            break;
        }
            // REQ_HK
        case REQUEST_PACKET_COUNTERS:
        {
            // Requests that an HK packet be sent to the ground
            debug_serial.print("Received HKReq Cmd to addr ");
            /*  Command format:
             *   CCSDS Command Header (8 bytes)
             *   Xbee address (1 byte) (or 0 if Gnd)
             */
            uint8_t destAddr = 0;
            uint16_t pktLength = 0;
            uint8_t payloadLength = 0;

            // extract the desintation address from the command
            extractFromTlm(destAddr, data, 8);
            debug_serial.println(destAddr);

            // create a HK pkt
            payloadLength = create_HK_payload(payload_buff);

            pktLength = ccsds_xbee.createTlmMsg(Pkt_Buff, PACKET_COUNTER_APID,
                    payload_buff, payloadLength);
            if (pktLength > 0)
            {

                // send the data
                send_and_log(destAddr, Pkt_Buff, pktLength);
            }

            // increment the cmd executed counter
            CmdExeCtr++;
            break;
        }
        case COMMAND_CLEAR_PACKET_COUNTERS:
        {
            // Requests that an HK packet be sent to the specified xbee address
            /*  Command format:
             *   CCSDS Command Header (8 bytes)
             *   Xbee address (1 byte)
             */

            debug_serial.println("Received ResetCtr Cmd");

            break;
        }
        case COMMAND_REBOOT:
        {
            // Requests reboot
            debug_serial.println("Received Reboot Cmd");

            // set the reboot timer
            wdt_enable (WDTO_1S);

            break;
        }
            // unrecognized fcn code
        default:
        {
            debug_serial.print("unrecognized fcn code ");
            debug_serial.println(getCmdFunctionCode(data), HEX);

            // reject command
            CmdRejCtr++;
        }
    } // end switch(FcnCode)

} // end command_response()

void Balloonduino::xbee_send_and_log(uint8_t dest_addr, uint8_t data[], uint8_t data_len, File file)
{
    /*  xbee_send_and_log()
     *
     *  Sends the given data out over the xbee and adds an entry to the xbee log file.
     */

    // send the HK packet via xbee and log it
    ccsds_xbee.sendRawData(dest_addr, data, data_len);

    debug_serial.print('Sent packet to ');
    debug_serial.println(dest_addr);
    for (int i = 0; i <= data_len; i++)
    {
        debug_serial.print(data[i], HEX);
        debug_serial.print(", ");
    }
    debug_serial.println();

    // log the sent data
    logPkt(file, data, sizeof(data), 0);

    // update the xbee send ctr
    IntCtr.XbeeSentByteCtr += data_len;
}

void Balloonduino::logPkt(File file, uint8_t data[], uint8_t len, uint8_t received_flg)
{
    /*  logPkt()
     *
     *  Prints an entry in the given log file containing the given data. Will prepend an
     *  'S' if the data was sent or an 'R' is the data was received based on the value
     *  of the received_flg.
     */

    // if the file is open
    if (file)
    {
        // prepend an indicator of if the data was received or sent
        // R indicates this was received data
        if (received_flg)
        {
            file.print("R ");
        }
        else
        {
            file.print("S ");
        }

        // Print a timestamp
        print_time(file);

        char buf[50];

        // print the data in hex
        file.print(": ");
        for (int i = 0; i < len; i++)
        {
            sprintf(buf, "%02x, ", data[i]);
            file.print(buf);
        }
        file.println();

        // ensure the data gets written to the file
        file.flush();
    }
}

void Balloonduino::print_time(File file)
{
    /*  print_time()
     *
     *  Prints the current time to the given log file
     */

    // get the current time from the RTC
    DateTime now;
    if (rtc_enable)
    {
        now = rtc.now();
    }
    else
    {
        now = SoftRTC.now();
    }

    // print a datestamp to the file
    char buf[50];
    sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    file.print(buf);
}
