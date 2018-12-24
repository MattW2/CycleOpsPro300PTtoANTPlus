/* Interface between CycleOps Wired Indoor Trainer and ANT+
Project details avaliable at https://www.blinkyme.com


Thank you to
Brody Kenrick for https://github.com/brodykenrick/ANTPlus_Arduino

Interfacing of Garmin ANT+ device (via a cheap Nordic nRF24AP UART module) to an Arduino.

Hardware
-WEMOS D1 mini, https://wiki.wemos.cc/products:d1:d1_mini
-nRF24AP2 module : https://www.aliexpress.com/item/Free-shipping-NRF24AP2-Networking-Module-8-channel-Serial-Interface-ANT-Networking/32460612988.html

The connector on nRF24AP2 board is (looking from the front, pin 1 is marked []):
[]GND(=VSS) | VDD(=3.3 volts)
UART_TX   | UART_RX
!(SUSP)   | SLEEP
RTS       | !(RESET)

Wiring to the WEMOS D1 mini can be found in the PCB documention.
*/

#include <Arduino.h>

//#define NDEBUG
//#define ANTPLUS_DEBUG //output the ANTPLUS Packets
//#define ANTPLUS_MSG_STR_DECODE // decode ANTPLUS Packets

#define __ASSERT_USE_STDERR
#include <assert.h>

#define ANTPLUS_ON_HW_UART //using a Arduino Pro Micro with "Real" USB serial. This is on Serial, nRF24AP2 will be on Serial

#if !defined(ANTPLUS_ON_HW_UART)
#include <SoftwareSerial.h>
#endif

#include <ANTPlus.h>

#define USE_SERIAL_CONSOLE //!<Use the hardware serial as the console. This needs to be off if using hardware serial for driving the ANT+ module.

#if defined(NDEBUG)
#undef USE_SERIAL_CONSOLE
#endif

//Telnet debuging
#define USE_TELNET_CONSOLE
#define ESP8266

#ifdef USE_TELNET_CONSOLE
#if defined (ESP8266)
#define USE_MDNS true // Use the MDNS ?

// Includes do ESP8266

#include <ESP8266WiFi.h>

#ifdef USE_MDNS
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#endif

#elif defined(ESP32)
//#define USE_MDNS true // Use the MDNS ? //TODO: not tested in Esp32 yet
// Includes do ESP32
#include <WiFi.h>
#ifdef USE_MDNS
#include "ESPmDNS.h"
#endif
#else
#error The board must be ESP8266 or ESP32
#endif // ESP
// Remote debug over telnet - not recommended for production, only for development
#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug
RemoteDebug Debug;
// SSID and password
const char* ssid = "CatBox";
const char* password = "wass9084586288";

// Host mDNS
#define HOST_NAME "esp_ant"

// Time
uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

// Buildin Led ON ?
boolean mLedON = false;
#endif


//Logging macros
//********************************************************************
#define SERIAL_DEBUG
#define DEBUG_LEVEL 4 //use 4 for all, 0 for basic

#if !defined(USE_SERIAL_CONSOLE)
//Disable logging under these circumstances
#undef SERIAL_DEBUG
#endif

//F() stores static strings that come into existence here in flash (makes things a bit more stable)
#ifdef SERIAL_DEBUG
#if DEBUG_LEVEL == 4
#define SERIAL_DEBUG_PRINT(x)         (Serial1.print(x))
#define SERIAL_DEBUG_PRINTLN(x)       (Serial1.println(x))
#define SERIAL_DEBUG_PRINT_F(x)       (Serial1.print(F(x)))
#define SERIAL_DEBUG_PRINTLN_F(x)     (Serial1.println(F(x)))
#define SERIAL_DEBUG_PRINT2(x,y)      (Serial1.print(x,y))
#define SERIAL_DEBUG_PRINTLN2(x,y)    (Serial1.println(x,y))
#else
#define SERIAL_DEBUG_PRINT(x)
#define SERIAL_DEBUG_PRINTLN(x)
#define SERIAL_DEBUG_PRINT_F(x)
#define SERIAL_DEBUG_PRINTLN_F(x)
#define SERIAL_DEBUG_PRINT2(x,y)
#define SERIAL_DEBUG_PRINTLN2(x,y)
#endif
#if DEBUG_LEVEL == 0
#define SERIAL_DEBUG_0_PRINT(x)       (Serial1.print(x))
#define SERIAL_DEBUG_0_PRINTLN(x)     (Serial1.println(x))
#define SERIAL_DEBUG_0_PRINT_F(x)     (Serial1.print(F(x)))
#define SERIAL_DEBUG_0_PRINTLN_F(x)   (Serial1.println(F(x)))
#define SERIAL_DEBUG_0_PRINT2(x,y)    (Serial1.print(x,y))
#define SERIAL_DEBUG_0_PRINTLN2(x,y)  (Serial1.println(x,y))
#else
#define SERIAL_DEBUG_0_PRINT(x)   
#define SERIAL_DEBUG_0_PRINTLN(x) 
#define SERIAL_DEBUG_0_PRINT_F(x)   
#define SERIAL_DEBUG_0_PRINTLN_F(x)     
#define SERIAL_DEBUG_0_PRINT2(x,y)      
#define SERIAL_DEBUG_0_PRINTLN2(x,y)
#endif
#endif

#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG_PRINT(x)
#define SERIAL_DEBUG_PRINTLN(x)
#define SERIAL_DEBUG_PRINT_F(x)
#define SERIAL_DEBUG_PRINTLN_F(x)
#define SERIAL_DEBUG_PRINT2(x,y)
#define SERIAL_DEBUG_PRINTLN2(x,y)

#define SERIAL_DEBUG_0_PRINT(x)   
#define SERIAL_DEBUG_0_PRINTLN(x) 
#define SERIAL_DEBUG_0_PRINT_F(x)   
#define SERIAL_DEBUG_0_PRINTLN_F(x)     
#define SERIAL_DEBUG_0_PRINT2(x,y)      
#define SERIAL_DEBUG_0_PRINTLN2(x,y)    
#endif

//********************************************************************

#define ANTPLUS_BAUD_RATE (9600) //!< The moduloe I am using is hardcoded to this baud rate.


//The ANT+ network keys are not allowed to be published so they are stripped from here.
//They are available in the ANT+ docs at thisisant.com
#define ANT_SENSOR_NETWORK_KEY {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}
#define ANT_GPS_NETWORK_KEY    {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1}

#if !defined( ANT_SENSOR_NETWORK_KEY ) || !defined(ANT_GPS_NETWORK_KEY)
#error "The Network Keys are missing. Better go find them by signing up at thisisant.com"
#endif

// ****************************************************************************
// ******************************  Defines for Sending Power  *****************
// ****************************************************************************
#define PAGE_SEND_DELAY_MS 250

// ****************************************************************************
// ******************************  GLOBALS  ***********************************
// ****************************************************************************

//Arduino Pro Mini pins to the nrf24AP2 modules pinouts
static const int RTS_PIN = 0; //!< RTS on the nRF24AP2 module
                 //static const int RTS_PIN_INT  = 0; //!< The interrupt equivalent of the RTS_PIN


#if !defined(ANTPLUS_ON_HW_UART)
static const int TX_PIN = 13; //Using software serial for the UART
static const int RX_PIN = 15; //Ditto
static SoftwareSerial ant_serial(TX_PIN, RX_PIN, false, 256); // RXArd, TXArd -- Arduino is opposite to nRF24AP2 module
#else
                 //Using Hardware Serial (0,1) instead
#endif

static ANTPlus        antplus = ANTPlus(RTS_PIN, 14/*SUSPEND*/, 16/*SLEEP*/, 12/*RESET*/);

static ANT_Channel fitness_channel =
{
  0, //Channel Number
  MASTER_DEVICE, //Channel_Type
  0, //Network Number
  DEVCE_TIMEOUT,
  DEVCE_TYPE_FITNESS_EQUIPMENT,
  DEVCE_SENSOR_FREQ,
  DEVCE_FITNESS_LOWEST_RATE,
  0x03, //device number MSB
  0x00, //device number LSB
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,

  0, //state_counter
};

volatile int rts_ant_received = 0; //!< ANT RTS interrupt flag see isr_rts_ant()

                   //Globals for Power measurement 
Bike_Trainer_with_Power Trainer_Data;
int testpower;
int message_step;

//Timers
unsigned long current_time_ms;
unsigned long last_time_ms;


// **************************************************************************************************
// *********************************  ISRs  *********************************************************
// **************************************************************************************************

//! Interrupt service routine to get RTS from ANT messages
void isr_rts_ant()
{
  rts_ant_received = 1;
}

// **************************************************************************************************
// ***********************************  ANT+  *******************************************************
// **************************************************************************************************

void process_packet(ANT_Packet * packet)
{
#if defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
  //This function internally uses Serial.println
  //Only use it if the console is available and if the ANTPLUS library is in debug mode
  antplus.printPacket(packet, false);
#endif //defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)

  switch (packet->msg_id)
  {
  case MESG_BROADCAST_DATA_ID:
  {
    break;
  }

  case MESG_ACKNOWLEDGED_DATA_ID:
  {
    const ANT_Broadcast * broadcast = (const ANT_Broadcast *)packet->data;
    SERIAL_DEBUG_PRINT_F("CHAN ");
    SERIAL_DEBUG_PRINT(broadcast->channel_number);
    SERIAL_DEBUG_PRINT_F(" ");
    const ANT_DataPage * dp = (const ANT_DataPage *)broadcast->data;

    //Update received data
    if (broadcast->channel_number == fitness_channel.channel_number)
    {
      fitness_channel.data_rx = true;
      //To determine the device type -- and the data pages -- check channel setups
      if (fitness_channel.device_type == DEVCE_TYPE_FITNESS_EQUIPMENT)
      {
        switch (dp->data_page_number)
        {
        case DATA_PAGE_FITNESS_BASIC_RESISTANCE: //Data Page 48 (0x30)
        {
          const ANT_Fitness_Basic_Resistance_DataPage * fitness_dp = (const ANT_Fitness_Basic_Resistance_DataPage *)dp;
          Trainer_Data.Target_Total_Resistance = (fitness_dp->total_resistance);
          SERIAL_DEBUG_PRINT_F("Fitness Page 48, Basic, Resistance = ");
          SERIAL_DEBUG_PRINTLN(Trainer_Data.Target_Total_Resistance);
#if DEBUG_LEVEL == 0
          SERIAL_DEBUG_0_PRINT_F("Fitness Page 48, Basic, Resistance = ");
          SERIAL_DEBUG_0_PRINTLN(Trainer_Data.Target_Total_Resistance);
#endif
        }
        break;

        case DATA_PAGE_FITNESS_TARGET_POWER: //Data Page 49 (0x31)
        {
          const ANT_Fitness_Target_Power_DataPage * fitness_dp = (const ANT_Fitness_Target_Power_DataPage *)dp;
          Trainer_Data.Target_Power = (unsigned int)(fitness_dp->Target_Power_MSB) << 8 | (fitness_dp->Target_Power_LSB);
          SERIAL_DEBUG_PRINT_F("Fitness Page 49, Target Power, Target Power = ");
          SERIAL_DEBUG_PRINTLN(Trainer_Data.Target_Power);
#if DEBUG_LEVEL == 0
          SERIAL_DEBUG_0_PRINT_F("Fitness Page 49, Target Power, Target Power = ");
          SERIAL_DEBUG_0_PRINTLN(Trainer_Data.Target_Power);
#endif
        }
        break;

        case DATA_PAGE_TRACK_RESISTANCE: //Data Page 51 (0x33)    
        {
          const ANT_Fitness_Track_Resistance_DataPage * fitness_dp = (const ANT_Fitness_Track_Resistance_DataPage *)dp;
          Trainer_Data.Simulated_Grade = (unsigned int)(fitness_dp->Grade_of_Simulated_Track_MSB) << 8 | (fitness_dp->Grade_of_Simulated_Track_LSB);
          Trainer_Data.Rolling_Resistance = fitness_dp->Coefficient_of_Rolling_Resistance;
          SERIAL_DEBUG_PRINT_F("Fitness Page 51, Track Resistance, Grade = ");
          SERIAL_DEBUG_PRINT(Trainer_Data.Simulated_Grade);
          SERIAL_DEBUG_PRINT_F(" Rolling Resistance = ");
          SERIAL_DEBUG_PRINTLN(Trainer_Data.Rolling_Resistance);

#if DEBUG_LEVEL == 0
          SERIAL_DEBUG_0_PRINT_F("Fitness Page 51, Track Resistance, Grade = ");
          SERIAL_DEBUG_0_PRINT(Trainer_Data.Simulated_Grade);
          SERIAL_DEBUG_0_PRINT_F(" Rolling Resistance = ");
          SERIAL_DEBUG_0_PRINTLN(Trainer_Data.Rolling_Resistance);
#endif  
        }
        break;

        default:
          SERIAL_DEBUG_PRINT_F(" Fitness DP# ");
          SERIAL_DEBUG_PRINTLN(dp->data_page_number);
          break;
        }
      }
    }
    break;
  }
  default:
    SERIAL_DEBUG_PRINTLN_F("Non-broadcast data received.");
    break;
  }
}

// **************************************************************************************************
// ************************************  Power Function  ********************************************
// **************************************************************************************************
void Update_Power(uint16_t INST_power)
{
  Trainer_Data.ANT_INST_power = INST_power;
  Trainer_Data.ANT_power = Trainer_Data.ANT_power + INST_power;
}

void Update_Cadence(uint16_t INST_cadence)
{
  Trainer_Data.ANT_icad = INST_cadence;
}

void Send_Page25()
{
  if (Trainer_Data.Event_Count < 255)
    Trainer_Data.Event_Count++;
  else
    Trainer_Data.Event_Count = 0;

  ANT_Fitness_Specific_Trainer_Data_struct Trainer_Data_Packet;
  Trainer_Data_Packet.data_page_number = DATA_PAGE_SPECIFIC_TRAINER_DATA_PAGE;
  Trainer_Data_Packet.update_event_count = Trainer_Data.Event_Count;
  Trainer_Data_Packet.instantaneous_cadence = Trainer_Data.ANT_icad;
  Trainer_Data_Packet.accumulated_power_LSB = byte(Trainer_Data.ANT_power & 0xFF);
  Trainer_Data_Packet.accumulated_power_MSB = byte((Trainer_Data.ANT_power >> 8) & 0xFF);
  Trainer_Data_Packet.instantaneous_power_LSB = byte(Trainer_Data.ANT_INST_power & 0xFF);
  Trainer_Data_Packet.instantaneous_power_MSB = byte((Trainer_Data.ANT_INST_power >> 8) & 0b00001111);
  Trainer_Data_Packet.trainer_status_bit_field = 0b0000;
  Trainer_Data_Packet.flags_bit_field = 0b0000;
  Trainer_Data_Packet.FE_state_bit_field = 0b0011;

  antplus.send(MESG_BROADCAST_DATA_ID, MESG_INVALID_ID, 9, 0,
    Trainer_Data_Packet.data_page_number, Trainer_Data_Packet.update_event_count, Trainer_Data_Packet.instantaneous_cadence,
    Trainer_Data_Packet.accumulated_power_LSB, Trainer_Data_Packet.accumulated_power_MSB, Trainer_Data_Packet.instantaneous_power_LSB,
    (Trainer_Data_Packet.instantaneous_power_MSB | (Trainer_Data_Packet.trainer_status_bit_field << 4)),
    (Trainer_Data_Packet.flags_bit_field | (Trainer_Data_Packet.FE_state_bit_field << 4)));
}

void Send_Page16()
{
  ANT_Fitness_General_FE_Data_struct FE_Data_Packet;
  FE_Data_Packet.data_page_number = GENERAL_FE_DATA_PAGE;
  FE_Data_Packet.equipment_type_bit_field = 0b00011001; //bits 0-4 = 25 = Trainer
  FE_Data_Packet.elapsed_time = (byte)(millis() / 250); //number of 0.25 seconds since start of program
  FE_Data_Packet.distance_traveled = 0xFF;
  FE_Data_Packet.speed_lsb = 0xFF;
  FE_Data_Packet.speed_msb = 0xFF;
  FE_Data_Packet.heart_rate = 0xFF;
  FE_Data_Packet.capabilities_bit_field = 0b0000; //3210 Bits:0-1 = No Heart Rate, Bit:2 = No Distance, Bit:3=Real Speed
  FE_Data_Packet.fe_state_bit_field = 0b0011; //3210 Bit:0:2 = FE State, Ready, Bit:3=Lap Toggle

  antplus.send(MESG_BROADCAST_DATA_ID, MESG_INVALID_ID, 9, 0,
    FE_Data_Packet.data_page_number, FE_Data_Packet.equipment_type_bit_field, FE_Data_Packet.elapsed_time,
    FE_Data_Packet.distance_traveled, FE_Data_Packet.speed_lsb, FE_Data_Packet.speed_msb, FE_Data_Packet.heart_rate,
    (FE_Data_Packet.capabilities_bit_field | (FE_Data_Packet.fe_state_bit_field << 4)));
}

void Send_Page80() //Common Data Page 80: Manufacturerâ€™s Information
{
  //byte:0 - Data Page Number (0x50)
  //byte:1 - Reserved - (0xFF)
  //byte:2 - Reserved - (0xFF)
  //byte:3 - HW Revision (0x01) Using 1
  //byte:4 - Manufacturer ID LSB (0xFF) Use 0x00FF for development
  //byte:5 - Manufacturer ID MSB (0x00)
  //byte:6 - Model Number LSB (0x02)
  //byte:7 - Model Number MSB (0x00)

  antplus.send(MESG_BROADCAST_DATA_ID, MESG_INVALID_ID, 9, 0, MANUFACTURES_INFORMATION_DATA_PAGE,
    0xFF, 0xFF, 0x01, 0xFF, 0x00, 0x02, 0x00);
}


void Send_Page81() //Common Page 81 (0x51) â€“ Product Information
{
  //byte:0 - Data Page Number (0x51)
  //byte:1 - Reserved - (0xFF)
  //byte:2 - SW Revision (Supplemental) - (0xFF)
  //byte:3 - SW Revision (Main) (0x01) Using 1
  //byte:4 - Serial Number (Bits 0 â€“ 7) (0x01) 
  //byte:5 - Serial Number (Bits 8 â€“ 15) (0x00)
  //byte:6 - Serial Number (Bits 16 â€“ 23) (0x00)
  //byte:7 - Serial Number (Bits 24 â€“ 31) (0x00)

  antplus.send(MESG_BROADCAST_DATA_ID, MESG_INVALID_ID, 9, 0, PRODUCT_INFORMATION_DATA_PAGE,
    0xFF, 0xFF, 0x01, 0x01, 0x00, 0x00, 0x00);
}


// **************************************************************************************************
// ************************************  Setup  *****************************************************
// **************************************************************************************************
void setup()
{
  delay(2000);
#if defined(USE_SERIAL_CONSOLE)
  Serial1.begin(115200);
#endif //defined(USE_SERIAL_CONSOLE)

  SERIAL_DEBUG_PRINTLN("ANTPlus Trainer!");
  SERIAL_DEBUG_PRINTLN_F("Setup.");
  SERIAL_DEBUG_PRINTLN_F("ANT+ Config.");

  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each valid ANT message received from the host at the chip
  attachInterrupt(digitalPinToInterrupt(RTS_PIN), isr_rts_ant, RISING);


#if defined(ANTPLUS_ON_HW_UART)
  //Using hardware UART
  Serial.begin(ANTPLUS_BAUD_RATE);
  Serial.swap();
  antplus.begin(Serial);
#else
  //Using soft serial
  ant_serial.begin(ANTPLUS_BAUD_RATE);
  antplus.begin(ant_serial);
#endif


#ifdef USE_TELNET_CONSOLE
  // Buildin led off ESP8266
  //pinMode(BUILTIN_LED, OUTPUT);
  //digitalWrite(BUILTIN_LED, LOW);

#ifdef ESP32
  // ESP32 configuration // TODO: see if it is necessary
  WiFi.enableSTA(true);
  delay(1000);
#endif

  // WiFi connection
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  int connection_counter;
  connection_counter = 0;
  while (WiFi.status() != WL_CONNECTED && connection_counter < 50) {
    connection_counter = connection_counter + 1;
    SERIAL_DEBUG_PRINTLN_F("Connection Wait..");
    delay(500);
  }

  SERIAL_DEBUG_PRINTLN_F("");
  SERIAL_DEBUG_PRINTLN_F("Connected to ");
  SERIAL_DEBUG_PRINTLN(ssid);
  SERIAL_DEBUG_PRINTLN_F("IP address: ");
  SERIAL_DEBUG_PRINTLN(WiFi.localIP());

  // Register host name in WiFi and mDNS
  String hostNameWifi = HOST_NAME;
  hostNameWifi.concat(".local");

#ifdef ESP8266 // Only for it
  WiFi.hostname(hostNameWifi);
#endif

#ifdef USE_MDNS  // Use the MDNS ?
  if (MDNS.begin(HOST_NAME)) {
    SERIAL_DEBUG_PRINTLN_F("* MDNS responder started. Hostname -> ");
    SERIAL_DEBUG_PRINTLN(HOST_NAME);
  }
  MDNS.addService("telnet", "tcp", 23);
#endif
  //Initialize the telnet server of RemoteDebug
  Debug.begin(HOST_NAME); // Initiaze the telnet server
  Debug.setResetCmdEnabled(true); // Enable the reset command
                  //Debug.showTime(true); // To show time
                  //Debug.showProfiler(true); // To show profiler - time between messages of Debug
                  // Good to "begin ...." and "end ...." messages
#endif

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config Finished.");
  SERIAL_DEBUG_PRINTLN_F("Setup Finished.");

  //Initialize Bike Trainer
  Trainer_Data.ANT_event = 0;
  Trainer_Data.Event_Count = 0;
  Trainer_Data.ANT_INST_power = 0;
  Trainer_Data.ANT_power = 0;
  Trainer_Data.ANT_icad = 0;

  testpower = 0;
  message_step = 0;
  current_time_ms = millis();
  last_time_ms = current_time_ms;

}

// **************************************************************************************************
// ************************************  Loop *******************************************************
// **************************************************************************************************

void loop()
{
  current_time_ms = millis();

  byte packet_buffer[ANT_MAX_PACKET_LEN];
  ANT_Packet * packet = (ANT_Packet *)packet_buffer;
  MESSAGE_READ ret_val = MESSAGE_READ_NONE;

  if (rts_ant_received == 1)
  {
    SERIAL_DEBUG_PRINTLN_F("Received RTS Interrupt. ");
    antplus.rTSHighAssertion();
    //Clear the ISR flag
    rts_ant_received = 0;
  }

  //Read messages until we get a none
  while ((ret_val = antplus.readPacket(packet, ANT_MAX_PACKET_LEN, 0)) != MESSAGE_READ_NONE)
  {
    if ((ret_val == MESSAGE_READ_EXPECTED) || (ret_val == MESSAGE_READ_OTHER))
    {
      SERIAL_DEBUG_PRINT_F("ReadPacket success = ");
      if ((ret_val == MESSAGE_READ_EXPECTED))
      {
        SERIAL_DEBUG_PRINTLN_F("Expected packet");
      }
      else
        if ((ret_val == MESSAGE_READ_OTHER))
        {
          SERIAL_DEBUG_PRINTLN_F("Other packet");
        }
      process_packet(packet);
    }
    else
    {
      SERIAL_DEBUG_PRINT_F("ReadPacket Error = ");
      SERIAL_DEBUG_PRINTLN(ret_val);
      if (ret_val == MESSAGE_READ_ERROR_MISSING_SYNC)
      {
        //Nothing -- allow a re-read to get back in sync
      }
      else
        if (ret_val == MESSAGE_READ_ERROR_BAD_CHECKSUM)
        {
          //Nothing -- fully formed package just bit errors
        }
        else
        {
          break;
        }
    }
  }

  if (fitness_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE)
  {
    antplus.progress_setup_channel(&fitness_channel);
    if (fitness_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_DEBUG_PRINT(fitness_channel.channel_number);
      SERIAL_DEBUG_PRINTLN_F(" - Established.");
    }
    else
      if (fitness_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
      {
        SERIAL_DEBUG_PRINT(fitness_channel.channel_number);
        SERIAL_DEBUG_PRINTLN_F(" - Progressing.");
      }
      else
      {
        SERIAL_DEBUG_PRINT(fitness_channel.channel_number);
        SERIAL_DEBUG_PRINTLN_F(" - ERROR!");
      }
  }

  //if all is good, start sending fake data
  if (fitness_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
  {
    if ((current_time_ms - last_time_ms) >= PAGE_SEND_DELAY_MS)
    {
      last_time_ms = current_time_ms;
      if (message_step == 132)
        message_step = 0;

      testpower = testpower + 1;
      Update_Power(315);
      Update_Cadence(80);

      if ((message_step % 130) == 0 || ((message_step - 1) % 130) == 0)
        Send_Page81(); //(0x50)
      else if ((message_step % 64) == 0 || ((message_step - 1) % 64) == 0)
        Send_Page80(); //(0x51)
      else if ((message_step % 3) == 0)
        Send_Page16(); //(0x10)
      else
        Send_Page25(); //(0x19)
      message_step++;
    }
  }

  //test debug
  DEBUG_V(" Loop End \n");
  // Remote debug over telnet
  Debug.handle();
  // Give a time for ESP8266
  yield();
}
