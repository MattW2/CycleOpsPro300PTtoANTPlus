#define UCHAR unsigned char
#define MESG_TX_SYNC ((UCHAR)0xA4)
#define MESG_SYSTEM_RESET_ID ((UCHAR)0x4A)
#define MESG_NETWORK_KEY_ID ((UCHAR)0x46)
#define MESG_ASSIGN_CHANNEL_ID ((UCHAR)0x42)
#define MESG_CHANNEL_ID_ID ((UCHAR)0x51)
#define MESG_CHANNEL_RADIO_FREQ_ID ((UCHAR)0x45)
#define MESG_CHANNEL_MESG_PERIOD_ID ((UCHAR)0x43) // Set channel period 0x43
#define MESG_RADIO_TX_POWER_ID ((UCHAR)0x47) // Set Tx Power 0x47
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID ((UCHAR)0x44) // Set Channel Search Timeout 0x44
#define MESG_OPEN_CHANNEL_ID ((UCHAR)0x4B) // ID Byte 0x4B
#define MESG_BROADCAST_DATA_ID ((UCHAR)0x4E)

//OLED Display
#define USE_OLCD
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#include <Bounce2.h>

//Pins for ANT+ Device Display
#define RTS_PIN 2
#define SUSPEND 6
#define SLEEP 4
#define RESET 9

//Pins for Torque Zero Button
#define TORQUE_ZERO_PIN 8

//Pins for Data and Debug
#define WHEEL_DATA 7
#define CLOCK_OUT 16
#define ERROR_OUT 10

//Conversion Constants
#define WHEEL_SPEED_CONVERSION 6283185.30718
#define TORQUE_INLBS_TO_NM 0.112984829
#define WHEEL_CIRCUMFERENCE_MM 2097
// #define BIKE_WHEEL_ROTATIONS_PER_MS_PER_RPM 0.00005493756314
#define BIKE_WHEEL_ROTATIONS_PER_SEC_PER_RPM 0.05493756313961
#define MPH_PER_RPM 0.257599163

#if defined(USE_OLCD)
//OLED Display
#define OLED_RESET -1
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c display; 
#endif

#define MAX_DATA_PAGE_INDEX 5


// ****************************************************************************
// *************************  GLOBALS for Read CycelOps************************
// ****************************************************************************
static const double WHEEL_SLOPE = 527.2213800;
static const double WHEEL_OFFSET = -3181.8199961;
static const double GEAR_RATIO = 0.269230769;
static const bool VALID_START_DATA[15] = {0,1,1,1,1,0,1,0,1,1,0,1,1,0,1};

int TORQUE_OFFSET = 512;

boolean start_reading;
boolean reset_ready;
volatile boolean new_data_set;
volatile boolean is_data_bit;
volatile int bit_read_count;
int bit_delay;

int data_bits = 50;

//Timers
unsigned long current_time_us;
volatile unsigned long last_time_us;
unsigned long delay_reset_us;
unsigned long wheel_time_now_ms;
unsigned long wheel_time_last_ms;

//Timers and switch for finding error bits
volatile unsigned long last_rising_edge;
volatile unsigned long last_falling_edge;
volatile boolean error_bit_found;
boolean data_being_read;

//array to store data
boolean data_input[70];
int data_position;
int torque;
int torque_in_lbs;
unsigned int wheel_speed;
unsigned int CycleOps_CheckSum;
double wheel_period; //time in us for 1 wheel rotation

// Instantiate a Bounce object for zeroing the torque
Bounce torque_zero_input = Bounce();


// ****************************************************************************
// *************************  GLOBALS for Power Out**********************
// ****************************************************************************

const int Mag_pickup = 3; // the number of the pushbutton pin
//int pin = 13;
volatile int state = LOW;
//unsigned long time1;
//unsigned long time2;
//unsigned long period;

double omega;
double torque_kgm;
double torque_Nm;

byte ANT_event = 0;
uint16_t ANT_INST_power = 0;
uint16_t ANT_power = 0;
uint8_t ANT_icad = 0; // Corrected Cadence
double powerconst;

// ****************************************************************************
// *************************  GLOBALS for Cadance Out**************************
// ****************************************************************************
uint8_t ANT_wheel_ticks = 0;
uint16_t ANT_wheel_period = 0;
uint16_t ANT_accumulated_torque = 0;
double Bike_Speed_MPH = 0;

boolean recalc = 0;

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 30; // the debounce time; increase if the output flickers
int buttonState; // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin
boolean sent = 0;
int data_page_index = 0;

void setup()
{
	//setup for CycelOp
	data_position = 0;
	start_reading = 0;
	new_data_set = 0;
	reset_ready = 0;
	//wheel_time_now = 0;
	//wheel_time_last = 0;
	bit_read_count = 0;

	torque = 0;
	torque_in_lbs = 0;
	ANT_icad = 0;
	ANT_INST_power = 0;
	CycleOps_CheckSum = 0;

  ANT_wheel_ticks = 0;
  ANT_wheel_period = 0;
  ANT_accumulated_torque = 0;
	
	last_rising_edge = 0;
	last_falling_edge = 0;
	error_bit_found = 0;
	data_being_read = 0;

  data_page_index = 0;

	digitalWrite(CLOCK_OUT, LOW);
	digitalWrite(ERROR_OUT, LOW);

	pinMode(WHEEL_DATA, INPUT);
	pinMode(CLOCK_OUT, OUTPUT);
	pinMode(ERROR_OUT, OUTPUT);

	//setup for Torque zero button
	pinMode(TORQUE_ZERO_PIN, INPUT_PULLUP);
	// After setting up the button, setup the Bounce instance :
	torque_zero_input.attach(TORQUE_ZERO_PIN);
	torque_zero_input.interval(50); // interval in ms

	//Setup for PowerOut
	pinMode(SUSPEND, OUTPUT);           // set pin to output
	pinMode(SLEEP, OUTPUT);             // set pin to output
	pinMode(RESET, OUTPUT);             // set pin to output
	pinMode(RTS_PIN, INPUT);            // set pin to input

	digitalWrite(RESET, HIGH);        // turn on pullup resistors
	digitalWrite(SUSPEND, HIGH);      // turn on pullup resistors
	digitalWrite(SLEEP, LOW);         // turn on pullup resistors

	Serial.begin(115200); //this is the USB port serial
	Serial1.begin(9600); //this is the hardware UART

	delay(1000);
	Serial1.flush();

	//reset the ANT+ module
	digitalWrite(RESET, LOW);       // turn on pullup resistors
	delay(5);
	digitalWrite(RESET, HIGH);       // turn on pullup resistors

#if defined(USE_OLCD)
	//Setup for OLCD
	// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
#if OLED_RESET >= 0
	display.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // OLED_RESET >= 0
	display.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // OLED_RESET >= 0
	// Call display.setI2cClock(frequency) to change from the default frequency.
	display.setFont(Adafruit5x7);    
	display.clear(); 

	// setup display with no data
	display.set1X();
	display.setCursor(0, 0); // begin text at this location, X,Y
	display.print(F("O:"));
	display.setCursor(40, 0); // begin text at this location, X,Y
	display.print(F("R:"));
	display.setCursor(80, 0); // begin text at this location, X,Y
	display.print(F("T:"));
	display.setCursor(0, 1); // begin text at this location, X,Y
	display.set2X();  
	display.print(F("  W:"));  
	display.setCursor(0, 4); // begin text at this location, X,Y  
	display.print(F("RPM:"));
#endif

	delay(50);
	initiate();
	delay(20);

	ant_send_basicpower();

	//interrupt for wheel data from CycleOps
	attachInterrupt(digitalPinToInterrupt(WHEEL_DATA), find_start_of_wheel_data, CHANGE);

	current_time_us = micros();
	last_time_us = current_time_us;
  
  wheel_time_now_ms = millis();
  wheel_time_last_ms = wheel_time_now_ms;
}

void loop()
{
	int i;
  unsigned long wheel_time_delta_ms;
  
	current_time_us = micros();

	if (new_data_set) {
		start_reading = 1;
		//Serial.print("found start = ");
		//Serial.println(current_time_us);
		new_data_set = 0;
	}

	if (is_data_bit)
		//bit_delay = 1765;
		bit_delay = 1875; //delay for data bits
	else
		//bit_delay = 1525;
		bit_delay = 8780; //delay for dead bits

	if (is_data_bit && bit_read_count == 4) {
		is_data_bit = 0;
		bit_read_count = 0;
	}

	if (!is_data_bit && bit_read_count == 1) {
		is_data_bit = 1;
		bit_read_count = 0;
	}

	if ((current_time_us - last_time_us) >= bit_delay) {
		if (start_reading && data_position < data_bits) {
			last_time_us = current_time_us;
			read_wheel_data();
			bit_read_count++;
		}
	}
	
	if (data_position == data_bits) { //time to spit out the data and reset
		start_reading = 0;
		//test if data is valid
		if (memcmp(data_input,VALID_START_DATA,15) == 0 && !error_bit_found) {			
			read_torque();
			read_speed();
			//read_checkSum();
			recalc = 1;
			data_position = 0;
			delay_reset_us = current_time_us;
			reset_ready = 1;
			error_bit_found = 0;
			digitalWrite(ERROR_OUT, LOW);
		} else { //data is not valid, clear buffer
			if (error_bit_found) {
				Serial.println(F("ERROR BIT FOUND"));
			}
			data_position = 0;
			delay_reset_us = current_time_us;
			reset_ready = 1;
			error_bit_found = 0;
			digitalWrite(ERROR_OUT, LOW);
		}
	}
	
	if (reset_ready && ((current_time_us - delay_reset_us) > 100000)) {
		reset_ready = 0;
		bit_read_count = 0;
		error_bit_found = 0;
	}

	if (recalc == 1)
	{
		ANT_icad = uint8_t(omega * 9.549296586 * GEAR_RATIO); // Cadence (RPM)

		ANT_INST_power = uint16_t(torque_Nm * omega); // Instant power calculation
		ANT_power += ANT_INST_power; // Incremental power calculation

    //values for wheel speed Data Page (0x11)
    wheel_time_now_ms = millis();
    wheel_time_delta_ms = wheel_time_now_ms - wheel_time_last_ms;
    wheel_time_last_ms = wheel_time_now_ms;
    
    ANT_wheel_ticks += uint8_t((wheel_time_delta_ms * ANT_icad * BIKE_WHEEL_ROTATIONS_PER_SEC_PER_RPM) / 1000);
    if (ANT_icad > 0)
    {
      ANT_wheel_period += uint16_t(1/(ANT_icad * BIKE_WHEEL_ROTATIONS_PER_SEC_PER_RPM ) * 2048);
    }
    ANT_accumulated_torque += uint16_t(torque_Nm * 32); // Accumulated torque calculation
    Bike_Speed_MPH = ANT_icad * MPH_PER_RPM;
    
	//write data to serial out
		for (i = 0; i <= data_bits; i++) {
			Serial.print(data_input[i]);
		}
		Serial.print(F(" "));
		Serial.print(torque, DEC);
		Serial.print(F(" "));
		Serial.print(wheel_speed, DEC);
		Serial.print(F(" "));
		Serial.print(ANT_icad, 1);
		Serial.print(F(" "));
		Serial.print(ANT_INST_power, 1);
		Serial.print(F(" "));
		//Serial.print(CycleOps_CheckSum, DEC);
    Serial.print(ANT_wheel_ticks, 1);
    Serial.print(F(" "));
    Serial.print(ANT_wheel_period, 1);
    Serial.print(F(" "));
    Serial.print(ANT_accumulated_torque, 1);
		Serial.println(F(""));

    if (data_page_index == 0)
    {
      ANT_event++;
      ant_send_basicpower(); // Main ANT tranmission of Basic Power Data and RPM
    }
    if (data_page_index >= 1)
    {
      ANT_event++;
      ant_send_wheel_torque(); //Main Ant transmission of wheel speed (for bike speed)
    }
    data_page_index++;
    if (data_page_index >= MAX_DATA_PAGE_INDEX)
    {
      data_page_index = 0;
    }
		Serial1.flush();

#if defined(USE_OLCD)
		update_OLED();
#endif
		recalc = 0;
	}

	// Update the Bounce instance :
	torque_zero_input.update();
	if (!torque_zero_input.read())
	{
		TORQUE_OFFSET = torque;
		update_OLED();
	}

}

UCHAR checkSum(UCHAR *data, int length)
{
	int i;
	UCHAR chksum = data[0];
	for (i = 1; i < length; i++)
	chksum ^= data[i]; // +1 since skip prefix sync code, we already counted it
	return chksum;
}

void reset ()
{
	uint8_t buf[5];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x01; // LENGTH Byte
	buf[2] = MESG_SYSTEM_RESET_ID; // 0x4A
	buf[3] = 0x00; // Data Byte N (N=LENGTH)
	buf[4] = checkSum(buf, 4);
	ANTsend(buf, 5);
}

void SetNetwork() //thisisANT.com and become an ANT+ Adopter
{
	uint8_t buf[13];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x09; // LENGTH Byte
	buf[2] = MESG_NETWORK_KEY_ID; // ID Byte 0x46
	buf[3] = 0x00; // Data Byte N (Network Number)
	buf[4] = 0xB9; // Data Byte N (Public Network Key)
	buf[5] = 0xA5; // Data Byte N (Public Network Key)
	buf[6] = 0x21; // Data Byte N (Public Network Key)
	buf[7] = 0xFB; // Data Byte N (Public Network Key)
	buf[8] = 0xBD; // Data Byte N (Public Network Key)
	buf[9] = 0x72; // Data Byte N (Public Network Key)
	buf[10] = 0xC3; // Data Byte N (Public Network Key)
	buf[11] = 0x45; // Data Byte N (Public Network Key)
	buf[12] = checkSum(buf, 12);
	ANTsend(buf, 13);
}
void assignch()
{
	uint8_t buf[7];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x03; // LENGTH Byte
	buf[2] = MESG_ASSIGN_CHANNEL_ID; // 0x42
	buf[3] = 0x00; // Channel Number
	buf[4] = 0x10; // Channel Type
	buf[5] = 0x00; // Network Number
	buf[6] = checkSum(buf, 6);
	ANTsend(buf, 7);
}

void SetChID()
{
	uint8_t buf[9];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x05; // LENGTH Byte
	buf[2] = MESG_CHANNEL_ID_ID; // Assign Channel ID 0x51
	buf[3] = 0x00; // channel number
	buf[4] = 0x06; // Device number
	buf[5] = 0x00; // Device number
	buf[6] = 0x0B; //Device type ID
	buf[7] = 0x00; //Transmission type -CHANGED
	buf[8] = checkSum(buf, 8);
	ANTsend(buf, 9);
}

void ANTsend(uint8_t buf[], int length) {
	//Serial.print("ANTsend TX: ");
	for (int i = 0 ; i <= length ; i++)
	{
		//Serial.print(buf[i], HEX);
		//Serial.print(" ");
		Serial1.write(buf[i]);
	}
	//Serial.println("");
}
void SetFreq()
{
	uint8_t buf[6];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x02; // LENGTH Byte
	buf[2] = MESG_CHANNEL_RADIO_FREQ_ID; // Set Channel RF Freq 0x45
	buf[3] = 0x00; // Channel number
	buf[4] = 0x39; // Frequency
	buf[5] = checkSum(buf, 5);
	ANTsend(buf, 6);
}

void SetPeriod()
{
	uint8_t buf[7];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x03; // LENGTH Byte
	buf[2] = MESG_CHANNEL_MESG_PERIOD_ID; // Set channel period 0x43
	buf[3] = 0x00; // Channel number
	buf[4] = 0xF6; // Messaging Period byte1
	buf[5] = 0x1f; // Messaging period byte2
	buf[6] = checkSum(buf, 6);
	ANTsend(buf, 7);
}

void SetPower()
{
	uint8_t buf[6];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x02; // LENGTH Byte
	buf[2] = MESG_RADIO_TX_POWER_ID; // Set Tx Power 0x47
	buf[3] = 0x00; // Channel Number
	buf[4] = 0x03; // Tx power
	buf[5] = checkSum(buf, 5);
	ANTsend(buf, 6);
}

void SetTimeout()
{
	uint8_t buf[6];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x02; // LENGTH Byte
	buf[2] = MESG_CHANNEL_SEARCH_TIMEOUT_ID; // Set Channel Search Timeout 0x44
	buf[3] = 0x00; // Channel number
	buf[4] = 0x1E; // Set timeout
	buf[5] = checkSum(buf, 5);
	ANTsend(buf, 6);
}

void OpenChannel()
{
	uint8_t buf[5];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x01; // LENGTH Byte
	buf[2] = MESG_OPEN_CHANNEL_ID; // ID Byte 0x4B
	buf[3] = 0x00;
	buf[4] = checkSum(buf, 4);
	ANTsend(buf, 5);
}

void initiate()
{
	SetNetwork();
	delay(100);
	assignch();
	delay(100);
	SetChID();
	delay(100);
	SetFreq();
	delay(100);
	SetPeriod();
	delay(100);
	SetPower();
	delay(100);
	SetTimeout();
	delay(100);
	OpenChannel();
	delay(100);
}

void ant_send_basicpower()
{
	uint8_t buf[13];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x09; // LENGTH Byte
	buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
	buf[3] = 0x00; // Channel number
	buf[4] = 0x10; // Standard Power-Only Message, Byte 0
	buf[5] = ANT_event; // Power Event Count, Byte 1
	buf[6] = 0xFF; // Pedal Power, Byte 2, 0xFF, Pedal Power Not Used
	buf[7] = ANT_icad; // Instant Cadence, RPM, Byte 3
	buf[8] = byte(ANT_power & 0xFF); // Accumulated power LSB, Byte 4
	buf[9] = byte((ANT_power >> 8) & 0xFF); // Accumulated power MSB, Byte 5
	buf[10] = byte(ANT_INST_power & 0xFF); // Instant power LSB, Byte 6
	buf[11] = byte((ANT_INST_power >> 8) & 0xFF); // Instant power MSB, Byte 7
	buf[12] = checkSum(buf, 12);
	ANTsend(buf, 13);
}

void ant_send_wheel_torque()
{
  uint8_t buf[13];
  buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
  buf[1] = 0x09; // LENGTH Byte
  buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
  buf[3] = 0x00; // Channel number
  buf[4] = 0x11; // 0x11 – sensor measures torque at wheel, Byte 0
  buf[5] = ANT_event; // Event counter increments with each information update, Byte 1
  buf[6] = byte(ANT_wheel_ticks); // Wheel tick count increments with each wheel revolution, Byte 2
  buf[7] = ANT_icad; // Instant Cadence, RPM, Byte 3
  buf[8] = byte(ANT_wheel_period & 0xFF); // Accumulated wheel period LSB, Byte 4
  buf[9] = byte((ANT_wheel_period >> 8) & 0xFF); // Accumulated wheel period  MSB, Byte 5
  buf[10] = byte(ANT_accumulated_torque & 0xFF); // Accumulated torque LSB, Byte 6
  buf[11] = byte((ANT_accumulated_torque >> 8) & 0xFF); // Accumulated torque MSB, Byte 7
  buf[12] = checkSum(buf, 12);
  ANTsend(buf, 13);
}

//functions for getting data from cycleops
void read_wheel_data() {
	if (start_reading) {
		data_being_read = 1;
		data_input[data_position] = digitalRead(WHEEL_DATA);
		if (!data_input[data_position]) {
			data_being_read = 0;
		}
		digitalWrite(CLOCK_OUT, HIGH);
		if (is_data_bit) {
			delayMicroseconds(200);
		} else {
			delayMicroseconds(100);
		}
		digitalWrite(CLOCK_OUT, LOW);
		data_position++;
	}
}

void find_start_of_wheel_data() {
	//looking for the start of a data packet
	if (data_position ==  0 && reset_ready == 0) {
		//check if it is an error bit
		//wait for the signal to be high for > 7200us
		if(digitalRead(WHEEL_DATA)) {
			do_rising_start_packet();
		} else {
			do_falling_start_packet();			     
		}
	} else if(start_reading == 1) { //we are in the process of reading in a data packet
		if(digitalRead(WHEEL_DATA)) { //we just found a rising edge
			do_rising_mid_packet();
		} else { 						//must be a falling edge
			do_falling_mid_packet();
		}	
	}
	//Serial.print("found start = ");
	//Serial.println(current_time_us);
}

void do_rising_start_packet() {
	last_rising_edge = micros();	
}

void do_falling_start_packet() {
	last_falling_edge = micros();
	if ((last_falling_edge - last_rising_edge) > 7200) {
		new_data_set = 1;
		last_time_us = current_time_us - 700; //without error check use 1050
		is_data_bit = 0;
		bit_read_count = 0;
	}
	data_being_read = 0;
}

void do_rising_mid_packet() {
	last_rising_edge = micros();
}

void do_falling_mid_packet() {
	last_falling_edge = micros();
	if (((last_falling_edge - last_rising_edge) < 1000) && is_data_bit && data_being_read) {
		error_bit_found = 1;
		digitalWrite(ERROR_OUT, HIGH);
	}		
	data_being_read = 0;
}

void read_torque() {
	torque = 0;
	bitWrite(torque, 11, data_input[16]);
	bitWrite(torque, 10, data_input[17]);
	bitWrite(torque, 9, data_input[18]);
	bitWrite(torque, 8, data_input[19]);
	bitWrite(torque, 7, data_input[21]);
	bitWrite(torque, 6, data_input[22]);
	bitWrite(torque, 5, data_input[23]);
	bitWrite(torque, 4, data_input[24]);
	bitWrite(torque, 3, data_input[26]);
	bitWrite(torque, 2, data_input[27]);
	bitWrite(torque, 1, data_input[28]);
	bitWrite(torque, 0, data_input[29]);
	
	torque_in_lbs = (double)torque - (double)TORQUE_OFFSET;
	if (torque_in_lbs < 0) {
		torque_Nm = 0;
		torque_in_lbs = 0;
	} else {
		torque_Nm = (torque_in_lbs) * TORQUE_INLBS_TO_NM;
	}
}

void read_speed() {
	wheel_speed = 0;
	bitWrite(wheel_speed, 11, data_input[31]);
	bitWrite(wheel_speed, 10, data_input[32]);
	bitWrite(wheel_speed, 9, data_input[33]);
	bitWrite(wheel_speed, 8, data_input[34]);
	bitWrite(wheel_speed, 7, data_input[36]);
	bitWrite(wheel_speed, 6, data_input[37]);
	bitWrite(wheel_speed, 5, data_input[38]);
	bitWrite(wheel_speed, 4, data_input[39]);
	bitWrite(wheel_speed, 3, data_input[41]);
	bitWrite(wheel_speed, 2, data_input[42]);
	bitWrite(wheel_speed, 1, data_input[43]);
	bitWrite(wheel_speed, 0, data_input[44]);

	if (wheel_speed == 4095) {
		omega = 0;
	} else {
		wheel_period = wheel_speed * WHEEL_SLOPE + WHEEL_OFFSET;
		omega = WHEEL_SPEED_CONVERSION / wheel_period;
	}
}

void read_checkSum() {
	CycleOps_CheckSum = 0;
	
	bitWrite(CycleOps_CheckSum, 15, data_input[46]);
	bitWrite(CycleOps_CheckSum, 14, data_input[47]);
	bitWrite(CycleOps_CheckSum, 13, data_input[48]);
	bitWrite(CycleOps_CheckSum, 12, data_input[49]);
	bitWrite(CycleOps_CheckSum, 11, data_input[51]);
	bitWrite(CycleOps_CheckSum, 10, data_input[52]);
	bitWrite(CycleOps_CheckSum, 9, data_input[53]);
	bitWrite(CycleOps_CheckSum, 8, data_input[54]);
	bitWrite(CycleOps_CheckSum, 7, data_input[56]);
	bitWrite(CycleOps_CheckSum, 6, data_input[57]);
	bitWrite(CycleOps_CheckSum, 5, data_input[58]);
	bitWrite(CycleOps_CheckSum, 4, data_input[59]);	
	bitWrite(CycleOps_CheckSum, 3, data_input[61]);
	bitWrite(CycleOps_CheckSum, 2, data_input[62]);
	bitWrite(CycleOps_CheckSum, 1, data_input[63]);
	bitWrite(CycleOps_CheckSum, 0, data_input[64]);
}

#if defined(USE_OLCD)
void update_OLED() {
	//char temp_string[4];
	// text display tests
	display.set1X();
	display.setCursor(0, 0); // begin text at this location, X,Y
	display.print(F("O:"));
	display.clearToEOL();
	display.print(TORQUE_OFFSET, 1);
	display.setCursor(40, 0); // begin text at this location, X,Y
	display.print(F("R:"));
	display.print(torque, 1);
	display.setCursor(80, 0); // begin text at this location, X,Y
	display.print(F("T:"));
	display.println(torque_in_lbs, 1);
	display.setCursor(0, 1); // begin text at this location, X,Y
	display.set2X();  
	display.print(F("  W:")); 
	display.clearToEOL();  
	display.println(ANT_INST_power, 1);
	display.setCursor(0, 4); // begin text at this location, X,Y  
	display.print(F("RPM:"));
	display.clearToEOL();
	display.print(ANT_icad, 1);
  display.set1X();
  display.setCursor(0, 6); // begin text at this location, X,Y
  display.print(F("S:"));
  display.clearToEOL();
  display.print(Bike_Speed_MPH, 1);
}
#endif
