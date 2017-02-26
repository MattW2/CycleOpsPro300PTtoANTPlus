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

#include <SPI.h>
//#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Bounce2.h>

//Pins for ANT+ Device Display
#define RTS_PIN 2
#define SUSPEND 6
#define SLEEP 4
#define RESET 9

//Pins for Torque Zero Button
#define TORQUE_ZERO_PIN 8

#if defined(USE_OLCD)
  //OLED Display
  #define OLED_RESET 10 //was 8 make sure this still works
  Adafruit_SSD1306 display(OLED_RESET);
  #define LOGO16_GLCD_HEIGHT 16 
  #define LOGO16_GLCD_WIDTH  16 
  #if (SSD1306_LCDHEIGHT != 64)
    #error(F("Height incorrect, please fix Adafruit_SSD1306.h!"));
  #endif
#endif


// ****************************************************************************
// *************************  GLOBALS for Read CycelOps************************
// ****************************************************************************
static const int WHEEL_DATA = 7;
static const int CLOCK_OUT = 16;
static const double WHEEL_SLOPE = 527.2213800;
static const double WHEEL_OFFSET = -3181.8199961;
static const double GEAR_RATIO = 0.269230769;

int TORQUE_OFFSET = 512;

boolean start_reading;
boolean reset_ready;
boolean new_data_set;
boolean is_data_bit;
int bit_read_count;
int bit_delay;

int data_bits = 100;

//Timers
unsigned long current_time_us;
unsigned long last_time_us;
unsigned long delay_reset_us;
unsigned long wheel_time_now;
unsigned long wheel_time_last;

//array to store data
boolean data_input[200];
int data_start_test_bits = 38;
int data_position;
int torque;
int torque_in_lbs;
unsigned int wheel_speed;
double wheel_period; //time in us for 1 wheel rotation

// Instantiate a Bounce object for zeroing the torque
Bounce torque_zero_input = Bounce(); 


// ****************************************************************************
// *************************  GLOBALS for Power Out**********************
// ****************************************************************************

const int Mag_pickup = 3; // the number of the pushbutton pin
//int pin = 13;
volatile int state = LOW;
unsigned long time1;
unsigned long time2;
unsigned long period;


double omega;
//double RPM;
//double velocity;
double torque_kgm;
double torque_Nm;

byte ANT_event = 0;
uint16_t ANT_INST_power = 0;
uint16_t ANT_power = 0;
uint8_t ANT_icada = 0; // Instant Cadence
uint8_t ANT_icad = 0; // Corrected Cadence
double powerconst;

boolean recalc = 0;

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 30; // the debounce time; increase if the output flickers
int buttonState; // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin
boolean sent = 0;

void setup()
{
	
	//setup for CycelOp
	data_position = 0;
	start_reading = 0;
	new_data_set = 0;
	reset_ready = 0;
	wheel_time_now = 0;
	wheel_time_last = 0;
	bit_read_count = 0;

  torque = 0;
  torque_in_lbs = 0;
  ANT_icad = 0;
  ANT_INST_power = 0;

	digitalWrite(CLOCK_OUT, LOW);

	pinMode(WHEEL_DATA, INPUT);
	pinMode(CLOCK_OUT, OUTPUT);
 
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

	digitalWrite(RESET, HIGH);       // turn on pullup resistors
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
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64) 
    display.clearDisplay();   
    update_OLED();  
  #endif
    
	delay(50);
	initiate();
	delay(20);

  basicpower();
  
	//interrupt for wheel data from CycleOps
	attachInterrupt(digitalPinToInterrupt(WHEEL_DATA), find_start_of_wheel_data, RISING);

  current_time_us = micros();
  last_time_us = current_time_us;
}

void loop()
{
	int i;

	// put your main code here, to run repeatedly:
	current_time_us = micros();

	if(new_data_set){  
		start_reading = 1;   
		//Serial.print("found start = ");
		//Serial.println(current_time_us);    
		new_data_set = 0;
	}

	if (is_data_bit)
	  //bit_delay = 1765;
   bit_delay = 1760; //delay for data bits
  else
	  //bit_delay = 1525;
   bit_delay = 1535; //delay for dead bits

	if (is_data_bit && bit_read_count == 4){
		is_data_bit = 0;
		bit_read_count = 0;
	}

	if (!is_data_bit && bit_read_count == 6){
		is_data_bit = 1;
		bit_read_count =0;
	}

	if ((current_time_us - last_time_us) >= bit_delay){
		if (start_reading && data_position < data_bits){
			last_time_us = current_time_us;
			read_wheel_data();
			bit_read_count++;
		}
	}

	if (data_position == data_bits){ //time to spit out the data and reset
		start_reading = 0;
		//test if data is valid
		//valid data start with 11110000001111000000101100000011001
		if (data_input[0] && data_input[1] && data_input[2] && data_input[3] && 
				!data_input[4] && !data_input[5] && !data_input[6] && !data_input[7] && !data_input[8] && !data_input[9] &&
				data_input[10] && data_input[11] && data_input[12] && data_input[13] &&
				!data_input[14] && !data_input[15] && !data_input[16] && !data_input[17] && !data_input[18] && !data_input[19] &&
				data_input[20] && !data_input[21] && data_input[22] && data_input[23] &&
        !data_input[24] && !data_input[25] && !data_input[26] && !data_input[27] && !data_input[28] && !data_input[29] &&
        data_input[30] && data_input[31] && !data_input[32] && data_input[23]       
				){
			read_torque();
			read_speed();
			recalc = 1;


			for (i=0; i<=data_bits; i++){
			  Serial.print(data_input[i]);
			}
			Serial.print(" ");
			Serial.print(torque,DEC);
			Serial.print(" ");
			Serial.print(wheel_speed,DEC);
			Serial.print(" ");
			//Serial.print(torque_in_lbs,1);
			//Serial.print(" ");        
			Serial.print(ANT_icad,1);
			//cal_wheel_speed();
			//Serial.print(" ");
			//Serial.print(external_wheel_speed_usec,0);
			Serial.println("");


      
			data_position = 0;
			delay_reset_us = current_time_us;
			reset_ready = 1;
		}else{ //data is not valid, clear buffer
			data_position = 0;
			delay_reset_us = current_time_us;
			reset_ready = 1;
		}
	}

	if (reset_ready && ((current_time_us - delay_reset_us) > 100000)){
		reset_ready = 0;
		bit_read_count = 0;		
	}

  if (recalc == 1)
	{
		ANT_event++;
		ANT_icad = uint8_t(omega*9.549296586*GEAR_RATIO); // Cadence (RPM)
				
		ANT_INST_power = uint16_t(torque_Nm*omega); // Instant power calculated
		ANT_power += ANT_INST_power; // Incremental power calcualted

    //TEST CODE
    //ANT_icad = 50;
    //ANT_INST_power = 100;
    //ANT_power += ANT_INST_power; // Incremental power calcualted
    //TEST
    
    /*
		Serial.print(F("Torque(raw): "));
		Serial.print(torque,DEC);
		Serial.print(F("\t Wheel Speed (RAW) "));
		Serial.print(wheel_speed,DEC);
		Serial.print(F("\t omega: "));
		Serial.print(omega);
		Serial.print(F("\t RPM: ")); //REM
		Serial.print(ANT_icad); //REM
		Serial.print(F("\t Torque(in-lbs): ")); //REM
		Serial.print(torque_in_lbs); //REM
		Serial.print(F("\t Torque(N-m): ")); //REM
		Serial.print(torque_Nm); //REM
		Serial.print(F("\t Power: ")); //REM
		Serial.print(ANT_INST_power); //REM
		Serial.print(F("\t ANT Event: ")); //REM
		Serial.print(ANT_event); //REM
		Serial.print(F("\t ACC Power: ")); //REM
		Serial.println(ANT_power); //REM
    */

		basicpower(); // Main ANT tranmission of Basic Power Data and RPM
		Serial1.flush();

    #if defined(USE_OLCD)
       update_OLED();
    #endif    
		recalc = 0;
	}

  // Update the Bounce instance :
  torque_zero_input.update();
  if(!torque_zero_input.read())
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
	buf[4] = checkSum(buf,4);
	ANTsend(buf,5);
}


void SetNetwork() //thisisANT.com and become an ANT+ Adopter
{
	uint8_t buf[13];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x09; // LENGTH Byte
	buf[2] = MESG_NETWORK_KEY_ID; // ID Byte 0x46
	buf[3] = 0x00; // Data Byte N (Network Number)
	
	//go to www.thisisant.com to get your key, it's free you just need to register	
	buf[4] = 0xXX; // Data Byte N (Public Network Key)
	buf[5] = 0xXX; // Data Byte N (Public Network Key)
	buf[6] = 0xXX; // Data Byte N (Public Network Key)
	buf[7] = 0xXX; // Data Byte N (Public Network Key)
	buf[8] = 0xXX; // Data Byte N (Public Network Key)
	buf[9] = 0xXX; // Data Byte N (Public Network Key)
	buf[10] = 0xXX; // Data Byte N (Public Network Key)
	buf[11] = 0xXX; // Data Byte N (Public Network Key)	
	
	buf[12] = checkSum(buf, 12);
	ANTsend(buf,13);
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
	buf[6] = checkSum(buf,6);
	ANTsend(buf,7);
}

void SetChID()
{
	uint8_t buf[9];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x05; // LENGTH Byte
	buf[2] = MESG_CHANNEL_ID_ID; // Assign Channel ID 0x51
	buf[3] = 0x00; // channel number
	buf[4] = 0x05; // Device number
	buf[5] = 0x00; // Device number
	buf[6] = 0x0B; //Device type ID
	buf[7] = 0x00; //Transmission type -CHANGED
	buf[8] = checkSum(buf, 8);
	ANTsend(buf,9);
}

void ANTsend(uint8_t buf[], int length){
	//Serial.print("ANTsend TX: ");
	for(int i = 0 ; i <= length ; i++)
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
	ANTsend(buf,6);
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
	ANTsend(buf,7);
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
	ANTsend(buf,6);
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
	ANTsend(buf,6);
}

void OpenChannel()
{
	uint8_t buf[5];
	buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
	buf[1] = 0x01; // LENGTH Byte
	buf[2] = MESG_OPEN_CHANNEL_ID; // ID Byte 0x4B
	buf[3] = 0x00;
	buf[4] = checkSum(buf, 4);
	ANTsend(buf,5);
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

void basicpower()
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
	buf[10] = byte(ANT_INST_power & 0xFF);; // Instant power LSB, Byte 6
	buf[11] = byte((ANT_INST_power >> 8) & 0xFF); // Instant power MSB, Byte 7
	buf[12] = checkSum(buf, 12);
	ANTsend(buf, 13);
}



//functions for getting data from cycleops
void read_wheel_data(){
	if(start_reading){
		data_input[data_position] = digitalRead(WHEEL_DATA);
		digitalWrite(CLOCK_OUT, HIGH);
    if(is_data_bit){
		  delayMicroseconds(200);
    }else{
      delayMicroseconds(100);
    }
		digitalWrite(CLOCK_OUT, LOW);
		data_position++;
	}
}

void find_start_of_wheel_data() {
	if (data_position ==  0 && reset_ready == 0){
		new_data_set = 1;
		last_time_us = current_time_us - 1050; //almost working value was 950
		is_data_bit = 1;  
		bit_read_count = 0;
	}
	//Serial.print("found start = ");
	//Serial.println(current_time_us);
}

void read_torque() {
	torque = 0;
	bitWrite(torque,11,data_input[40]);
	bitWrite(torque,10,data_input[41]);
	bitWrite(torque,9,data_input[42]);
	bitWrite(torque,8,data_input[43]);
	bitWrite(torque,7,data_input[50]);
	bitWrite(torque,6,data_input[51]);
	bitWrite(torque,5,data_input[52]);
	bitWrite(torque,4,data_input[53]);
	bitWrite(torque,3,data_input[60]);
	bitWrite(torque,2,data_input[61]);
	bitWrite(torque,1,data_input[62]);
	bitWrite(torque,0,data_input[63]); 

	torque_in_lbs = (double)torque - (double)TORQUE_OFFSET;
  if (torque_in_lbs < 0){
    torque_Nm = 0;
    torque_in_lbs = 0;
  }else{
    torque_Nm = (torque_in_lbs)*0.112984829; 
  }
}

void read_speed() {
	wheel_speed = 0;
	bitWrite(wheel_speed,11,data_input[70]);
	bitWrite(wheel_speed,10,data_input[71]);
	bitWrite(wheel_speed,9,data_input[72]);
	bitWrite(wheel_speed,8,data_input[73]);
	bitWrite(wheel_speed,7,data_input[80]);
	bitWrite(wheel_speed,6,data_input[81]);
	bitWrite(wheel_speed,5,data_input[82]);
	bitWrite(wheel_speed,4,data_input[83]);
	bitWrite(wheel_speed,3,data_input[90]);
	bitWrite(wheel_speed,2,data_input[91]);
	bitWrite(wheel_speed,1,data_input[92]);
	bitWrite(wheel_speed,0,data_input[93]); 

	if (wheel_speed == 4095){
		omega = 0;
	}else{	
		wheel_period = wheel_speed*WHEEL_SLOPE+WHEEL_OFFSET;
		omega = 6283185.30718/wheel_period;		
	}
}

#if defined(USE_OLCD)
  void update_OLED() {
      display.clearDisplay();
        // text display tests
      display.setTextSize(1); // printable sizes from 1 to 8; typical use is 1, 2 or 4
      display.setTextColor(WHITE);    
      display.setCursor(0,0); // begin text at this location, X,Y
      display.print("O:");
      display.print(TORQUE_OFFSET,1);
      display.setCursor(40,0); // begin text at this location, X,Y
      display.print("R:");
      display.print(torque,1);
      display.setCursor(80,0); // begin text at this location, X,Y
      display.print("T:");
      display.println(torque_in_lbs,1);
      //display.setTextColor(BLACK, WHITE); // 'inverted' text
      //display.setCursor(80,0); // begin text at this location, X,Y
      //display.print("C:");
      //display.println(ANT_icad,1);
      display.setTextSize(2);
      display.print("P:");
      display.println(ANT_INST_power,1);
      display.print("C:");
      display.println(ANT_icad,1);
      display.display();
  }    
#endif



