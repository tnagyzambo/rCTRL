#include <Arduino_MachineControl.h>
#include <Serial.h>
#include <Wire.h>
#include <SparkFun_ADS122C04_ADC_Arduino_Library.h>
#include <ArduinoJson.h>

using namespace machinecontrol;

//Sensor setup definitions
SFE_ADS122C04 LoadCell;
SFE_ADS122C04 TankPS;
float res_divider 	= 0.28057; // Required for machine control
float reference 	= 3.3;	// Machine control reference
float CCPressureMax = 30.0;

//JSON communication setup
const int sensorJsonCapacity = JSON_OBJECT_SIZE(8); //right now we have, LC, tankps1, tankps2, ccPS0, ccPS1, thermo 1, 2, 3
StaticJsonDocument<sensorJsonCapacity> sensorJson;

unsigned char data = 99;   // for incoming serial data

void setup() {
	//Set over current behavior of all channels to latch mode:
  	digital_outputs.setLatch();

	//At startup set all channels to CLOSED
  	digital_outputs.setAll(0);

	//TODO: up serial speeds?
	Serial.begin(9600);
	while (!Serial)
    ; //Wait for user to open terminal
  	Serial.println(F("LoadCell and Pressure Sensor test"));

	//----------------------Analog input setup--------------
	analogReadResolution(16); //TODO: find information on machine control ADC
	analog_in.set0_10V();

	// ---------------------- I2C setup ----------------------------
	Wire.begin(); // Machine control does not like you defining pins
	//Wire.begin(PH_8, PH_7); // sda= PH8 /scl= PH7 (on machine control)
  	Wire.setClock(400000); //Set to Fast mode (400kbps) (ADS122C04 can do up to Fm+ 1Mbps) //Can also be done in oneliner in .begin// no idea what the max of machine control is

	//------------- Set ADC modes ----------------
	if (LoadCell.begin(0x40) == false) //Connect to the Load Cell using the defaults: Address 0x40 is the board without the direct connect connector. Defualt Wire port
	{
		Serial.println(F("Error I2C bus to the Loadcell. Check address and I2C lines. Please check wiring. Freezing."));
		while (1);
	}

	if (TankPS.begin(0x44) == false) //Connect to the Pressure sensor using the defaults: Address 0x44 is the board with the direct connector. Defaul tWire port
	{
		Serial.println(F("Error I2C bus to the Pressure Sensor. Check address and I2C lines. Please check wiring. Freezing."));
		while (1);
	}	

	// Configure ADC modes
	// TODO: set datarates as low as possible based on loop freq for lower noise
	LoadCell.configureADCmode(ADS122C04_LC_MODE, ADS122C04_DATA_RATE_600SPS); //1200SPS due to turbo enabled
	LoadCell.setLCSlopeAndOffset(1,0); // 1, 0 is slope of 1 and offset of zero. Basically returning the differential voltage vin.

	TankPS.configureADCmode(ADS122C04_PS_MODE, ADS122C04_DATA_RATE_600SPS); //1200SPS due to turbo enabled
	TankPS.setPSMaxAndOffset(5.0); //Set the pressure sensor to the correct pressure

	// Print configurations to Serial
	// Comment out if not needed
	LoadCell.enableDebugging(Serial); //Enable debug messages on Serial
	LoadCell.printADS122C04config(); //Print the configuration
	LoadCell.disableDebugging(); //Enable debug messages on Serial

	TankPS.enableDebugging(Serial); //Enable debug messages on Serial
	TankPS.printADS122C04config(); //Print the configuration
	TankPS.disableDebugging(); //Enable debug messages on Serial

	//PGA offset calibration loop
  	//Set false to skip
	if (false) {
		//Set mux to internal short
		LoadCell.setInputMultiplexer(ADS122C04_MUX_SHORTED);
		TankPS.setInputMultiplexer(ADS122C04_MUX_SHORTED);

		//Setup variables
		int samples = 100;
		double loadcell_offset = 0.0;
		double TankPS_offset = 0.0;

		for (int i = 0; i<samples; i++){ 
		loadcell_offset -= LoadCell.readLC()/samples; //Go negative as the offset is added to the output of the function
		TankPS_offset -= TankPS.readPS()/samples;
		delay(25); //Delay a bit
		}

		//Set offsets 
		//TODO: make setPGAOffset function do all of this with just a sample or time
		LoadCell.setPGAOffset(loadcell_offset);
		TankPS.setPGAOffset(TankPS_offset);

		//Set Mux back
		LoadCell.setInputMultiplexer(ADS122C04_MUX_AIN0_AIN2);
		TankPS.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS);
	}

	// ---------- Thermocouple initilization -----------
	// Code taken from thermocouple example
	// Initialize temperature probes
  	temp_probes.tc.begin();
	// Enables Thermocouples chip select
  	temp_probes.enableTC();
}

void loop() {
	

	// -----------Read Sensors-------------------
	// Read LC value
	float lc_value  = LoadCell.readLC(); //Use the read function. Units depend on calibration. The output voltage true voltage (aka raw/gain)

	// Read pressure sensor 1 and 2 (Tank sensors)
	float tankPS0 = TankPS.readPS(); //Read first pressure sensor in Bar. Default sense pin is the out+ pin (aka AIN2)
	TankPS.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); //Setting it to out- (aka AIN0)
	float tankPS1 = TankPS.readPS(); //Read second pressure sensor in Bar
	TankPS.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); //Set back to default. 

	// Read CC pressure sensors
	// TODO: create wrapper functions
	// Read code is taken directly from machine control analog example
	float raw_voltage_ch0 = analog_in.read(0);
	float voltage_ch0 = (raw_voltage_ch0 * reference) / 65535 / res_divider;
	float ccPS0 = voltage_ch0/10.0*CCPressureMax; // P = (0-10V) v/10 (normalize) * 30 bar

	delay(150); //TODO: trying to figure out why they read the same
	float raw_voltage_ch1 = analog_in.read(1);
	float voltage_ch1 = (raw_voltage_ch1 * reference) / 65535 / res_divider;
	float ccPS1 = voltage_ch1/10.0*CCPressureMax;// P = (0-10V) v/10 (normalize) * 30 bar

	// Read thermocouples
	// TODO: find out why the heck changing channels takes 150 ms????
	//Set CH0, has internal 150 ms delay
	temp_probes.selectChannel(0);
	//Take CH0 measurement
	float temp_ch0 = temp_probes.tc.readTemperature();
	//Set CH1, has internal 150 ms delay
	temp_probes.selectChannel(1);
	//Take CH1 measurement
	float temp_ch1 = temp_probes.tc.readTemperature();
	//Set CH2, has internal 150 ms delay
	temp_probes.selectChannel(2);
	//Take CH2 measurement
	float temp_ch2 = temp_probes.tc.readTemperature();

	// Writing sensor JSON object for serialization
	//Do not write objects with allocated memory. Otherwise the loop will cause a memory leak
	sensorJson["LC0"] 	= lc_value;
	sensorJson["tPS0"] 	= tankPS0;
	sensorJson["tPS1"]	= tankPS1;
	sensorJson["ccPS0"]	= ccPS0;
	sensorJson["ccPS1"]	= ccPS1;
	sensorJson["T0"]	= temp_ch0;
	sensorJson["T1"]	= temp_ch1;
	sensorJson["T2"] 	= temp_ch2;

	// Write serialized object to serial com
	//serializeJson(sensorJson, Serial); //more efficient but not human readable
	//serializeJsonPretty(sensorJson,Serial);//with line breaks for debugging
	//serializeMsgPack(sensorJson,Serial); //Most efficient method but not technically a json anymore
	
	// Print statements to debug values
	if (true)
	{
		Serial.print(F("Loadcell: "));
		Serial.print(lc_value, 3);
		Serial.print(F("\t tPS0: "));
		Serial.print(tankPS0, 3); 
		Serial.print(F("Bar\t tPS1: "));
		Serial.print(tankPS1,3);
		Serial.print(F("Bar\t ccPS0: "));
		Serial.print(ccPS0,3);
		Serial.print(F("Bar\t ccPS1: "));
		Serial.print(ccPS1,3);
		Serial.print(F("Bar\t T 0,1,2: "));
		Serial.print(temp_ch0,2);
		Serial.print(F(",\t"));
		Serial.print(temp_ch1,2);
		Serial.print(F(",\t"));
		Serial.print(temp_ch2,2);
		Serial.print(F("°C\n"));
	}

	if (Serial.available() > 0) {
		data = (unsigned char)Serial.read();

		switch(data) {
			case 1:
				digital_outputs.set(0, LOW);
				break;
			case 2:
				digital_outputs.set(0, HIGH);
				break;
			case 3:
				digitalWrite(LEDR, HIGH);
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDB, LOW);
				break;
			default:
				break;
		}
	}
	// Serial.println("{\"configurations\":[{\"name\":\"Remote launch\",\"type\":\"lldb\",\"request\":\"launch\",\"program\":\"${workspaceFolder}/ros2/build/rstate/rstate_node\",\"initCommands\":[\"platform select remote-linux\",\"platform connect connect:localhost:3000\",\"settings set target.inherit-env true\"]}]}");
	delay(1);
}

