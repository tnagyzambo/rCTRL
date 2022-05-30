#include <ArduinoJson.h>
#include <Arduino_MachineControl.h>
#include <Serial.h>
#include <SparkFun_ADS122C04_ADC_Arduino_Library.h>
#include <Wire.h>
#include <valve.hpp>

using namespace machinecontrol;

// Loop globals
// We are going to setup two different loop rates.
// One for the highspeed items like tank and CC PS
// One for low speed items that require additional time for the mux switching
// First and foremost the termocouples as by default have a 150ms delay call when you switch the mux.
// The custom select channel function does not have this delay and gives no time for the ADC to settle!
// The LS loop period should be low enough that you read one sensor at a time.
// You then switch the mux and let the ADC settle before the next read on the next sensor
const unsigned long HS_PERIOD = 100;  // in ms. Hz = 1000/period
const unsigned long LS_PERIOD = 100; // Lets see if 100 ms works
// Now we define the working variable that will be updated in the loop
unsigned long CURRENT_TIME, HS_PREVIOUS, LS_PREVIOUS;

//------------ Sensor setup definitions------------
float res_divider = 0.28057; // Required for machine control
float reference = 3.3;       // Machine control reference
float CCPressureMax = 30.0;
float tankPressureMax = 30.0;
float lc_slope = 1.0;
// Sensor variables
float lc0, tankPS0, ccPS0, temp_ch0, temp_ch1, temp_ch2;
int temp_channel_select = 0;
// Sensor functions
float analog_to_pressure_CC(const float analog_read);
float analog_to_pressure_tank(const float analog_read);
float analog_to_pressure_lc(const float analog_read);

// Valve definition
auto mv1 = std::make_shared<NormClosed>(NormClosed(0));
auto mv2 = std::make_shared<NormClosed>(NormClosed(1));
auto pv = std::make_shared<NormClosed>(NormClosed(2));
auto bv = std::make_shared<NormOpen>(NormOpen(3));
auto pyro = std::make_shared<NormClosed>(NormClosed(4));
auto datavalve = std::make_shared<DataBoi>(DataBoi(5));

// Sequence decleration
autoSequence testSequence;
bool inSequence = false;

// JSON communication setup
// TODO: investigate having a lowspeed and a highspeed JSON packet to avoid sending data we know all the time.
// https://arduinojson.org/v6/how-to/reuse-a-json-document/
// Larger packets might allow us to reach higher data rates and lower the Serial usage.

const int sensorJsonCapacity = JSON_OBJECT_SIZE(12); //right now we have, LC, tankps1, tankps2, ccPS0, ccPS1, thermo 1, 2, 3, and 4 valves

unsigned char data = 99; // for incoming serial data

void setup() {

	//Sequence decleration
	testSequence.addEvent(9500,datavalve,openValve);
	testSequence.addEvent(10000,mv1,openValve);
	// testSequence.addEvent(10000,mv2,openValve);
	// testSequence.addEvent(9600,pyro,openValve);
	// testSequence.addEvent(10100,pyro,closeValve);
	testSequence.addEvent(12000,mv1,closeValve);
	// testSequence.addEvent(12000,mv2,closeValve);
	testSequence.addEvent(12000,pv,closeValve);
	testSequence.addEvent(12500,datavalve,closeValve);

	//Set over current behavior of all channels to latch mode:
  	digital_outputs.setLatch();

	//At startup set all channels to CLOSED
  	digital_outputs.setAll(0);

    // TODO: Up Serial speed
    Serial.begin(9600);
    while (!Serial)
        ; // Wait for user to open terminal
    // Serial.println(F("LoadCell and Pressure Sensor test"));

    // -------------------- Initialize variables for looping -------------------
    // Set the previous times to now so they aren't null during first comparison
    HS_PREVIOUS = millis();
    LS_PREVIOUS = HS_PREVIOUS;

    //----------------------Analog input setup--------------
    analogReadResolution(16);
    analog_in.set0_10V();

    // ---------- Thermocouple initilization -----------
    // Code taken from thermocouple example
    // Initialize temperature probes
    temp_probes.tc.begin();
    // Enables Thermocouples chip select
    temp_probes.enableTC();
    // We need to select the first channel before we start so that the loop can get going nicely
    temp_probes.selectChannel(temp_channel_select); // We use the delay(150) to let ADC settle before main loop
}

void loop() {
    if (Serial.available() > 0) {
        data = (unsigned char)Serial.read();

		switch(data) {
			case 49:
				pyro->open();
				break;
			case 50:
				pyro->close();
				break;
			case 1:
				// MV1 open
				mv1->open();
				break;
			case 2:
				// MV1 close
				mv1->close();
				break;
			case 3:
				// MV2 open
				mv2->open();
				break;
			case 4:
				// MV2 close
				mv2->close();
				break;
			case 5:
				// PV open
				pv->open();
				break;
			case 6:
				// PV close
				pv->close();
				break;
			case 7:
				// BC open
				bv->open();
				break;
			case 8:
				// BV close
				bv->close();
				break;
			case 9:
				// Run autosequnce
				inSequence = true;
				break;
			case 10:
				// Stop autosequence
				inSequence = false;
				break;
			case 11:
				// Reset sequence
				testSequence.resetSequence();
				break;
			default:
				break;
		}
	}

	CURRENT_TIME = millis(); // latest loop time
	// Call auto sequence if in sequence
	if (inSequence) {
		inSequence = testSequence.runSequence(CURRENT_TIME);
		//Put digital pin to high
		//else low
	}

    // -----------Read Sensors-------------------
    // LS loop
    if (CURRENT_TIME - LS_PREVIOUS >= LS_PERIOD) {
        // Read thermocouples
        // The thermocouple mux and ADC is very slow to settle. Default select channel has a delay of 150ms
        // Instead we use a custom no delay function and update one thermocouple each time
        // This way we don't introduce a blocking moment in the code
        // TODO: Find out fastest acceptable rate

        // ENSURE LS LOOP PROVIDES SUFFICIENT SETTLING TIME OR ELSE READING WILL BE BAD!!
        switch (temp_channel_select) {
        case 0:
            temp_ch0 = temp_probes.tc.readTemperature();
            temp_probes.selectChannelNoDelay(1);
            temp_channel_select = 1;
            break;
        case 1:
            temp_ch1 = temp_probes.tc.readTemperature();
            temp_probes.selectChannelNoDelay(2);
            temp_channel_select = 2;
            break;
        case 2:
            temp_ch2 = temp_probes.tc.readTemperature();
            temp_probes.selectChannelNoDelay(0);
            temp_channel_select = 0;
            break;
        default:
            break;
        }

        // Update LS_PREVIOUS
        LS_PREVIOUS = millis();
    }
	// HS loop
	if (CURRENT_TIME - HS_PREVIOUS >= HS_PERIOD)
	{
		StaticJsonDocument<sensorJsonCapacity> sensorJson;
		// readPS() and readLC have a small delay in it incase DRDY is not ready. It won't go longer then ADS122C04_CONVERSION_TIMEOUT
		// It should never be delayed as the mode is set to continuous

        // Read CC pressure sensors
        // Read raw voltage and then convert it to bar and save
        ccPS0 = analog_to_pressure_CC(analog_in.read(0));
		tankPS0 = analog_to_pressure_tank(analog_in.read(1));
        lc0 = analog_to_pressure_lc(analog_in.read(2));
		// Data transfer happens at HS
		// Writing sensor JSON object for serialization
		// Do not write objects with allocated memory. Otherwise the loop will cause a memory leak
		sensorJson["mv1"]   = mv1->isOpen();
		sensorJson["mv2"]   = mv2->isOpen();
		sensorJson["pv"]    = pv->isOpen();
		sensorJson["bv"]   	= bv->isOpen();
		sensorJson["LC0"] 	= lc0;
		sensorJson["tPS0"] 	= tankPS0;
		sensorJson["tPS1"]	= 0.0;
		sensorJson["ccPS0"]	= ccPS0;
		sensorJson["ccPS1"]	= 0.0;
		sensorJson["T0"]	= temp_ch0;
		sensorJson["T1"]	= temp_ch1;
		sensorJson["T2"] 	= temp_ch2;
		

		// Write serialized object to Serial com port
		serializeJson(sensorJson, Serial); //Efficient but not very readable
		//serializeMsgPack(sensorJson,Serial); //Most efficient method but cannot be displayed in serial
		Serial.print("\n");

		// Set HS_PREVIOUS
		HS_PREVIOUS = millis();

	}
}

// This function takes the analog_in.read(ch) and converts it to the bar
// It was written to make the loop more readiable
float analog_to_pressure_CC(const float analog_read) {
    float voltage = (analog_read * reference) / 65535 / res_divider;
    return voltage / 10.0 * CCPressureMax; // P = (0-10V) v/10 (normalize) * 30 bar
}

float analog_to_pressure_tank(const float analog_read) {
    float voltage = (analog_read * reference) / 65535 / res_divider;
	 double PS_OFFSET = -tankPressureMax/8.0; 
	 long double ps_slope = 5*tankPressureMax/std::pow(2,23)/4;
    return ps_slope*voltage + PS_OFFSET; // P = (0-10V) v/10 (normalize) * 30 bar
}

float analog_to_pressure_lc(const float analog_read) {
    float voltage = (analog_read * reference) / 65535 / res_divider;
    return voltage * lc_slope; // P = (0-10V) v/10 (normalize) * 30 bar
}
