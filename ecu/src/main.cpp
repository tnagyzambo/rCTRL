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
const unsigned long HS_PERIOD = 10;  // in ms. Hz = 1000/period
const unsigned long LS_PERIOD = 100; // Lets see if 100 ms works
// Now we define the working variable that will be updated in the loop
unsigned long CURRENT_TIME, HS_PREVIOUS, LS_PREVIOUS;

//------------ Sensor setup definitions------------
SFE_ADS122C04 LoadCell;
SFE_ADS122C04 TankPS;
float res_divider = 0.28057; // Required for machine control
float reference = 3.3;       // Machine control reference
float CCPressureMax = 30.0;
// Sensor variables
float lc_value, tankPS0, tankPS1, ccPS0, ccPS1, temp_ch0, temp_ch1, temp_ch2;
int temp_channel_select = 0;
// Sensor functions
float analog_to_pressure(const float analog_read);

// Valve definition
auto mv1 = std::make_shared<NormClosed>(NormClosed(0));
auto mv2 = std::make_shared<NormClosed>(NormClosed(1));
auto pv = std::make_shared<NormClosed>(NormClosed(2));
auto bv = std::make_shared<NormOpen>(NormOpen(3));

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
	testSequence.addEvent(150,mv1,openValve);
	testSequence.addEvent(1050,mv1,closeValve);
	testSequence.addEvent(500,mv2,openValve);
	testSequence.addEvent(1500,mv2,closeValve);
	testSequence.addEvent(1000,pv,openValve);
	testSequence.addEvent(2000,pv,closeValve);
	testSequence.addEvent(1500,bv,closeValve);
	testSequence.addEvent(2500,bv,openValve);

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

	// ---------------------- I2C setup ----------------------------
	// Do this testing in final configuration with proper wiring as this will have a major affect on the speeds we can reach
	Wire.begin(); // No need to select I2C pins on the machine control
  	Wire.setClock(1000000); //Set to Fast mode + (1Mbps). Lower this value if errors on the i2c start to occur. 
	//------------- Set ADC modes ----------------
	// First we intialize and check the address configuration. See comments on which address is which board
	if (LoadCell.begin(0x40) == false) //Connect to the Load Cell using the defaults: Address 0x40 is the board without the direct connect connector. Defualt Wire port
	{
		// Serial.println(F("Error I2C bus to the Loadcell. Check address and I2C lines. Please check wiring. Freezing."));
		while (1);
	}

	if (TankPS.begin(0x44) == false) //Connect to the Pressure sensor using the defaults: Address 0x44 is the board with the direct connector. Defaul tWire port
	{
		// Serial.println(F("Error I2C bus to the Pressure Sensor. Check address and I2C lines. Please check wiring. Freezing."));
		while (1);
	}	

    // Configure ADC modes
    // TODO: set datarates as low as possible based on loop freq for lower noise
    LoadCell.configureADCmode(ADS122C04_LC_MODE, ADS122C04_DATA_RATE_600SPS); // 1200SPS due to turbo enabled
    LoadCell.setLCSlopeAndOffset(1, 0);        // 1, 0 is slope of 1 and offset of zero. Basically returning the differential voltage vin.
    LoadCell.start(); // Required after setting continuous mode bit in configureADC

    TankPS.configureADCmode(ADS122C04_PS_MODE, ADS122C04_DATA_RATE_600SPS); // 1200SPS due to turbo enabled
    TankPS.setPSMaxAndOffset(30.0); // Set the pressure sensor to the correct pressure
    TankPS.start();                // Required after setting continuous mode bit in configureADC

	//PGA offset calibration loop
	// TODO: make a wrapper function to clean up this
	// TODO: Fix functionality. Currently this step is not working at all
  	//Set false to skip
	if (false) {
		//Set mux to internal short
		LoadCell.setInputMultiplexer(ADS122C04_MUX_SHORTED);
		TankPS.setInputMultiplexer(ADS122C04_MUX_SHORTED);

        // Setup variables
        int samples = 100;
        double loadcell_offset = 0.0;
        double TankPS_offset = 0.0;

        for (int i = 0; i < samples; i++) {
            loadcell_offset -=
                LoadCell.readLC() / samples; // Go negative as the offset is added to the output of the function
            TankPS_offset -= TankPS.readPS() / samples;
            delay(25); // Delay a bit
        }

        // Set offsets
        LoadCell.setPGAOffset(loadcell_offset);
        TankPS.setPGAOffset(TankPS_offset);

        // Set Mux back
        LoadCell.setInputMultiplexer(ADS122C04_MUX_AIN0_AIN2);
        TankPS.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS);
    }

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

        // Read LC value
        lc_value = LoadCell.readLC(); // Use the read function. Units depend on calibration. The output voltage true voltage (aka raw/gain)

        // Read pressure sensor 1 and 2 (Tank sensors)
        // Currently both sensors will be same pressure range. If not we also need to change the slope and offset
        tankPS0 = TankPS.readPS(); // Read first pressure sensor in Bar. Default sense pin is the out+ pin (aka AIN2)
        TankPS.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); // Setting it to out- (aka AIN0)
        tankPS1 = TankPS.readPS();                           // Read second pressure sensor in Bar
        TankPS.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS); // Set back to default.

        // Read CC pressure sensors
        // Read raw voltage and then convert it to bar and save
        ccPS0 = analog_to_pressure(analog_in.read(0));
        ccPS1 = analog_to_pressure(analog_in.read(1));
		// Data transfer happens at HS
		// Writing sensor JSON object for serialization
		// Do not write objects with allocated memory. Otherwise the loop will cause a memory leak
		sensorJson["mv1"]   = mv1->isOpen();
		sensorJson["mv2"]   = mv2->isOpen();
		sensorJson["pv"]    = pv->isOpen();
		sensorJson["bv"]   	= bv->isOpen();
		sensorJson["LC0"] 	= lc_value;
		sensorJson["tPS0"] 	= tankPS0;
		sensorJson["tPS1"]	= tankPS1;
		sensorJson["ccPS0"]	= ccPS0;
		sensorJson["ccPS1"]	= ccPS1;
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
float analog_to_pressure(const float analog_read) {
    float voltage_ch0 = (analog_read * reference) / 65535 / res_divider;
    return voltage_ch0 / 10.0 * CCPressureMax; // P = (0-10V) v/10 (normalize) * 30 bar
}
