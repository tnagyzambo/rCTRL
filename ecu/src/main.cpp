#include <Arduino_MachineControl.h>
#include <Serial.h>

using namespace machinecontrol;

unsigned char data = 99;   // for incoming serial data

void setup() {
	//Set over current behavior of all channels to latch mode:
  	digital_outputs.setLatch();

	//At startup set all channels to CLOSED
  	digital_outputs.setAll(0);

	Serial.begin(9600);
}

void loop() {
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

	Serial.println("{\"configurations\":[{\"name\":\"Remote launch\",\"type\":\"lldb\",\"request\":\"launch\",\"program\":\"${workspaceFolder}/ros2/build/rstate/rstate_node\",\"initCommands\":[\"platform select remote-linux\",\"platform connect connect:localhost:3000\",\"settings set target.inherit-env true\"]}]}");
	delay(1);
}

