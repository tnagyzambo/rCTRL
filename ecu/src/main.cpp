#include <Serial.h>

unsigned char data = 99;   // for incoming serial data

bool status = false;

void setup() {
	pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);

	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, HIGH);
	digitalWrite(LEDB, HIGH);

	Serial.begin(9600);
}

void loop() {
	if (Serial.available() > 0) {
		data = (unsigned char)Serial.read();

		switch(data) {
			case 1:
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDB, HIGH);
				digitalWrite(LEDR, LOW);
				break;
			case 2:
				digitalWrite(LEDR, HIGH);
				digitalWrite(LEDB, HIGH);
				digitalWrite(LEDG, LOW);
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

