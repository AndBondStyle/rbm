#include <Arduino.h>
#include <Wire.h>

#include "motor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "batt.hpp"
#include "screen.hpp"
#include "led.hpp"
#include "proto.hpp"


void setup() {
	Serial.begin(921600);
	while (!Serial) delay(10);

	Wire.begin();
	Wire.setClock(400000);

	static TwoWire TOF_I2C(1);
	TOF_I2C.begin(33, 32, 1000000);

	static SemaphoreHandle_t wireMutex = xSemaphoreCreateMutex();

	if (wireMutex == nullptr) {
		while (true) {
	    		delay(1000);
		}
    	}

	static Motors motors;

	static Imu imu(Wire, wireMutex, 0x69);
	static Tof tof(TOF_I2C, 0x29);
	static Ina ina(Wire, wireMutex, 0x41);
	static Screen screen(Wire, wireMutex, 0x3C, 25, 26, imu, tof, ina);
	static Led led;
	static Proto proto(Serial, motors, imu, tof, ina, screen);


    	motors.initMotors();
    	imu.initImu();
    	tof.initToF();
    	ina.initIna();
    	screen.initScreen();
    	led.initLEDs();
    	proto.initProto();
	if (motors.isInit()) {
		xTaskCreatePinnedToCore(Motors::task, "motors", 2048, &motors, 1, NULL, 1);
	}

	if (imu.isInit()) {
		xTaskCreatePinnedToCore(Imu::task, "imu", 2048, &imu, 1, NULL, 1);
	}

	if (tof.isInit()) {
		xTaskCreatePinnedToCore(Tof::task, "tof", 2048, &tof, 1, NULL, 1);
	}
	if (ina.isInit()) {
		xTaskCreatePinnedToCore(Ina::task, "ina", 2048, &ina, 1, NULL, 1);
	}

	if (screen.isInit()) {
		xTaskCreatePinnedToCore(Screen::screenTask, "screen", 2048, &screen, 1, NULL, 1);
		xTaskCreatePinnedToCore(Screen::buttonTask, "button", 2048, &screen, 1, NULL, 1);
	}

	if (led.isInit()) {
		xTaskCreatePinnedToCore(Led::task, "leds", 2048, &led, 1, NULL, 1);
	}

	if (proto.isInit()) {
		xTaskCreatePinnedToCore(Proto::packTask, "pack_update", 2048, &proto, 1, NULL, 1);
		xTaskCreatePinnedToCore(Proto::packInTask, "PackInTask", 2048, &proto, 1, NULL, 1);
	}
}

void loop() {
}
