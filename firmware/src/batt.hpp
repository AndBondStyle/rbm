#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>


class Ina {
public:
	struct State {
		float voltage = 0.0f;
		float current = 0.0f;
		float percent = 0.0f;
	};

	Ina(TwoWire& wire, SemaphoreHandle_t& wireMutex, uint8_t address) : wire_(wire), wireMutex_(wireMutex), address_(address), ina219_(address) {

	}

	bool initIna() {
		INA_INIT_OK_ = false;
		if (!i2cCheck(wire_, address_)) {
            		return false;
        	}
		INA_INIT_OK_ = ina219_.begin(&wire_);
		if (INA_INIT_OK_) ina219_.setCalibration_32V_2A();
		return INA_INIT_OK_;
	}

	void updateIna() {
		while (true) {
			if (xSemaphoreTake(wireMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {

            			state_.voltage = ina219_.getBusVoltage_V();
            			state_.current = ina219_.getCurrent_mA();

            			xSemaphoreGive(wireMutex_);

            			state_.percent =
                		interpolSoc(
                    			state_.voltage,
                    			discharge_,
                    			chargeSoc_,
                    			BAT_TABLE_SIZE
                		);
        		}
			vTaskDelay(pdMS_TO_TICKS(250));
		}
	}

	bool isInit() const {
		return INA_INIT_OK_;
	}

	static void task(void* arg) {
		Ina* ina = static_cast<Ina*>(arg);
		ina->updateIna();
	}

	const State& getState() const {
		return state_;
	}

private:
	bool i2cCheck(TwoWire& bus, uint8_t address) {
        	bus.beginTransmission(address);
        	return bus.endTransmission() == 0;
    	}


    	float interpolSoc(float v, const float volt[], const float soc[], int n) {
		if (v >= volt[0]) {
			return soc[0];
		}

		if (v <= volt[n - 1]) {
			return soc[n - 1];
		}
		for (int i = 0; i < n - 1; ++i) {
			if (v <= volt[i] && v >= volt[i + 1]) {
				float t = (v - volt[i + 1]) / (volt[i] - volt[i + 1]);
				return soc[i + 1] + t * (soc[i] - soc[i + 1]);
			}
		}
		return 0.0f;
	}

	static constexpr int BAT_TABLE_SIZE = 10;

    	static constexpr float discharge_[BAT_TABLE_SIZE] = {
	12.40f,
        12.13f,
        12.04f,
        11.77f,
        11.54f,
        11.29f,
        11.00f,
        10.76f,
        10.53f,
        10.27f
    	};

    	static constexpr float chargeSoc_[BAT_TABLE_SIZE] = {
        100.0f,
        90.0f,
        80.0f,
        70.0f,
        60.0f,
        50.0f,
        40.0f,
        30.0f,
        20.0f,
        10.0f
    	};


	TwoWire& wire_;
	SemaphoreHandle_t& wireMutex_;

	uint8_t address_;

	Adafruit_INA219 ina219_;

	bool INA_INIT_OK_ = false;

	State state_;
};
