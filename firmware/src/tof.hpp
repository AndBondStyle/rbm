#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>


class Tof {
public:
	static constexpr int ZONES = 64;

	struct State {
		int16_t distance_mm[ZONES] = {};
		uint8_t reflectance[ZONES] = {};
		uint8_t status[ZONES] = {};
	};

	Tof(TwoWire& wire, uint8_t address) : wire_(wire), address_(address) {
	}

	bool initToF() {
		TOF_INIT_OK_ = false;
		if (!i2cCheck(wire_, address_)) {
			return false;
		}
   		TOF_INIT_OK_ = lidar_.begin(address_, wire_);
    		if (TOF_INIT_OK_) {
        		lidar_.setResolution(64);
        		lidar_.setRangingFrequency(15);
        		lidar_.startRanging();
    		}
		return TOF_INIT_OK_;
	}

	void updateToF() {
    
   		while (true) {
        		if (lidar_.isDataReady() && lidar_.getRangingData(&lidar_data_)) {
            			for (int i = 0; i < 64; i++) {
            				state_.distance_mm[i] = lidar_data_.distance_mm[i];
        				state_.reflectance[i] = lidar_data_.reflectance[i];
        				state_.status[i] = lidar_data_.target_status[i];
        			}
        		}
        
        		vTaskDelay(pdMS_TO_TICKS(10));
    		}
	}

	bool isInit() const {
		return TOF_INIT_OK_;
	}

	static void task(void* arg) {
		Tof* tof = static_cast<Tof*>(arg);
		tof->updateToF();
	}

	const State& getState() const {
		return state_;
	}

private:
	bool i2cCheck(TwoWire& bus,uint8_t address) {
        	bus.beginTransmission(address);
        	return bus.endTransmission() == 0;
    	}


    	TwoWire& wire_;
    	uint8_t address_;

    	SparkFun_VL53L5CX lidar_;
    	VL53L5CX_ResultsData lidar_data_;

    	bool TOF_INIT_OK_ = false;

    	State state_;
};
