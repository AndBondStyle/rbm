#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <OneButton.h>

#include "imu.hpp"
#include "tof.hpp"
#include "batt.hpp"

class Screen {
public:
	enum class Page {
		MENU,
		INFO,
		SENSOR,
		ENERGY,
		TESTS
	};

	Screen(TwoWire& wire, SemaphoreHandle_t& wireMutex, uint8_t address, uint8_t btnA, uint8_t btnB, Imu& imu, Tof& tof, Ina& ina) : wire_(wire), wireMutex_(wireMutex), address_(address), btnA_(btnA), btnB_(btnB), button_one_(btnA, true), button_two_(btnB, true), imu_(imu), tof_(tof), ina_(ina), u8g2_(U8G2_R0, U8X8_PIN_NONE) {
		instance_ = this;
	}

	bool initScreen() {
		SCREEN_INIT_OK_ = false;

		if (!i2cCheck(wire_, address_)) {
			return false;
		}
    		pinMode(btnA_, INPUT);
    		pinMode(btnB_, INPUT);
    
    		u8g2_.begin();
    		u8g2_.clearBuffer();
    		u8g2_.setDrawColor(1);
		u8g2_.drawBox(0, 0, u8g2_.getDisplayWidth(), u8g2_.getDisplayHeight());
    		u8g2_.sendBuffer();
		
		button_one_.attachClick(
			buttonOneClickCallback
		);

		button_two_.attachClick(
			buttonTwoClickCallback
		);

		button_one_.attachLongPressStart(
            		buttonOneLongCallback
        	);

        	button_two_.attachLongPressStart(
            		buttonTwoLongCallback
        	);

		SCREEN_INIT_OK_ = true;
		return SCREEN_INIT_OK_;
	}

	void updateScreen() {
    		uint8_t counter = 0;
    
    		while (true) {
        		u8g2_.clearBuffer();
        		u8g2_.setFont(u8g2_font_profont12_mf);
			
			switch (page_) {
				case Page::MENU:
					drawMenu();
					break;
				case Page::INFO:
					drawInfo();
					break;
				case Page::SENSOR:
					drawSensors();
					break;
				case Page::ENERGY:
					drawEnergy();
					break;
				case Page::TESTS:
					drawTests();
					break;
			}

			if (xSemaphoreTake(wireMutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        			u8g2_.sendBuffer();
				xSemaphoreGive(wireMutex_);
			}

        		vTaskDelay(pdMS_TO_TICKS(250));
    		}
	}

	void updateButton() {	
		while (true) {
			button_one_.tick();
			button_two_.tick();

			if (button_one_long_check_ && button_two_long_check_) {
				esp_restart();
			}

			if (button_one_click_ && !button_two_click_) {
				if (page_ == Page::MENU) {
					if (selected_ == 0) {
						selected_ = MENU_ITEMS - 1;
					} else {
						selected_--;
					}
				}

				button_one_click_ = false;
			}

			if (!button_one_click_ && button_two_click_) {
				if (page_ == Page::MENU) {
					selected_++;
					if (selected_ >= MENU_ITEMS) {
						selected_ = 0;
					}
				}
				button_two_click_ = false;
			}

			if (button_one_long_check_ && !button_two_long_check_) {
				if (page_ == Page::MENU) {
					switch (selected_) {
						case 0:
							page_ = Page::INFO;
							break;
						case 1:
							page_ = Page::SENSOR;
							break;
						case 2:
							page_ = Page::ENERGY;
							break;
						case 3:
							page_ = Page::TESTS;
							break;
					}
				}
				button_one_long_check_ = false;
			}

			if (!button_one_long_check_ && button_two_long_check_) {
				if (page_ != Page::MENU) {
					page_ = Page::MENU;
				}
				button_two_long_check_ = false;
			}

			vTaskDelay(pdMS_TO_TICKS(20));
		}
	}

	void setNetworkInfo(const String& ip, const String& network) {
		ip_ = ip;
		network_ = network;
	}

	bool isInit() const {
		return SCREEN_INIT_OK_;
	}

	static void screenTask(void* arg) {
		Screen* screen = static_cast<Screen*>(arg);
		screen->updateScreen();
	}

	static void buttonTask(void* arg) {
		Screen* screen = static_cast<Screen*>(arg);
		screen->updateButton();
	}
private:
	static constexpr uint8_t MENU_ITEMS = 4;

	volatile Page page_ = Page::MENU;
	volatile uint8_t selected_ = 0;

	void drawMenu() {
		const char* items[MENU_ITEMS] = {
			"Info",
			"Sensor",
			"Energy",
			"Tests"
		};

		for (uint8_t i = 0; i < MENU_ITEMS; ++i) {
			const int y = i * 16;

			if (i == selected_) {
				u8g2_.setDrawColor(1);
				u8g2_.drawBox(0, y, 128, 16);

				u8g2_.setDrawColor(0);
				u8g2_.setCursor(5, y + 12);
				u8g2_.print(items[i]);
				u8g2_.setDrawColor(1);
			} else {
				u8g2_.setDrawColor(1);
				u8g2_.setCursor(5, y + 12);
				u8g2_.print(items[i]);
			}
		}
	}

	void drawInfo() {
		u8g2_.setCursor(0, 12);
        	u8g2_.print("Info");

        	u8g2_.setCursor(0, 28);
        	u8g2_.print("IP:");
        	u8g2_.print(ip_);

        	u8g2_.setCursor(0, 44);
        	u8g2_.print("Net:");

        	u8g2_.setCursor(0, 60);
        	u8g2_.print(network_);
	}

	void drawSensors() {
		u8g2_.setCursor(0, 12);
		u8g2_.print("Sensors");

		u8g2_.setCursor(0, 28);
		u8g2_.print("IMU: ");
		u8g2_.print(imu_.isInit() ? "OK" : "NONE");

		u8g2_.setCursor(0, 44);
		u8g2_.print("ToF: ");
		u8g2_.print(tof_.isInit() ? "OK" : "NONE");

		u8g2_.setCursor(0, 60);
		u8g2_.print("INA: ");
		u8g2_.print(ina_.isInit() ? "OK" : "NONE");
	}

	void drawEnergy() {
		const auto& bat = ina_.getState();
		
		u8g2_.setCursor(0, 12);
		u8g2_.print("Energy");

		u8g2_.setCursor(0, 28);
		u8g2_.print("V: ");
		u8g2_.print(bat.voltage, 2);
		u8g2_.print(" V");

		u8g2_.setCursor(0, 44);
		u8g2_.print("I: ");
		u8g2_.print(bat.current, 1);
		u8g2_.print(" mA");

		u8g2_.setCursor(0, 60);
		u8g2_.print("SOC: ");
		u8g2_.print(bat.percent, 1);
		u8g2_.print("%");
	}

	void drawTests() {
		const auto& dmp = imu_.getQuat();
		
		u8g2_.setCursor(0, 12);
		u8g2_.print("w:");
		u8g2_.print(dmp.w, 2);

		u8g2_.setCursor(0, 28);
		u8g2_.print("x:");
		u8g2_.print(dmp.x, 2);

		u8g2_.setCursor(0, 44);
		u8g2_.print("y:");
		u8g2_.print(dmp.y, 2);

		u8g2_.setCursor(0, 60);
		u8g2_.print("z:");
		u8g2_.print(dmp.z, 2);
	}

	bool i2cCheck(TwoWire& bus, uint8_t address) {
        	bus.beginTransmission(address);
        	return bus.endTransmission() == 0;
    	}

	static void buttonOneClickCallback() {
        	if (instance_ != nullptr) {
            		instance_->button_one_click_ = true;
        	}
    	}


    	static void buttonTwoClickCallback() {
        	if (instance_ != nullptr) {
            		instance_->button_two_click_ = true;
        	}
    	}

    	static void buttonOneLongCallback() {
        	if (instance_ != nullptr) {
            		instance_->button_one_long_check_ = true;
        	}
    	}


    	static void buttonTwoLongCallback() {
        	if (instance_ != nullptr) {
            		instance_->button_two_long_check_ = true;
        	}
    	}


    	TwoWire& wire_;
    	SemaphoreHandle_t& wireMutex_;

    	uint8_t address_;

    	uint8_t btnA_;
    	uint8_t btnB_;


    	U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_;

    	OneButton button_one_;
    	OneButton button_two_;


    	Imu& imu_;
    	Tof& tof_;
    	Ina& ina_;


    	bool SCREEN_INIT_OK_ = false;
	
	volatile bool button_one_click_ = false;
	volatile bool button_two_click_ = false;
    	volatile bool button_one_long_check_ = false;
    	volatile bool button_two_long_check_ = false;

    	volatile uint8_t check_1_ = 0;
    	volatile uint8_t check_2_ = 0;

	String ip_ = "N/A";
	String network_ = "N/A";

    	inline static Screen* instance_ = nullptr;
};
