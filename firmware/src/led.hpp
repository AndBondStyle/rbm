#pragma once

#include <Arduino.h>
#include <FastLED.h>

constexpr int WS_LED_PIN = 16;
constexpr int NUM_LEDS = 80;
constexpr int MAX_BRIGHTNESS = 128;


class Led {
public:
	bool initLEDs() {
		LED_INIT_OK_ = false;

    		FastLED.addLeds<WS2812, WS_LED_PIN, GRB>(leds_, NUM_LEDS); 
    		FastLED.setBrightness(0);
		
		fill_solid(leds_, NUM_LEDS, CRGB::White);
		FastLED.show();

		LED_INIT_OK_ = true;
		return LED_INIT_OK_;
	}

	void updateLEDs() {
    		int brightness = 0;
    		int dir = 1;

    		while (true) {
			FastLED.setBrightness(brightness);
			FastLED.show();

			brightness += dir;
			if (brightness >= MAX_BRIGHTNESS) {
				brightness = MAX_BRIGHTNESS;
				dir = -1;
			}

			if (brightness <= 0) {
				brightness = 0;
				dir = 1;
			}
			vTaskDelay(pdMS_TO_TICKS(15));
    		}

    //uint8_t pos = 0;
    //int8_t direction = 1;
    //uint8_t r_count = 0;
    //uint8_t g_count = 0;
    //uint8_t b_count = 0;

    //while (true) {
    //    fadeToBlackBy(leds, NUM_LEDS, 20);
    //    leds[pos] = CRGB(r_count, g_count, b_count);
    //    if (r_count + direction * 15 <= 255) r_count += direction * 15;
    //    else if (r_count - direction * 15 >= 0) r_count -=direction * 15;
    //    if (g_count + direction * 15 <= 255) g_count += direction * 15;
    //    else if (r_count - direction * 15 >= 0) g_count -=direction * 15;
    //    if (b_count + direction * 15 <= 255) b_count += direction * 15;
    //    else if (r_count - direction * 15 >= 0) b_count -=direction * 15;
        
    //    pos += direction;
    //    if (pos == 0 || pos == NUM_LEDS - 1) direction = -direction;
    //    
    //    FastLED.show();
    //    vTaskDelay(pdMS_TO_TICKS(50));
    //}
	}

	bool isInit() const {
		return LED_INIT_OK_;
	}

	static void task(void* arg) {
		Led* led = static_cast<Led*>(arg);
		led->updateLEDs();
	}

private:
	CRGB leds_[NUM_LEDS];

	bool LED_INIT_OK_ = false;
};
