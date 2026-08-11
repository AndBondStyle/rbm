#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <ICM_20948.h>
#include <U8g2lib.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <SimpleDCMotor.h>
#include <Adafruit_INA219.h>
#include <OneButton.h>
#include "ESP32HWEncoderFix.h"

bool INA_INIT_OK = false;
bool TOF_INIT_OK = false;
bool IMU_INIT_OK = false;

// ==================================== //

const int ENCODER_CPR = 330;
constexpr float VOLTAGE = 1.0f;
constexpr float VEL_LIM = 40;
const int PWM_FREQ = 10000;

constexpr float VEL_PID_P = 2.0f; //2
constexpr float VEL_PID_I = 0.01f; //10
constexpr float VEL_PID_D = 0.001f; //0.001
constexpr float VEL_PID_LIM = 500.0f;
constexpr float VEL_LPF_TF = 0.02;
constexpr float EPSILON = 0.001f;

class CustomMotor : public DCMotor {
    public:
    using DCMotor::DCMotor;

    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) override {
        if (enabled) {
            if (fabs(target) < EPSILON) {
                driver->setPwm(0);
                driver->disable();
                PID_velocity.reset();
            } else if (fabs(Uq) < EPSILON) {
                driver->setPwm(0);
            } else if ((target > 0 && Uq < 0) || (target < 0 && Uq > 0)) {
                driver->setPwm(0);
                driver->disable();
            } else {
                driver->setPwm(Uq);
            }
        }
        _UNUSED(Ud);
        _UNUSED(angle_el);
    }
};

Commander commander = Commander(Serial);

CustomMotor motor_a = CustomMotor();
DCDriver2PWM driver_a = DCDriver2PWM(13, 27);
ESP32HWEncoder sensor_a = ESP32HWEncoder(19, 18, ENCODER_CPR);
void onMotorA(char* cmd){ commander.motor(&motor_a, cmd); }

CustomMotor motor_b = CustomMotor();
DCDriver2PWM driver_b = DCDriver2PWM(2, 4);
ESP32HWEncoder sensor_b = ESP32HWEncoder(23, 5, ENCODER_CPR);
void onMotorB(char* cmd){ commander.motor(&motor_b, cmd); }

CustomMotor motor_c = CustomMotor();
DCDriver2PWM driver_c = DCDriver2PWM(12, 17);
ESP32HWEncoder sensor_c = ESP32HWEncoder(36, 35, ENCODER_CPR);
void onMotorC(char* cmd){ commander.motor(&motor_c, cmd); }

CustomMotor motor_d = CustomMotor();
DCDriver2PWM driver_d = DCDriver2PWM(14, 15);
ESP32HWEncoder sensor_d = ESP32HWEncoder(39, 34, ENCODER_CPR);
void onMotorD(char* cmd){ commander.motor(&motor_d, cmd); }

void initMotorStack(char c, CustomMotor &motor, DCDriver2PWM &driver, ESP32HWEncoder &sensor) {
    driver.voltage_power_supply = VOLTAGE;
    driver.voltage_limit = VOLTAGE;
    driver.pwm_frequency = PWM_FREQ;
    driver.init();
    sensor.init();
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);

    motor.voltage_limit = VOLTAGE;
    motor.velocity_limit = VEL_LIM;
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.init();

    motor.PID_velocity.P = VEL_PID_P;
    motor.PID_velocity.I = VEL_PID_I;
    motor.PID_velocity.D = VEL_PID_D;
    motor.PID_velocity.output_ramp = VEL_PID_LIM;
    motor.LPF_velocity.Tf = VEL_LPF_TF;


    motor.useMonitoring(Serial);
    motor.monitor_downsample = 1;
    motor.monitor_decimals = 2;
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    motor.monitor_start_char = c;

    motor.target = 0.0f;
    motor.enable();
}

void initMotors() {
    // SimpleFOCDebug::enable(&Serial);
    commander.verbose = VerboseMode::nothing;
    initMotorStack('A', motor_a, driver_a, sensor_a);
    initMotorStack('B', motor_b, driver_b, sensor_b);
    initMotorStack('C', motor_c, driver_c, sensor_c);
    initMotorStack('D', motor_d, driver_d, sensor_d);
    commander.add('A', onMotorA, "motor A");
    commander.add('B', onMotorB, "motor B");
    commander.add('C', onMotorC, "motor C");
    commander.add('D', onMotorD, "motor D");
}

void updateMotors(void* _) {
    uint32_t last_ts = 0;
    
    while (true) {
        motor_a.move();
        motor_b.move();
        motor_c.move();
        motor_d.move();
        commander.run();
    
        uint32_t now = micros();
        if ((now - last_ts >= 100000) || (now < last_ts)) {
            motor_a.monitor();
            motor_b.monitor();
            motor_c.monitor();
            motor_d.monitor();
            last_ts = now;
        }

        vTaskDelay(0);
    }
}

// ==================================== //

#define WS_LED_PIN 16
#define NUM_LEDS 30
CRGB leds[NUM_LEDS];

void initLEDs() {
    FastLED.addLeds<WS2812, WS_LED_PIN, GRB>(leds, NUM_LEDS); 
    FastLED.setBrightness(128);
}

void updateLEDs(void* _) {
    uint8_t pos = 0;
    int8_t direction = 1;
    uint8_t r_count = 0;
    uint8_t g_count = 0;
    uint8_t b_count = 0;

    while (true) {
        fadeToBlackBy(leds, NUM_LEDS, 20);
        leds[pos] = CRGB(r_count, g_count, b_count);
        if (r_count + direction * 15 <= 255) r_count += direction * 15;
        else if (r_count - direction * 15 >= 0) r_count -=direction * 15;
        if (g_count + direction * 15 <= 255) g_count += direction * 15;
        else if (r_count - direction * 15 >= 0) g_count -=direction * 15;
        if (b_count + direction * 15 <= 255) b_count += direction * 15;
        else if (r_count - direction * 15 >= 0) b_count -=direction * 15;
        
        pos += direction;
        if (pos == 0 || pos == NUM_LEDS - 1) direction = -direction;
        
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================================== //

#define IMU_WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

void initIMU() {
    myICM.begin(IMU_WIRE_PORT, AD0_VAL);
    bool initialized = myICM.status == ICM_20948_Stat_Ok;
    
    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    if (success) IMU_INIT_OK = true;
}

void updateIMU(void* _) {
    if (!IMU_INIT_OK) {
        while (true) taskYIELD();
    }

    uint32_t last_ts = 0;
    
    while (true) {
        icm_20948_DMP_data_t data;
        myICM.readDMPdataFromFIFO(&data);
        double q1, q2, q3, q0;
        
        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
            if ((data.header & DMP_header_bitmap_Quat9) > 0) {
                q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
                q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
                q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
                // q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            }
        }

        uint32_t now = micros();
        if ((now - last_ts >= 100000) || (now < last_ts)) {
            Serial.print("Q");
            Serial.print(q0, 3);
            Serial.print(" ");
            Serial.print(q1, 3);
            Serial.print(" ");
            Serial.print(q2, 3);
            Serial.print(" ");
            Serial.print(q3, 3);
            Serial.print(" ");
            Serial.println(data.Quat9.Data.Accuracy);
            last_ts = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================================== //

#define BTN_A 25
#define BTN_B 26

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

OneButton button_one(BTN_A, true);
OneButton button_two(BTN_B, true);

void initScreen() {
    pinMode(BTN_A, INPUT);
    pinMode(BTN_B, INPUT);
    
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, u8g2.getDisplayWidth(), u8g2.getDisplayHeight());
    u8g2.sendBuffer();
}

volatile bool restart_check = false;

void restart() {
	restart_check = true;
}

void updateButton(void* _) {	
	button_one.attachLongPressStart(restart);
	while (true) {
		button_one.tick();

		if (restart_check) {
			esp_restart();
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void updateScreen(void* _) {
    uint8_t counter = 0;
    
    while (true) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_profont12_mf);
        u8g2.setCursor(5, 15);
        u8g2.print("SCREEN TEST ");
        u8g2.print(counter);
        u8g2.setCursor(5, 35);
        u8g2.print("BUTTONS: A=");
        u8g2.print(!digitalRead(BTN_A));
        u8g2.print(" B=");
        u8g2.print(!digitalRead(BTN_B));
        u8g2.setCursor(5, 55);
        u8g2.print("IMU=");
        u8g2.print(IMU_INIT_OK);
        u8g2.print(" ToF=");
        u8g2.print(TOF_INIT_OK);
	u8g2.print(" INA=");
	u8g2.print(INA_INIT_OK);
        u8g2.sendBuffer();
        counter += 1;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================================== //

//#define TOF_WIRE_PORT Wire1

TwoWire TOF_I2C = TwoWire(1);
SparkFun_VL53L5CX lidar;
VL53L5CX_ResultsData lidar_data;

void initToF() {
    TOF_I2C.begin(33, 32, 1000000);
    TOF_INIT_OK = lidar.begin(0x29, TOF_I2C);
    if (TOF_INIT_OK) {
        lidar.setResolution(64);
        lidar.setRangingFrequency(5);
        lidar.startRanging();
    }
}

void updateToF(void* _) {
    if (!TOF_INIT_OK) {
        while (true) taskYIELD();
    }
    
    while (true) {
        if (lidar.isDataReady() && lidar.getRangingData(&lidar_data)) {
            Serial.print("L");
            for (int i = 0; i < 64; i++) {
                // Serial.print(lidar_data.distance_mm[i] > 3000 ? 3000 : lidar_data.distance_mm[i], DEC);
                Serial.print(lidar_data.distance_mm[i], DEC);
                if (i < 63) Serial.print(",");
            }
            Serial.print(";");
            for (int i = 0; i < 64; i++) {
                Serial.print(lidar_data.target_status[i], DEC);
                if (i < 63) Serial.print(",");
            }
            Serial.println();
            Serial.flush();
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// ==================================== //
Adafruit_INA219 ina219(0x41);


void initIna() {
	INA_INIT_OK = ina219.begin();
	if (INA_INIT_OK) ina219.setCalibration_32V_2A();
}

void updateIna(void* _) {
	if (!INA_INIT_OK) {
		while (true) taskYIELD();		
	}
	while (true) {
		Serial.print("V: ");
		Serial.print(ina219.getBusVoltage_V());
		Serial.print("V I: ");
		Serial.print(ina219.getCurrent_mA() * 10);
		Serial.print("mA P: ");
		Serial.print(ina219.getPower_mW());
		Serial.print("mw");
		Serial.println();
	}

	vTaskDelay(pdMS_TO_TICKS(250));
}


// ==================================== //

void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(921600);
    while (!Serial) delay(10);

    Wire.begin();
    Wire.setClock(400000);
    
    //Wire1.begin(33, 32);
    //Wire1.setClock(1000000);

    initMotors();
    initLEDs();
    initIMU();
    initScreen();
    initToF();
    initIna();
    
    xTaskCreatePinnedToCore(updateMotors, "motors", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateLEDs, "leds", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateIMU, "imu", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateScreen, "screen", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateToF, "tof", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateIna, "ina", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateButton, "button", 2048, NULL, 1, NULL, 1);
}

void loop() {
	Serial.print("TOF: ");
	Serial.print(TOF_INIT_OK);
	Serial.print(" IMU: ");
	Serial.print(IMU_INIT_OK);
	Serial.println();
	delay(1000);
}
