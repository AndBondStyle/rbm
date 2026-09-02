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
#include <stdfloat>
#define PACKETIZER_USE_INDEX_AS_DEFAULT
#define PACKETIZER_USE_CRC_AS_DEFAULT
#include <Packetizer.h>
#include "ESP32HWEncoderFix.h"
// #include "encoders/esp32hwencoder/ESP32HWEncoder.h"

bool INA_INIT_OK = false;
bool TOF_INIT_OK = false;
bool IMU_INIT_OK = false;
bool SCREEN_INIT_OK = false;

bool INA_PRESENT = false;
bool TOF_PRESENT = false;
bool IMU_PRESENT = false;
bool SCREEN_PRESENT = false;

SemaphoreHandle_t wireMutex = nullptr;
// ==================================== //

#define MSG_STAT 1
#define MSG_IMU 2
#define MSG_TOF 3
#define MSG_MOT_IN 4
#define MSG_PID_CONF 5

struct __attribute__((packed)) ControlPayloadIn {
	float front_left_speed;
	float front_right_speed;
	float rear_left_speed;
	float rear_right_speed;
};

struct __attribute__((packed)) PIDConfigIn {
	float kp;
    float ki;
    float kd;
    float limit;
    float lpf_tf;
};

struct __attribute__((packed)) StatusPayloadOut {
	uint32_t flags;
	float front_left_motor_stat[4]; //0 - target, 1 - shaft_velocity, 2 - shaft_angle, 3 - voltage.q
	float front_right_motor_stat[4];
	float rear_left_motor_stat[4];
	float rear_right_motor_stat[4];
	float bat_status[3]; // 0 - voltage, 1 - current, 2 - percent
};

struct __attribute__((packed)) ImuPayloadOut {
	float acc[3]; // XYZ
	float gyr[3];
	float quat[4];
};

#define TOF_ZONES 64
struct __attribute__((packed)) TofPayloadOut {
	int16_t distance_mm[TOF_ZONES];
	uint8_t reflectance[TOF_ZONES];
	uint8_t status[TOF_ZONES];
};

#define MAX_PAYLOAD_SIZE sizeof(TofPayloadOut)

void sendPacket(uint8_t msg_type, const void* payload, size_t payloadSize) {
	//uint8_t frame[MAX_PAYLOAD_SIZE];

	if (payloadSize > MAX_PAYLOAD_SIZE) {
		return;
	}

	//frame[0] = msg_type;
	//memcpy(frame + 1, payload, payloadSize);

	Packetizer::send(
		Serial,
		msg_type,
		(const uint8_t*)payload,
		payloadSize
	);
}

StatusPayloadOut status_pack_out;
ImuPayloadOut imu_pack_out;
TofPayloadOut tof_pack_out;

// ==================================== //

const int ENCODER_CPR = 330;
constexpr float VOLTAGE = 1.0f;
constexpr float VEL_LIM = 40;
const int PWM_FREQ = 10000;

constexpr float VEL_PID_P = 0.5f; //2
constexpr float VEL_PID_I = 5.0f; //10
constexpr float VEL_PID_D = 0.001f; //0.001
constexpr float VEL_PID_LIM = 500.0f;
constexpr float VEL_LPF_TF = 0.01;
constexpr float EPSILON = 0.001f;

class CustomMotor : public DCMotor {
    public:
    using DCMotor::DCMotor;

    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) override {
        if (enabled) {
            // зачем с-style модуль?
            if (fabs(target) < EPSILON) {
                driver->setPwm(0);
                driver->disable();
                PID_velocity.reset();
            } else if (fabs(Uq) < EPSILON) {
                driver->setPwm(0);
            } else if ((target > 0 && Uq < 0) || (target < 0 && Uq > 0)) {
                driver->setPwm(0);
                //driver->disable();
            } else {
                driver->setPwm(Uq);
            }
        }
        _UNUSED(Ud);
        _UNUSED(angle_el);
    }
};

//Commander commander = Commander(Serial);

CustomMotor motor_a = CustomMotor();
DCDriver2PWM driver_a = DCDriver2PWM(13, 27);
ESP32HWEncoder sensor_a = ESP32HWEncoder(19, 18, ENCODER_CPR);
//void onMotorA(char* cmd){ commander.motor(&motor_a, cmd); }

CustomMotor motor_b = CustomMotor();
DCDriver2PWM driver_b = DCDriver2PWM(2, 4);
ESP32HWEncoder sensor_b = ESP32HWEncoder(23, 5, ENCODER_CPR);
//void onMotorB(char* cmd){ commander.motor(&motor_b, cmd); }

CustomMotor motor_c = CustomMotor();
DCDriver2PWM driver_c = DCDriver2PWM(12, 17);
ESP32HWEncoder sensor_c = ESP32HWEncoder(36, 35, ENCODER_CPR);
//void onMotorC(char* cmd){ commander.motor(&motor_c, cmd); }

CustomMotor motor_d = CustomMotor();
DCDriver2PWM driver_d = DCDriver2PWM(14, 15);
ESP32HWEncoder sensor_d = ESP32HWEncoder(39, 34, ENCODER_CPR);
//void onMotorD(char* cmd){ commander.motor(&motor_d, cmd); }

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

    //motor.useMonitoring(Serial);
    //motor.monitor_downsample = 1;
    //motor.monitor_decimals = 2;
    //motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    //motor.monitor_start_char = c;

    motor.target = 0.0f;
    motor.enable();
}

void initMotors() {
    initMotorStack('A', motor_a, driver_a, sensor_a);
    initMotorStack('B', motor_b, driver_b, sensor_b);
    initMotorStack('C', motor_c, driver_c, sensor_c);
    initMotorStack('D', motor_d, driver_d, sensor_d);
}

void updateMotors(void* _) {
    TickType_t last_ts = 0;
    
    while (true) {
        motor_a.move();
        motor_b.move();
        motor_c.move();
        motor_d.move();
        vTaskDelay(pdMS_TO_TICKS(0));
    }
}

// ==================================== //

ControlPayloadIn control_pack_in;
void ApplyMotorCommand() {
	motor_a.target = control_pack_in.front_left_speed;
	motor_b.target = control_pack_in.rear_left_speed;
	motor_c.target = control_pack_in.rear_right_speed;
	motor_d.target = control_pack_in.front_right_speed;
}

PIDConfigIn pid_config_in;
void ApplyPidCommand() {
    CustomMotor* motor_array[4] = {&motor_a, &motor_b, &motor_c, &motor_d};
    for (int i = 0; i < 4; i++) {
        motor_array[i]->PID_velocity.P = pid_config_in.kp;
        motor_array[i]->PID_velocity.I = pid_config_in.ki;
        motor_array[i]->PID_velocity.D = pid_config_in.kd;
        motor_array[i]->PID_velocity.output_ramp = pid_config_in.limit;
        motor_array[i]->LPF_velocity.Tf = pid_config_in.lpf_tf; 
        //motor_array[i]->PID_velocity.reset();
    }
}

void HandlePack(uint8_t msg_type, const uint8_t* payload, size_t payloadSize) {
	if (payloadSize == 0) {
		return;
	}

	if (msg_type == MSG_MOT_IN) {
		if (payloadSize != sizeof(ControlPayloadIn)) return;
		memcpy(&control_pack_in, payload, sizeof(control_pack_in));
		ApplyMotorCommand();
	} else if (msg_type == MSG_PID_CONF) {
        if (payloadSize != sizeof(PIDConfigIn)) return;
		memcpy(&pid_config_in, payload, sizeof(PIDConfigIn));
        ApplyPidCommand();
        // ~('_')~
    }
}

void updatePackIn(void* _) {
	while (true) {
		Packetizer::parse();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

// ==================================== //

#define WS_LED_PIN 16
#define NUM_LEDS 80
CRGB leds[NUM_LEDS];

void initLEDs() {
    FastLED.addLeds<WS2812, WS_LED_PIN, GRB>(leds, NUM_LEDS); 
    FastLED.setBrightness(128);
}

void updateLEDs(void* _) {
    int brightness = 0;
    int dir = 1;

    fill_solid(leds, NUM_LEDS, CRGB::White);

    while (true) {
	FastLED.setBrightness(brightness);
	FastLED.show();

	brightness += dir;
	if (brightness >= 128) {
		brightness = 128;
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

// ==================================== //

#define IMU_WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

void initIMU() {
    myICM.begin(IMU_WIRE_PORT, AD0_VAL);
    bool initialized = myICM.status == ICM_20948_Stat_Ok;
    
    bool success = initialized;
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
    //if (!IMU_INIT_OK) {
    //    while (true) taskYIELD();
    //}

    uint32_t last_ts = 0;
    
    while (true) {
        icm_20948_DMP_data_t data;
	if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        	myICM.readDMPdataFromFIFO(&data);

		xSemaphoreGive(wireMutex);
        double q1, q2, q3, q0;
        
        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
            
		if ((data.header & DMP_header_bitmap_Quat9) > 0) {
                	q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
                	q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
                	q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
                	q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
                	imu_pack_out.quat[0] = q0;
	    		imu_pack_out.quat[1] = q1;
	   		imu_pack_out.quat[2] = q2;
	    		imu_pack_out.quat[3] = q3;
	    }
        }
	}

        uint32_t now = micros();
        if ((now - last_ts >= 100000) || (now < last_ts)) {
            if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
	        myICM.getAGMT();
	    

    	    if (myICM.status == ICM_20948_Stat_Ok) {
        		imu_pack_out.acc[0] = myICM.accX();
    	    	imu_pack_out.acc[1] = myICM.accY();
    	    	imu_pack_out.acc[2] = myICM.accZ();
    
    	    	imu_pack_out.gyr[0] = myICM.gyrX();
    	    	imu_pack_out.gyr[1] = myICM.gyrY();
    	    	imu_pack_out.gyr[2] = myICM.gyrZ();
    	    }
	    xSemaphoreGive(wireMutex);
	    }

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

volatile bool button_one_long_check = false;
volatile bool button_two_long_check = false;

void button_one_long() {
	button_one_long_check = true;
}

void button_two_long() {
	button_two_long_check = true;
}

volatile uint8_t check_1 = 0;
volatile uint8_t check_2 = 0;

void updateButton(void* _) {	
	button_one.attachLongPressStart(button_one_long);
	button_two.attachLongPressStart(button_two_long);
	while (true) {
		button_one.tick();
		button_two.tick();

		if (!button_one_long_check && button_two_long_check) {
			check_1 = 0;
			check_2 = 0;
		}

		if (button_one_long_check && !button_two_long_check) {
			check_1 = 2;
		}

		if (!button_one_long_check && button_two_long_check) {
			check_2 = 3;
		}

		if (button_one_long_check && button_two_long_check) {
			esp_restart();
		}

		button_one_long_check = false;
		button_two_long_check = false;


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
        u8g2.print(check_1);
        u8g2.print(" B=");
        u8g2.print(check_2);
        u8g2.setCursor(5, 55);
        u8g2.print("IMU=");
        u8g2.print(IMU_INIT_OK);
        u8g2.print(" ToF=");
        u8g2.print(TOF_INIT_OK);
	u8g2.print(" INA=");
	u8g2.print(INA_INIT_OK);
	if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        	u8g2.sendBuffer();
		xSemaphoreGive(wireMutex);
	}
        counter += 1;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ==================================== //

TwoWire TOF_I2C = TwoWire(1);
SparkFun_VL53L5CX lidar;
VL53L5CX_ResultsData lidar_data;

void initToF() {
    //TOF_I2C.begin(33, 32, 1000000);
    TOF_INIT_OK = lidar.begin(0x29, TOF_I2C);
    if (TOF_INIT_OK) {
        lidar.setResolution(64);
        lidar.setRangingFrequency(5);
        lidar.startRanging();
    }
}

void updateToF(void* _) {
    //if (!TOF_INIT_OK) {
    //    while (true) taskYIELD();
    //}
    
    while (true) {
        if (lidar.isDataReady() && lidar.getRangingData(&lidar_data)) {
            for (int i = 0; i < 64; i++) {
            	tof_pack_out.distance_mm[i] = lidar_data.distance_mm[i];
        		tof_pack_out.reflectance[i] = lidar_data.reflectance[i];
        		tof_pack_out.status[i] = lidar_data.target_status[i];
        	}
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// ==================================== //
Adafruit_INA219 ina219(0x41);

const int BAT_TABLE_SIZE = 10;
const float discharge[BAT_TABLE_SIZE] = {
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

const float chargeSoc[BAT_TABLE_SIZE] = {
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


void initIna() {
	INA_INIT_OK = ina219.begin();
	if (INA_INIT_OK) ina219.setCalibration_32V_2A();
}

void updateIna(void* _) {
	//if (!INA_INIT_OK) {
	//	while (true) taskYIELD();		
	//}
	while (true) {
		float voltage = 0.0f;
		float current = 0.0f;
		if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(50)) == pdTRUE) {

            		voltage = ina219.getBusVoltage_V();
            		current = ina219.getCurrent_mA();

            		xSemaphoreGive(wireMutex);

           		status_pack_out.bat_status[0] = voltage;
            		status_pack_out.bat_status[1] = current;
            		status_pack_out.bat_status[2] =
                	interpolSoc(
                    		voltage,
                    		discharge,
                    		chargeSoc,
                    		BAT_TABLE_SIZE
                	);
        	}
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

// ==================================== //



void updatePack(void* _) {
	uint32_t lastStatus = 0;
	uint32_t lastImu = 0;
	uint32_t lastTof = 0;
	while (true) {
		uint32_t now = millis();

		status_pack_out.flags = (INA_INIT_OK << 1) | (TOF_INIT_OK << 2) | (IMU_INIT_OK << 3) | (SCREEN_INIT_OK << 0);
		if (now - lastStatus >= 100) { 
			lastStatus = now;
        
        status_pack_out.front_left_motor_stat[0] = motor_a.target;
        status_pack_out.front_left_motor_stat[1] = motor_a.shaft_velocity;
        status_pack_out.front_left_motor_stat[2] = motor_a.shaft_angle;
        status_pack_out.front_left_motor_stat[3] = motor_a.voltage.q;
        
        status_pack_out.front_right_motor_stat[0] = motor_d.target;
        status_pack_out.front_right_motor_stat[1] = motor_d.shaft_velocity;
        status_pack_out.front_right_motor_stat[2] = motor_d.shaft_angle;
        status_pack_out.front_right_motor_stat[3] = motor_d.voltage.q;
       
        status_pack_out.rear_left_motor_stat[0] = motor_b.target;
        status_pack_out.rear_left_motor_stat[1] = motor_b.shaft_velocity;
        status_pack_out.rear_left_motor_stat[2] = motor_b.shaft_angle;
        status_pack_out.rear_left_motor_stat[3] = motor_b.voltage.q;
        
        status_pack_out.rear_right_motor_stat[0] = motor_c.target;
        status_pack_out.rear_right_motor_stat[1] = motor_c.shaft_velocity;
        status_pack_out.rear_right_motor_stat[2] = motor_c.shaft_angle;
        status_pack_out.rear_right_motor_stat[3] = motor_c.voltage.q;
		
		StatusPayloadOut pack = status_pack_out;
		sendPacket(MSG_STAT, &pack, sizeof(pack));      
        
		}
		if (now - lastImu >= 20) {
		 	lastImu = now;
			ImuPayloadOut pack = imu_pack_out;
		 	sendPacket(MSG_IMU, &pack, sizeof(pack));
		}

		if (now - lastTof >= 50) {
		 	lastTof = now;
			TofPayloadOut pack = tof_pack_out;
		 	sendPacket(MSG_TOF, &pack, sizeof(pack));
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

bool i2cCheck(TwoWire& bus, uint8_t address) {
	bus.beginTransmission(address);
	return bus.endTransmission() == 0;
}

void setup() {
    //Serial.setTxBufferSize(1024);
    Serial.begin(921600);
    while (!Serial) delay(10);

    Wire.begin();
    Wire.setClock(400000);

    wireMutex = xSemaphoreCreateMutex();

    if (wireMutex == nullptr) {
	while (true) {
	    delay(1000);
	}
    }

    TOF_I2C.begin(33, 32, 1000000);
    
    //Wire1.begin(33, 32);
    //Wire1.setClock(1000000);
    

    SCREEN_PRESENT = i2cCheck(Wire, 0x3C);
    INA_PRESENT = i2cCheck(Wire, 0x41);
    IMU_PRESENT = i2cCheck(Wire, 0x69);
    TOF_PRESENT = i2cCheck(TOF_I2C, 0x29);

    Packetizer::subscribe(Serial, MSG_MOT_IN, [&](const uint8_t* data, const size_t size) {
	if (size != sizeof(ControlPayloadIn)) return;
        memcpy(&control_pack_in, data, sizeof(control_pack_in));
        ApplyMotorCommand();
    });

    Packetizer::subscribe(Serial, MSG_PID_CONF, [&](const uint8_t* data, const size_t size) {
	if (size != sizeof(PIDConfigIn)) return;
        memcpy(&pid_config_in, data, sizeof(pid_config_in));
        ApplyPidCommand();
    });

    initMotors();
    initLEDs();
    if (IMU_PRESENT) {
    	initIMU();
    }
    if (SCREEN_PRESENT) {
    	initScreen();
	SCREEN_INIT_OK = true;
    }
    if (TOF_PRESENT) {
    	initToF();
    }
    if (INA_PRESENT) {
    	initIna();
    }
    
    xTaskCreatePinnedToCore(updateMotors, "motors", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateLEDs, "leds", 2048, NULL, 1, NULL, 1);
    if (IMU_INIT_OK) {
	    xTaskCreatePinnedToCore(updateIMU, "imu", 2048, NULL, 1, NULL, 1);
    }
    if (TOF_INIT_OK) {
    	xTaskCreatePinnedToCore(updateToF, "tof", 2048, NULL, 1, NULL, 1);
    }
    if (INA_INIT_OK) {
    	xTaskCreatePinnedToCore(updateIna, "ina", 2048, NULL, 1, NULL, 1);
    }
    if (SCREEN_INIT_OK) {
	xTaskCreatePinnedToCore(updateScreen, "screen", 2048, NULL, 1, NULL, 1);
    	xTaskCreatePinnedToCore(updateButton, "button", 2048, NULL, 1, NULL, 1);
    }
    xTaskCreatePinnedToCore(updatePack, "pack_update", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updatePackIn, "pack_in", 2048, NULL, 1, NULL, 1);

}

void loop() {
	//Packetizer::parse();
    //delay(10);
}
