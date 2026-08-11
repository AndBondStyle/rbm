#include <Arduino.h>
#include <Wire.h>
//#include <SparkFun_VL53L5CX_Library.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"
//#include "encoders/esp32hwencoder/ESP32HWEncoder.h"
#include "ESP32HWEncoderFix.h"

#define MON_ENABLE

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

// ==================================== //

#define SDA_PIN 26  //21
#define SCL_PIN 25  //22
#define LPN_PIN 16
#define I2C_SPEED 1000000

// ==================================== //

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

// ==================================== //

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

    #ifdef MON_ENABLE
    motor.useMonitoring(Serial);
    motor.monitor_downsample = 1;
    motor.monitor_decimals = 2;
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    motor.monitor_start_char = c;
    #endif

    motor.target = 0.0f;
    motor.enable();
}

// ==================================== //

// ==================================== //

void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(921600);
    while (!Serial) { delay(10); };

    // SimpleFOCDebug::enable(&Serial);
    commander.verbose = VerboseMode::nothing;
    Serial.println("init start");

    //lidar_ok = tof_lidar_init();
    //if (!lidar_ok) Serial.println("lidar init failed");

    initMotorStack('A', motor_a, driver_a, sensor_a);
    initMotorStack('B', motor_b, driver_b, sensor_b);
    initMotorStack('C', motor_c, driver_c, sensor_c);
    initMotorStack('D', motor_d, driver_d, sensor_d);
    commander.add('A', onMotorA, "motor A");
    commander.add('B', onMotorB, "motor B");
    commander.add('C', onMotorC, "motor C");
    commander.add('D', onMotorD, "motor D");
}

uint32_t last_ts = 0;

void loop() {
    motor_a.move();
    motor_b.move();
    motor_c.move();
    motor_d.move();
    commander.run();

    uint32_t now = micros();
    if ((now - last_ts >= 10000) || (now < last_ts)) {
        motor_a.monitor();
        motor_b.monitor();
        motor_c.monitor();
        motor_d.monitor();
        last_ts = now;
    }

    // update_lidar();
}
