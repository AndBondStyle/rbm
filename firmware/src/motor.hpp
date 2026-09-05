#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include "ESP32HWEncoderFix.h"
#include <SimpleDCMotor.h>

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

    	float getTarget() const {
		return target;
	}

	float getShaftVelocity() const {
		return shaft_velocity;
	}

	float getShaftAngle() const {
		return shaft_angle;
	}

	float getVoltageQ() const {
		return voltage.q;
	}
};

class Motors {
public:
	struct MotorState {
		float target = 0.0f;
		float speed = 0.0f;
		float angle = 0.0f;
		float effort = 0.0f;
	};

	Motors()
		: driver_a_(13, 27), encoder_a_(19, 18, ENCODER_CPR),
		  driver_b_(2, 4), encoder_b_(23, 5, ENCODER_CPR),
		  driver_c_(12, 17), encoder_c_(36, 35, ENCODER_CPR),
		  driver_d_(14, 15), encoder_d_(39, 34, ENCODER_CPR)
		{}

	bool initMotors() {
		MOTORS_INIT_OK_ = false;
		
		initMotorStack('A', motor_a_, driver_a_, encoder_a_);
		initMotorStack('B', motor_b_, driver_b_, encoder_b_);
		initMotorStack('C', motor_c_, driver_c_, encoder_c_);
		initMotorStack('D', motor_d_, driver_d_, encoder_d_);
		
    		MOTORS_INIT_OK_ = true;
		return MOTORS_INIT_OK_;
	}

	void updateMotors() {
    		while (true) {
			motor_a_.move();
        		motor_b_.move();
        		motor_c_.move();
        		motor_d_.move();
        		vTaskDelay(pdMS_TO_TICKS(1));
    		}
	}

	bool isInit() const {
		return MOTORS_INIT_OK_;
	}

	static void task(void* arg) {
		Motors* motors = static_cast<Motors*>(arg);
		motors->updateMotors();
	}

	void setTargets(float front_left, float front_right, float rear_left, float rear_right) {
		motor_a_.target = front_left;
		motor_b_.target = rear_left;
		motor_c_.target = rear_right;
		motor_d_.target = front_right;
	}

	void setPid(float kp, float ki, float kd, float limit, float lpf_tf) {
		CustomMotor* motors[4] = {&motor_a_, &motor_b_, &motor_c_, &motor_d_};
		for (int i = 0; i < 4; ++i) {
			motors[i]->PID_velocity.P = kp;
			motors[i]->PID_velocity.I = ki;
			motors[i]->PID_velocity.D = kd;
            		motors[i]->PID_velocity.output_ramp = limit;
            		motors[i]->LPF_velocity.Tf = lpf_tf;
		}
	}

	CustomMotor& getMotorA() {
		return motor_a_;
	}

	CustomMotor& getMotorB() {
		return motor_b_;
	}

	CustomMotor& getMotorC() {
		return motor_c_;
	}

	CustomMotor& getMotorD() {
		return motor_d_;
	}
private:
    	void initMotorStack(char c, CustomMotor& motor, DCDriver2PWM& driver, ESP32HWEncoder& encoder) {
	
		driver.voltage_power_supply = VOLTAGE;
   		driver.voltage_limit = VOLTAGE;
    		driver.pwm_frequency = PWM_FREQ;
    		driver.init();
    		encoder.init();
    		motor.linkDriver(&driver);
    		motor.linkSensor(&encoder);

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


	DCDriver2PWM driver_a_;
	ESP32HWEncoder encoder_a_;
	CustomMotor motor_a_;
	
	DCDriver2PWM driver_b_;
	ESP32HWEncoder encoder_b_;
	CustomMotor motor_b_;

	DCDriver2PWM driver_c_;
	ESP32HWEncoder encoder_c_;
	CustomMotor motor_c_;

	DCDriver2PWM driver_d_;
	ESP32HWEncoder encoder_d_;
	CustomMotor motor_d_;


	bool MOTORS_INIT_OK_ = false;
};
