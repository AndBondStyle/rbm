#pragma once

#include <Arduino.h>

#define PACKETIZER_USE_INDEX_AS_DEFAULT
#define PACKETIZER_USE_CRC_AS_DEFAULT
#include <Packetizer.h>

#include "motor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "batt.hpp"
#include "screen.hpp"

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

struct __attribute__((packed)) MotStat {
	float target;
	float shaft_velocity;
	float shaft_angle;
	float voltage_q;
};

struct __attribute__((packed)) BatStat {
	float voltage;
	float current;
	float percent;
};

struct __attribute__((packed)) StatusPayloadOut {
	uint32_t flags;
	MotStat front_left_motor_stat; //0 - target, 1 - shaft_velocity, 2 - shaft_angle, 3 - voltage.q
	MotStat front_right_motor_stat;
	MotStat rear_left_motor_stat;
	MotStat rear_right_motor_stat;
	BatStat bat_status; // 0 - voltage, 1 - current, 2 - percent
};

struct __attribute__((packed)) Coord {
	float x;
	float y;
	float z;
};

struct __attribute__((packed)) CoordQuat {
	float w;
	float x;
	float y;
	float z;
};


struct __attribute__((packed)) ImuPayloadOut {
	Coord acc; // XYZ
	Coord gyr;
	CoordQuat quat;
};

#define TOF_ZONES 64
struct __attribute__((packed)) TofPayloadOut {
	int16_t distance_mm[TOF_ZONES];
	uint8_t reflectance[TOF_ZONES];
	uint8_t status[TOF_ZONES];
};

#define MAX_PAYLOAD_SIZE sizeof(TofPayloadOut)

class Proto {
public:
	Proto(HardwareSerial& serial, Motors& motors, Imu& imu, Tof& tof, Ina& ina, Screen& screen) : serial_(serial), motors_(motors), imu_(imu), tof_(tof), ina_(ina), screen_(screen) {
		
	}

	bool initProto() {
		PROTO_INIT_OK_ = false;

		Packetizer::subscribe(serial_, MSG_MOT_IN, [this](const uint8_t* data, const size_t size) {
			if (size != sizeof(ControlPayloadIn)) return;
        		memcpy(&control_pack_in_, data, sizeof(control_pack_in_));
    			motors_.setTargets(
                    		control_pack_in_.front_left_speed,
                    		control_pack_in_.front_right_speed,
                    		control_pack_in_.rear_left_speed,
                    		control_pack_in_.rear_right_speed
                	);
		});

    		Packetizer::subscribe(serial_, MSG_PID_CONF, [this](const uint8_t* data, const size_t size) {
			if (size != sizeof(PIDConfigIn)) return;
        		memcpy(&pid_config_in_, data, sizeof(pid_config_in_));
        		motors_.setPid(
                   		pid_config_in_.kp,
                    		pid_config_in_.ki,
                    		pid_config_in_.kd,
                    		pid_config_in_.limit,
                    		pid_config_in_.lpf_tf
                	);
    		});

		PROTO_INIT_OK_ = true;
		return PROTO_INIT_OK_;
	}

	void updatePack() {
		uint32_t lastStatus = 0;
		uint32_t lastImu = 0;
		uint32_t lastTof = 0;
		while (true) {
			uint32_t now = millis();

			if (now - lastStatus >= 100) { 
				lastStatus = now;
        			updateStatusPacket();
				sendPacket(MSG_STAT, &status_pack_out_, sizeof(status_pack_out_));      
        
			}
			if (imu_.isInit() && now - lastImu >= 20) {
		 		lastImu = now;
				updateImuPacket();
		 		sendPacket(MSG_IMU, &imu_pack_out_, sizeof(imu_pack_out_));
			}

			if (tof_.isInit() && now - lastTof >= 67) {
		 		lastTof = now;
				updateTofPacket();
		 		sendPacket(MSG_TOF, &tof_pack_out_, sizeof(tof_pack_out_));
			}

		vTaskDelay(pdMS_TO_TICKS(10));
		}
	}

	void updatePackIn() {
		while(true) {
			Packetizer::parse();
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}

	bool isInit() const {
		return PROTO_INIT_OK_;
	}

	static void packTask(void* arg) {
		Proto* proto = static_cast<Proto*>(arg);
		proto->updatePack();
	}

	static void packInTask(void* arg) {
		Proto* proto = static_cast<Proto*>(arg);
		proto->updatePackIn();
	}
private:
	void sendPacket(uint8_t msg_type, const void* payload, size_t payloadSize) {
		//uint8_t frame[MAX_PAYLOAD_SIZE];

		if (payloadSize > MAX_PAYLOAD_SIZE) {
			return;
		}

	//frame[0] = msg_type;
	//memcpy(frame + 1, payload, payloadSize);

		Packetizer::send(serial_, msg_type, (const uint8_t*)payload, payloadSize);
	}
	
	void updateStatusPacket() {
		status_pack_out_.flags = (screen_.isInit() << 0) | (ina_.isInit() << 1) | (tof_.isInit() << 2) | (imu_.isInit() << 3);

		const auto& motor_a = motors_.getMotorA();
		const auto& motor_b = motors_.getMotorB();
		const auto& motor_c = motors_.getMotorC();
		const auto& motor_d = motors_.getMotorD();

		status_pack_out_.front_left_motor_stat = MotStat{
				motor_a.getTarget(),
				motor_a.getShaftVelocity(),
				motor_a.getShaftAngle(),
				motor_a.getVoltageQ()
		};
		status_pack_out_.front_right_motor_stat = MotStat{
				motor_d.getTarget(),
				motor_d.getShaftVelocity(),
				motor_d.getShaftAngle(),
				motor_d.getVoltageQ()
		};
		status_pack_out_.rear_left_motor_stat = MotStat{
				motor_b.getTarget(),
				motor_b.getShaftVelocity(),
				motor_b.getShaftAngle(),
				motor_b.getVoltageQ()
		};
		status_pack_out_.rear_right_motor_stat = MotStat{
				motor_c.getTarget(),
				motor_c.getShaftVelocity(),
				motor_c.getShaftAngle(),
				motor_c.getVoltageQ()
		};
		

		const auto& bat = ina_.getState();

		status_pack_out_.bat_status = BatStat{
				bat.voltage,
				bat.current,
				bat.percent
		};
	}

	void updateImuPacket() {
		const auto& acc = imu_.getAcc();
		const auto& gyr = imu_.getGyr();
		const auto& quat = imu_.getQuat();
		imu_pack_out_.acc = {
        		acc.x,
        		acc.y,
        		acc.z
    		};

    		imu_pack_out_.gyr = {
        		gyr.x,
        		gyr.y,
        		gyr.z
    		};

    		imu_pack_out_.quat = {
        		quat.w,
        		quat.x,
        		quat.y,
        		quat.z
    		};
	}

	void updateTofPacket() {
		const auto& tofState = tof_.getState();

		for (int i = 0; i < TOF_ZONES; ++i) {
			tof_pack_out_.distance_mm[i] = tofState.distance_mm[i];
			tof_pack_out_.reflectance[i] = tofState.reflectance[i];
			tof_pack_out_.status[i] = tofState.status[i];
		}
	}

	HardwareSerial& serial_;

    	Motors& motors_;
    	Imu& imu_;
    	Tof& tof_;
    	Ina& ina_;
    	Screen& screen_;


    	ControlPayloadIn control_pack_in_{};
    	PIDConfigIn pid_config_in_{};

    	StatusPayloadOut status_pack_out_{};
    	ImuPayloadOut imu_pack_out_{};
    	TofPayloadOut tof_pack_out_{};

    	bool PROTO_INIT_OK_ = false;
};
