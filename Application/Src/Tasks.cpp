/*
 * Tasks.c
 *
 *  Created on: Nov 9, 2023
 *      Author: horgo
 */

#define TESTING

#include "Tasks.h"
#include "main.h"
#include "cmsis_os.h"
#include "Servo.h"
#include "MotorControl.h"
#include "JLB/logic.hxx"
#include "LineSensor.h"
#include "ADC.h"
#include "IMU.h"
#include "DistanceSensor.h"
#include "Encoder.h"


extern uint32_t adc_values_raw[8];
extern ADC_HandleTypeDef hadc1;
extern MotorControData_s motorcontrol;
extern analog_signals_s adc_values;
extern encoder_instance enc_instance_mot;

float motor_battery_voltage, lv_battery_voltage, motor_current;

float rpm_averaging_array[5];
float averaged_rpm = 0.0f;

#ifdef TESTING
float pwm_servo_test = 0.0f;
#endif
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes =
{ .name = "ADCTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityRealtime };

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes =
{ .name = "MainTask", .stack_size = 1024 * 5, .priority = (osPriority_t) osPriorityRealtime1 };


osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes =
{ .name = "EncoderTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityRealtime2 };



void RegistrateUserTasks()
{
	adcTaskHandle = osThreadNew(ADCTask, NULL, &adcTask_attributes);
	mainTaskHandle = osThreadNew(MainTask, NULL, &mainTask_attributes);
	encoderTaskHandle = osThreadNew(Encoder_Task, NULL, &encoderTask_attributes);

}

void ADCTask(void *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		HAL_ADC_Start_DMA(&hadc1, adc_values_raw, 8u);
		vTaskDelayUntil(&xLastWakeTime, 5u);
	}
}

void MainTask(void * argument)
{
	static uint8_t direction = 1u;
	static uint8_t index = 0u;
	uint8_t i;
	jlb::Logic logic;
	for (;;)
	{
		lv_battery_voltage = adc_values.lv_batt_voltage_raw / 4096.0f * 3.3f * LV_BATERY_VOLTAGE_DIVIDER * 1.04447;
		LineSensorTask();
		MotorControlTask();
		IMU_Task();
		DistanceSensorTask();

		rpm_averaging_array[index++] = enc_instance_mot.rpm;

		if(index == 5)
		{
			index = 0;
		}

		averaged_rpm = 0.0f;
		for(i = 0 ; i < 5; ++i)
		{
			averaged_rpm += rpm_averaging_array[i] / 5.0f;
		}

		averaged_rpm *= -1.36f;

#ifdef TESTING
		SetSteeringAngle(0.0f);
		if(pwm_servo_test > 22.0f)
		{
			direction = 0u;
		}
		else if(pwm_servo_test < -22.0f)
		{
			direction = 1u;
		}
		if(direction == 1u)
		{
			pwm_servo_test+= 0.5f;
		}
		else
		{
			pwm_servo_test-= 0.5f;
		}
#else
		logic.odometry.imu_callback(0.0f);
		logic.odometry.rpm_callback(averaged_rpm);

		bool kisfaszom[32] = {false};
		std::vector<float> nagyfaszom;

		logic.controller.set_detection_front(kisfaszom, nagyfaszom);
		logic.controller.set_detection_rear(kisfaszom, nagyfaszom);

		auto [target_angle, target_speed] = logic.update();
		SetSteeringAngle(target_angle);
		motorcontrol.actual_velocity = logic.odometry.vx_t;
		motorcontrol.target_velocity = target_speed;
		MotorControlTask();

		logic.signal_sender.send_telemetry();

#endif
		vTaskSuspend(static_cast<TaskHandle_t>(mainTaskHandle));
	}
}
