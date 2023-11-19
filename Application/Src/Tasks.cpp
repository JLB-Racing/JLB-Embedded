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

extern uint32_t adc_values_raw[8];
extern ADC_HandleTypeDef hadc1;
extern MotorControData_s motorcontrol;

float pwm_servo_test = 0.0f;

osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes =
{ .name = "ADCTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityRealtime };

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes =
{ .name = "MainTask", .stack_size = 1024 * 6, .priority = (osPriority_t) osPriorityRealtime1 };




void RegistrateUserTasks()
{
	adcTaskHandle = osThreadNew(ADCTask, NULL, &adcTask_attributes);
	mainTaskHandle = osThreadNew(MainTask, NULL, &mainTask_attributes);

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
	jlb::Logic logic;
	for (;;)
	{
#ifdef TESTING
		LineSensorTask();
		SetSteeringAngle(pwm_servo_test);
		if(pwm_servo_test > 19.0f)
		{
			direction = 0u;
		}
		else if(pwm_servo_test < -19.0f)
		{
			direction = 1u;
		}
		if(direction == 1u)
		{
			pwm_servo_test+= 0.1f;
		}
		else
		{
			pwm_servo_test-= 0.1f;
		}
		MotorControlTask();
#else
		logic.odometry.imu_callback(0.0f);
		logic.odometry.rpm_callback(0.0f);

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
		vTaskDelay(5);
	}
}
