/*
 * Tasks.c
 *
 *  Created on: Nov 9, 2023
 *      Author: horgo
 */

//#define TESTING

#include "Tasks.h"
#include "main.h"
#include "cmsis_os.h"
#include "Servo.h"
#include "MotorControl.h"
#include "JLB/logic.hxx"
#include "JLB/measurements.hxx"
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
extern IMU_signals_s imu;
extern LineSensorData_s ls_data;
extern DistanceSensorData_s distance_sensor;
float motor_battery_voltage, lv_battery_voltage, motor_current;
float wheel_rpm;

jlb::Logic logic;

#ifdef TESTING
float pwm_servo_test = 0.0f;
extern uint32_t usWidth_throttle;
#endif
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes =
{ .name = "ADCTask", .stack_size = 128 * 2, .priority = (osPriority_t) osPriorityRealtime };

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes =
{ .name = "MainTask", .stack_size = 1024 * 6, .priority = (osPriority_t) osPriorityRealtime1 };


osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes =
{ .name = "EncoderTask", .stack_size = 128 * 2, .priority = (osPriority_t) osPriorityRealtime2 };

osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes =
{ .name = "LoggerTask", .stack_size = 128 * 5, .priority = (osPriority_t) osPriorityHigh };



void RegistrateUserTasks()
{
	adcTaskHandle = osThreadNew(ADCTask, NULL, &adcTask_attributes);
	mainTaskHandle = osThreadNew(MainTask, NULL, &mainTask_attributes);
	encoderTaskHandle = osThreadNew(Encoder_Task, NULL, &encoderTask_attributes);
	loggerTaskHandle = osThreadNew(LoggerTask, NULL, &loggerTask_attributes);

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
	logic.set_states({jlb::FastState::OUT_ACCEL_ZONE});
	for (;;)
	{
		lv_battery_voltage = adc_values.lv_batt_voltage_raw / 4096.0f * 3.3f * LV_BATERY_VOLTAGE_DIVIDER * 1.04447;
		LineSensorTask();
		IMU_Task();
		DistanceSensorTask();
		wheel_rpm = CalculateRPM();

#ifdef TESTING
		SetSteeringAngle(pwm_servo_test);
		if(pwm_servo_test > 90.0f)
		{
			direction = 0u;
		}
		else if(pwm_servo_test < -90.0f)
		{
			direction = 1u;
		}
		if(direction == 1u)
		{
			pwm_servo_test+= 2.0f;
		}
		else
		{
			pwm_servo_test-= 2.0f;
		}

		motorcontrol.actual_velocity = wheel_rpm;
		motorcontrol.target_velocity = 400.0f;
		if(motorcontrol.target_velocity < 0.0f)
		{
			motorcontrol.target_velocity = 0.0f;
		}
#else

		logic.imu_callback(imu.yaw);
		logic.rpm_callback(wheel_rpm);

		std::reverse(std::begin(ls_data.front_detection), std::end(ls_data.front_detection));
		//std::reverse(std::begin(ls_data.rear_detection), std::end(ls_data.rear_detection));
		logic.set_detection_front( ls_data.front_detection, ls_data.front);
		logic.set_detection_rear(ls_data.rear_detection, ls_data.rear);
		logic.set_object_range(distance_sensor.distance);
		Measurements meas;
		meas.duty_cycle = motorcontrol.duty_cycle;
		meas.motor_current = motorcontrol.motor_current;
		meas.object_range = distance_sensor.distance;
		meas.wheel_rpm = wheel_rpm;
		logic.set_measurements(meas);
		auto [target_angle, target_speed] = logic.update();
		auto [vx_t, x_t, y_t, theta_t] = logic.get_odometry();
		motorcontrol.actual_velocity = vx_t;
		motorcontrol.target_velocity = target_speed;
		MotorControlTask();
		SetSteeringAngle(target_angle * -180.0f / 3.14f);
		logic.send_telemetry();

#endif
		vTaskSuspend(static_cast<TaskHandle_t>(mainTaskHandle));
	}
}

void LoggerTask(void *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, 50u);
	}

}

