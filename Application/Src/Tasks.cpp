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
#include "Radio.h"

extern uint32_t adc_values_raw[8];
extern ADC_HandleTypeDef hadc1;
extern MotorControData_s motorcontrol;
extern PID motorcontrol_pid;
extern analog_signals_s adc_values;
extern encoder_instance enc_instance_mot;
extern IMU_signals_s imu;
extern LineSensorData_s ls_data;
extern DistanceSensorData_s distance_sensor;
float motor_battery_voltage, lv_battery_voltage, motor_current;
float wheel_rpm;

uint8_t telemetry_delayer = 0u;

uint32_t tick_counter_main, tick_counter_main_prev;
float dt_main;

extern bool flood_arrived;
extern char pirate_from, pirate_to, pirate_next;
extern int pirate_percentage;
bool flood_active = false;
uint8_t flood_counter = 40u;

jlb::Logic logic;

#ifdef TESTING
float pwm_servo_test = 0.0f;
extern uint32_t usWidth_throttle;
#endif
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes =
{ .name = "ADCTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityRealtime };

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes =
{ .name = "MainTask", .stack_size = 1024 * 12, .priority = (osPriority_t) osPriorityRealtime7 };


osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes =
{ .name = "EncoderTask", .stack_size = 128 * 10, .priority = (osPriority_t) osPriorityRealtime2 };

osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes =
{ .name = "IMUTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityRealtime3 };

osThreadId_t LSTaskHandle;
const osThreadAttr_t LSTask_attributes =
{ .name = "LSTask", .stack_size = 128 * 8, .priority = (osPriority_t) osPriorityRealtime4 };


void RegistrateUserTasks()
{
	adcTaskHandle = osThreadNew(ADCTask, NULL, &adcTask_attributes);
	mainTaskHandle = osThreadNew(MainTask, NULL, &mainTask_attributes);
	//encoderTaskHandle = osThreadNew(Encoder_Task, NULL, &encoderTask_attributes);
	IMUTaskHandle = osThreadNew(IMUTask, NULL, &IMUTask_attributes);
	LSTaskHandle = osThreadNew(LSTask, NULL, &LSTask_attributes);

}

void ADCTask(void *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		HAL_ADC_Start_DMA(&hadc1, adc_values_raw, 8u);
		vTaskDelayUntil(&xLastWakeTime, 10u);
	}
}

void IMUTask(void *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		IMU_Task();
		vTaskDelayUntil(&xLastWakeTime, 20u);
	}
}

void LSTask(void *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint32_t tick_before, tick_after;
	for (;;)
	{
		tick_before = HAL_GetTick();
		LineSensorTask();
		tick_after = HAL_GetTick();
		vTaskDelayUntil(&xLastWakeTime, 20u);
	}
}


void MainTask(void * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		lv_battery_voltage = adc_values.lv_batt_voltage_raw / 4096.0f * 3.3f * LV_BATERY_VOLTAGE_DIVIDER * 1.04447;
		wheel_rpm = CalculateRPM();

		auto [derivative, integral, prev_error] = motorcontrol_pid.get_debug();

		logic.imu_callback(imu.roll,imu.pitch,imu.yaw,imu.acc_x, imu.acc_y, imu.acc_y);
		logic.rpm_callback(wheel_rpm);

		logic.set_detection_front( ls_data.front_detection, ls_data.front);
		logic.set_detection_rear(ls_data.rear_detection, ls_data.rear);
		logic.set_object_range(distance_sensor.distance);
		logic.set_under_gate(ls_data.front.size() >= 4u);
		logic.set_at_cross_section((getPercentageFront() > jlb::CROSS_SECTION_THRESHOLD) && (ls_data.front.size() <= 2));
		logic.set_flood(flood_active);
		logic.pirate_callback(pirate_from, pirate_to, pirate_next, pirate_percentage);
		logic.start_signal();



		auto [target_angle, target_speed] = logic.update();
		//target_angle += 0.02f;
		auto [vx_t, x_t, y_t, theta_t, distance_local] = logic.get_odometry();

		DistanceSensorTask(target_angle * -180.0f / 3.14f);


		motorcontrol.actual_velocity = vx_t;
		motorcontrol.target_velocity = target_speed;
		MotorControlTask(logic.as_state.mission);

		Measurements meas;
		meas.duty_cycle = motorcontrol.duty_cycle;
		meas.motor_current = motorcontrol.motor_current;
		meas.object_range = distance_sensor.distance;
		meas.wheel_rpm = wheel_rpm;
		meas.lv_battery_voltage = lv_battery_voltage;
	    meas.hv_battery_voltage = motorcontrol.battery_voltage;
		logic.set_measurements(meas);

		SetSteeringAngle(target_angle * -180.0f / 3.14f);


		// If flood message arrives reset counter and set flood to active
		if((flood_arrived == true) && (flood_counter > 0))
		{
			flood_active = true;
			flood_arrived = false;
			flood_counter = 40u;
		}
		//If flood message was not sent decrement counter
		else
		{
			flood_counter--;
		}
		//If decrement reaches zero flood is no longer active
		if(flood_counter == 0)
		{
			flood_active = false;
		}

		tick_counter_main_prev = tick_counter_main;
    	tick_counter_main = HAL_GetTick();
        dt_main = (((float)tick_counter_main) - ((float)(tick_counter_main_prev)));

        telemetry_delayer++;

        if(telemetry_delayer == 8u)
        {
    		logic.send_telemetry();
    		telemetry_delayer = 0u;
        }

		vTaskDelayUntil(&xLastWakeTime, 10u);
	}
}

