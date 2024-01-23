/*
 * Tasks.h
 *
 *  Created on: Nov 9, 2023
 *      Author: horgo
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

void RegistrateUserTasks();

void ADCTask(void *argument);
void IMUTask(void *argument);
void LSTask(void *argument);
void TelemetryTask(void *argument);
void MainTask(void * argument);
void LoggerTask(void *argument);
#endif /* INC_TASKS_H_ */
