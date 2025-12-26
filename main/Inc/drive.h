/*
 * drive.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_DRIVE_H_
#define MAIN_INC_DRIVE_H_
#include <stdint.h>
#include <stdbool.h>

#include "../../MDK-ARM/Inc/main.h"
void Drive_First(void);
void Drive_Second(void);
void Drive_Third(void);
void Drive_Forth(void);
void Drive_Start();
void Drive_Test_Without_Motor();
void Drive_LPTIM5_IRQ();
void First_Drive_Mark_Debug();
void Drive_Test_Menu();
void Drive_Second();



#endif /* MAIN_INC_DRIVE_H_ */

