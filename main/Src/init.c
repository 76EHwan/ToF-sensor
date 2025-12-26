/*
 * init.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 *      Modifier: Soyeon Park
 */

#include "init.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "LSM6DS3TR.h"

#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/tim.h"

#define MAIN_MENU_CNT 8

#define CUSTOM_FLASH_SECTOR_ADDR  0x08040000  // 원하는 sector 시작 주소 (예: Bank 1, Sector 4)
#define CUSTOM_FLASH_SECTOR_SIZE  0x20000     // 128KB
//추후 flash 메모리 써주세요 참고용

void Setting() {

}



menu_t menu[] = {
		{ "2. first  ", Drive_First },
		{ "3. zyro", LSM6DS3TR_C_Init }
};




//CUSTOM_JS_C는 검증되지 않았습니다. 따라서 코드가 정상작동하는지 확인 후 해당 코드는 지워주세요.
void Button_Test() {
	int i = 0;
	while (1) {
		Custom_LCD_Printf(0, 0, "%d", i);
		uint8_t sw = Custom_Switch_Read();
		if (sw == CUSTOM_JS_R_TO_L) {
			i--;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_L_TO_R) {
			i++;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_U_TO_D) {
			i = i / 2;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_D_TO_U) {
			i = i * 2;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_C) {
			break;
			Custom_LCD_Clear();
		}
	}
}





void Init() {
	HAL_GPIO_WritePin(E3_GPIO_Port, GPIO_PIN_3, GPIO_PIN_SET);

	Button_Test();

	HAL_Delay(1000);

	while (1) {

		while (1) {

		}
	}
}
