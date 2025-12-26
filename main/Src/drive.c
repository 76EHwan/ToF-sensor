/*
 * drive.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 *    Modifier: Soyeon Park
 */

#include "init.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "stdbool.h"
#include "math.h"

#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/tim.h"

// 센서 가중치 테이블
int32_t positionCenter[15] = { -28000, -24000, -20000, -16000, -12000, -8000,
		-4000, 0, 4000, 8000, 12000, 16000, 20000, 24000, 28000 };

// =============================================================================
// [TODO] 전역 변수 선언
// -----------------------------------------------------------------------------
// 1. 주행 관련 변수: 현재 속도, 목표 속도, 가속도, 감속도
// 2. PID 제어 변수: 주행용 Kp, Kd (모터 속도제어용 PID와는 별개)
// 3. 맵핑 관련 변수: 마크(Mark) 저장 배열, 거리(Length) 저장 배열, 인덱스 등
// =============================================================================

menu_t drive_menu[] = { { "1st D    ", Drive_First }, { "4.encoder",
		Encoder_Test } };

void Drive_Test_Menu() {
	// =================================================================
	// [TODO] 드라이브 메뉴 UI 루프
	// -----------------------------------------------------------------
	// 1. LCD에 메뉴 목록 출력 (drive_menu 배열 활용)
	// 2. 스위치 입력을 받아 커서 이동 및 기능 선택
	// 3. 선택 시 해당 함수 포인터 실행
	// =================================================================
}

void Save_Length(uint32_t *temp, uint32_t index) {
	// =================================================================
	// [TODO] 거리 정보 저장 (1차 주행 후 호출)
	// -----------------------------------------------------------------
	// 1차 주행에서 임시 버퍼에 저장된 마크 간의 거리(Encoder Ticks)를
	// 2차 주행에서 사용할 실제 맵 배열로 복사합니다.
	// =================================================================
}

// ★★★ 핵심: 주행 제어 인터럽트 (Control Loop) ★★★
// LPTIM5 인터럽트 CUBEMX설정필요
void Drive_LPTIM5_IRQ() {
	// =================================================================
	// [TODO]
	// -----------------------------------------------------------------
	// 가/감속 프로파일 처리:
	//    - 현재 속도 < 목표 속도: 가속도(accel) 만큼 증가
	//    - 현재 속도 > 목표 속도: 감속도(decel) 만큼 감소
	//
	// =================================================================
}

void Drive_Start() {
	HAL_LPTIM_Counter_Start_IT(&hlptim5, 0); // 이 함수의 period는 ARR값임. 32KHz이며 div 16 해서 2000Hz 500us면 적당함
}

void Drive_Stop() {
	HAL_LPTIM_Counter_Stop_IT(&hlptim5);
}

void Buzzer_Start() {
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
}

void Buzzer_Stop() {
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);

}
__STATIC_INLINE uint32_t Position_Center(int32_t position, uint16_t state) {
	UNUSED(position);
	UNUSED(state);
	return 0;
}

__STATIC_INLINE uint8_t State_Machine() {

	return 0;
}

__STATIC_INLINE uint8_t Pre_State(uint8_t mark) {
	UNUSED(mark);
	return 0;
}

void Drive_First() {

}

__STATIC_INLINE void Second_State_Machine() {

}

void Drive_Second() {

}

void Drive_Third() {

}

void Drive_Forth() {

}

void Mark_Debug() {

}
