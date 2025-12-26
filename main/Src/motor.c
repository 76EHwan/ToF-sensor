/*
 * motor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 *      Modifier: Soyeon Park
 */

#include "motor.h"
#include "math.h"
#include "drive.h"
#include "sensor.h"
#include "init.h"
#include <stdbool.h>
#include "dwt_delay.h"
#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/gpio.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/tim.h"

// =============================================================================
// [TODO: 하드웨어 파라미터 확인 및 수정]
// -----------------------------------------------------------------------------
// 실제 로봇의 사양에 맞춰 아래 값들을 수정하세요.
// =============================================================================
#define WHEEL_DIA 0.035f          // 바퀴 지름 (Meter 단위, 예: 3.5cm = 0.035)
#define RADIUS (WHEEL_DIA/2.0f)   // 바퀴 반지름
#define GEAR_RATIO (63.0f/17.0f)  // 기어비 (모터 축 회전수 : 바퀴 회전수)
#define ENCODER_PPR 4096.0f       // Pulse Per Revolution (엔코더 분해능)

// 제어 주기 (LPTIM4의 인터럽트 주기와 일치해야 함. )
#define CONTROL_PERIOD

// 물리량 변환 상수
#define PI 3.141592f
// 엔코더 1틱당 이동 거리 (m) = (바퀴둘레) / (기어비 * 분해능)
#define METER_PER_TICK ((PI * WHEEL_DIA) / (ENCODER_PPR * GEAR_RATIO))

// 전압 제한 (Safety)
#define BATTERY_VOLTAGE    // 현재 배터리 전압 (가변적일 수 있음)
#define MOTOR_MAX_VOLT    // 모터에 인가할 최대 허용 전압

// =============================================================================
// 하드웨어 핸들러 매핑 (CubeMX 설정에 따름)
// =============================================================================
#define MOTOR_TIM		     &htim8
#define MOTORL_CHANNEL	     TIM_CHANNEL_2
#define MOTORR_CHANNEL	     TIM_CHANNEL_1
#define MOTORL_ENCODER_TIMER &hlptim2
#define MOTORR_ENCODER_TIMER &hlptim1

#define MOTORL_PH_GPIO_Port  GPIOD
#define MOTORL_PH_Pin        GPIO_PIN_14
#define MOTORR_PH_GPIO_Port  GPIOA
#define MOTORR_PH_Pin        GPIO_PIN_9

// 구조체 인스턴스 선언 (Motor_t 타입 사용)
Motor_t MotorL;
Motor_t MotorR;

// 유틸리티 매크로
#define ABS(x) ((x>0) ? (x) : (-(x)))
#define MIN(a, b) ((a > b) ? (b) : (a))

void Motor_Test() {
    // [TODO 11] 오픈 루프 테스트 (Open Loop Control)
    // PID 제어 없이, PWM을 직접 인가하여 모터와 드라이버의 건전성을 확인합니다.

    // 1. Motor_Start()를 호출하여 드라이버를 초기화합니다.
    // 2. PWM Duty를 약 30~50% 정도로 강제 설정합니다.
    //    예: __HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORR_CHANNEL, 500);
    // 3. 바퀴가 지정된 방향으로 잘 도는지 눈으로 확인합니다.
    // 4. 버튼을 누르면 Motor_Stop()을 호출하고 종료합니다.

    // (참고: Start 함수가 없다면 아래와 같이 직접 초기화 코드를 작성해볼 수도 있습니다.)
    // DRVOFF = 0, nSLEEP = 1 설정 등...
}

void Motor_Start() {
	// 1. 변수 초기화 (P, I 게인 등 0으로 초기화) 추후, 튜닝한 gain값 사용
	MotorL.Kp = 0.0f; MotorL.Ki = 0.0f;
	MotorR.Kp = 0.0f; MotorR.Ki = 0.0f;

	// =================================================================
	// [TODO 1] 엔코더 카운팅 시작
	// -----------------------------------------------------------------
	// 좌/우 모터의 엔코더 타이머(LPTIM1, LPTIM2)를 시작하세요.
	// Period는 최대값(0xFFFF)으로 설정합니다.
	// =================================================================


	// =================================================================
	// [TODO 2] 모터 드라이버 GPIO 초기 상태 설정
	// -----------------------------------------------------------------
	// 1. DRVOFF 핀: 모터 출력을 활성화(Enable) 하기 위해 Low(0)로 설정
	// 2. nSLEEP 핀: 모터 드라이버를 깨우기(Active) 위해 High(1)로 설정
	// 3. 전압 안정화를 위해 잠시 대기 (HAL_Delay 1ms)
	// =================================================================


	// =================================================================
	// [TODO 3] 모터 드라이버 초기화 및 Fault Clear 시퀀스 (★중요★)
	// -----------------------------------------------------------------
	// 모터 드라이버(H-Bridge)가 켜질 때, nFAULT 핀이 Low(에러 상태)로 시작하는 경우가 있습니다.
	// 이를 정상 상태(High)로 만들기 위해서는 nSLEEP 핀을 잠시 내렸다가(Low) 올려야(High) 합니다.
	// 데이터시트 요구사항: Reset Pulse 폭은 20us 이상 40us 미만이어야 합니다.
	//
	// <구현 가이드>
	// 1. nFAULT 핀이 Low인 동안 루프를 돕니다 (while 문):
	//    a. nSLEEP 핀을 Low로 내림
	//    b. 30us 동안 대기 (DWT_Delay_us 함수 사용 권장)
	//    c. nSLEEP 핀을 다시 High로 올림
	//    d. 안정화를 위해 1ms 대기
	// 2. 루프 탈출 후 디버깅용 LED 제어 등 마무리
	// =================================================================


	// =================================================================
	// [TODO 4] PWM 타이머 시작
	// -----------------------------------------------------------------
	// 1. TIM8번의 채널 1, 2번을 인터럽트 모드(PWM Start IT)로 시작하세요.
	// 2. 시작과 동시에 급발진하지 않도록 Compare(Duty) 값을 0으로 초기화하세요.
	// =================================================================


	// =================================================================
	// [TODO 5] 제어 루프 타이머 시작
	// -----------------------------------------------------------------
	// 정해진 주기(CONTROL_PERIOD)마다 PID 제어 함수(Motor_LPTIM4_IRQ)가
	// 실행되도록 LPTIM4 타이머를 인터럽트 모드로 시작하세요.
	// =================================================================

}

void Motor_Stop() {
	// =================================================================
	// [TODO] 모터 정지 및 안전 조치
	// -----------------------------------------------------------------
	// 1. 모터 드라이버를 비활성화 하세요 (DRVOFF -> High, nSLEEP -> Low)
	// 2. 모든 타이머(PWM, Encoder, Control Loop)를 중지하세요.
	// =================================================================
}


// ★★★ 핵심 제어 루프 (Interrupt Service Routine) ★★★
// CONTROL_PERIOD(예: 0.5ms) 마다 실행됨
void Motor_LPTIM4_IRQ() {

	// 1. 엔코더 값 읽기 (CNT 레지스터 직접 접근)
	// LPTIM은 16비트 카운터이므로 오버플로우를 고려해 uint16_t로 읽습니다.
	// (참고: 오른쪽 모터 장착 방향에 따라 부호를 반대로(-) 붙여야 할 수 있음)

	// ============================================================
	// [TODO 6] 속도 계산 (Feedback)
	// ------------------------------------------------------------
	// 1. 현재 엔코더(current)와 이전 엔코더(prev)의 차이(Diff)를 구하세요. (overflow 주의)
	// 2. Diff를 물리적 속도(m/s)로 변환하세요.
	//    수식: currentSpeed = (Diff * METER_PER_TICK) / CONTROL_PERIOD
	// 3. 현재 값을 과거 값 변수(prevEncoder)에 저장하세요.
	// ============================================================


	// ============================================================
	// [TODO 7] PID 제어기 구현
	// ------------------------------------------------------------
	// 1. 속도 오차(Error) 계산: targetSpeed - currentSpeed
	// 2. P(비례)항 계산: Error * Kp
	// 3. I(적분)항 계산: Error 누적 * Ki * CONTROL_PERIOD (Integral 변수 사용)
	// 4. 필요 전압(controlVoltage) 계산: P항 + I항
	// ============================================================


	// ============================================================
	// [TODO 8] 모터 구동 및 PWM 출력
	// ------------------------------------------------------------
	// 1. controlVoltage를 배터리 전압(BATTERY_VOLTAGE) 범위 내로 제한(Clamping)
	// 2. 전압의 부호(+/-)에 따라 방향 핀(PH Pin) High/Low 제어
	// 3. 전압의 크기(절대값)를 PWM Duty(0 ~ ARR)로 변환하여 타이머에 설정
	//    수식: Duty = (Voltage / Battery_Voltage) * Timer_Period
	// ============================================================

}

void Encoder_Test() {
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	uint8_t sw = 0;
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "L %5d", LPTIM2->CNT);
		Custom_LCD_Printf(0, 2, "R %5u", LPTIM1->CNT);
		if (sw == CUSTOM_JS_L_TO_R) {
			break;
		}

	}
}

// === 게인 튜닝 함수들 ===

void Motor_Left_Gain_P() {
    // [TODO 9] P 게인 튜닝
    // 스위치 입력으로 MotorL.Kp 값을 조절하고 LCD에 출력하세요.
}

void Motor_Left_Gain_I() {
    // [TODO 10] I 게인 튜닝
    // 스위치 입력으로 MotorL.Ki 값을 조절하고 LCD에 출력하세요.
}

void Motor_Right_Gain_P() {
    // 오른쪽 모터 P 게인 조절
}

void Motor_Right_Gain_I() {
    // 오른쪽 모터 I 게인 조절
}

