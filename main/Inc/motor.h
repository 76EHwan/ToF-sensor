/*
 * motor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include <stdint.h>

#include "../../MDK-ARM/Inc/main.h"
#ifndef MAIN_INC_MOTOR_H_
#define MAIN_INC_MOTOR_H_
// ==============================================================================
// 모터 제어 구조체 (Motor Control Struct)
// 역할: 센서값 입력, 목표 설정, PID 연산, 최종 출력까지의 모든 상태를 저장
// ==============================================================================
typedef struct {
    // --------------------------------------------------------------------------
    // 1. 센싱 데이터 (Sensing): 모터의 현재 상태를 읽어오는 변수들
    // --------------------------------------------------------------------------
    volatile uint16_t currentEncoder;   // 현재 엔코더 카운터 값 (Timer CNT 레지스터)
    volatile uint16_t prevEncoder;      // 바로 전 제어 주기(1ms 전)의 엔코더 값
    volatile int16_t  encoderDiff;      // [현재 - 과거] 펄스 차이 (속도 계산의 기초 데이터)

    volatile float    currentSpeed;     // 엔코더 변화량을 물리적 속도(m/s)로 변환한 값
    volatile int32_t  totalTicks;       // 주행 시작 후 누적된 총 펄스 수 (거리 측정용)

    // --------------------------------------------------------------------------
    // 2. 제어 목표 (Target): 우리가 도달하고자 하는 값
    // --------------------------------------------------------------------------
    volatile float    targetSpeed;      // 사용자가 설정한 목표 속도 (m/s)

    // --------------------------------------------------------------------------
    // 3. PID 제어 변수 (Control): 오차를 줄이기 위해 계산하는 변수들
    // --------------------------------------------------------------------------
    volatile float    speedError;       // 속도 오차 (목표 속도 - 현재 속도)
    volatile float    errorIntegral;    // 오차의 누적값 (I제어: 미세한 오차 보정용)

    volatile float    Kp;               // P(비례) 게인: 반응 속도 조절
    volatile float    Ki;               // I(적분) 게인: 잔류 오차 제거

    // --------------------------------------------------------------------------
    // 4. 모터 출력 (Output): 계산된 결과를 실제 하드웨어로 보내는 값
    // --------------------------------------------------------------------------
    volatile float    controlVoltage;   // PID 계산 결과로 나온 필요 전압 (V)
    volatile int32_t  pwmDuty;          // 모터 드라이버에 전달할 최종 PWM 듀티 (0 ~ ARR)

} Motor_t;

// 좌우 모터 인스턴스 (전역 변수)
extern Motor_t MotorL;
extern Motor_t MotorR;

// 함수 프로토타입 선언
void Motor_Start(void);           // 모터 및 타이머 초기화
void Motor_Stop(void);            // 모터 정지
void Motor_LPTIM4_IRQ(void);      // PID 제어 루프 (인터럽트 핸들러)

// 테스트 및 튜닝 함수
void Motor_Test_Menu(void);       // 모터 테스트 메뉴 진입
void Encoder_Test(void);          // 엔코더 동작 확인
void Motor_Test_OpenLoop(void);   // PID 없이 PWM만 줘서 모터 구동 확인 (Open Loop)

// 게인 튜닝 함수
void Motor_Left_Gain_P(void);
void Motor_Left_Gain_I(void);
void Motor_Right_Gain_P(void);
void Motor_Right_Gain_I(void);
void Encoder_Test();
void Motor_Start();
void Motor_Stop();

void Motor_Right_Gain_P();
void Motor_Right_Gain_I();
void Motor_Left_Gain_P();
void Motor_Left_Gain_I();
void  Motor_Speed_Test();
#endif /* MAIN_INC_MOTOR_H_ */
