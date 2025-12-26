/*
 * sensor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 *      Modifier: Soyeon Park
 */

#include "init.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "../../MDK-ARM/Inc/adc.h"
#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/main.h"
#include "../../MDK-ARM/Inc/tim.h"


#define ADC_RESOLUTION_MAX      65535.0f // 16-bit ADC Max Value
#define ADC_REF_VOLTAGE         3.3f     // Reference Voltage
#define BATTERY_CALIB_SLOPE     11.06f   // ax + b 에서 a (기울기)
#define BATTERY_CALIB_OFFSET    2.309f   // ax + b 에서 b (절편)

float_t batteryVolt;

uint32_t battery_adc_raw = 0;


int32_t sensorPosition[16] = {
		-30000, -26000, -22000, -18000, -14000, -10000, -6000, -2000, 2000, 6000, 10000, 14000, 18000, 22000, 26000, 30000 };



uint32_t MARK_STATE_LEFT[16] = { 0x0000, 0x0000, 0x0000, 0x8000, 0xC000, 0xE000,
		0xF000, 0xF800, 0xFC00, 0xFE00, 0xFF00, 0xFF80, 0xFFC0, 0xFFE0, 0xFFF0,
		0xFFF0 //
		};

uint32_t MARK_STATE_RIGHT[16] = { 0x0FFF, 0x07FF, 0x03FF, 0x01FF, 0x00FF,
		0x007F, 0x003F, 0x001F, 0x000F, 0x0007, 0x0003, 0x0001, 0x0000, 0x0000,
		0x0000, 0x0000 //
		};

uint32_t MARK_STATE_CENTER[16] = { 0xF000, 0xF100, 0xFC00, 0x7E00, 0x3F00,
		0x1F80, 0x0FC0, 0x07E0, 0x03F0, 0x01F8, 0x00FC, 0x007E, 0x003F, 0x001F,
		0x000F, 0x000F //
		};



__STATIC_INLINE uint32_t Battery_ADC_Read() {

	static uint32_t adcValue = 0;
	__disable_irq();
	HAL_ADC_Start(&hadc2);

	if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK) {
		adcValue = HAL_ADC_GetValue(&hadc2);

	}

	HAL_ADC_Stop(&hadc2);
	__enable_irq();
	return adcValue;
}




void Battery_LPTIM3_IRQ() {
	battery_adc_raw = Battery_ADC_Read();
    // 수식: (ADC / Max) * Ref * Slope + Offset
    batteryVolt = (((float)battery_adc_raw) / ADC_RESOLUTION_MAX) * ADC_REF_VOLTAGE * BATTERY_CALIB_SLOPE + BATTERY_CALIB_OFFSET;
}



void Battery_Test() {

}

void Battery_Start() {
	HAL_LPTIM_Counter_Start_IT(&hlptim3, 0);
}
void Battery_Stop() {

	HAL_LPTIM_Counter_Stop_IT(&hlptim3);
}
void Sensor_Start() {
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_Delay(10);
}
void Sensor_Stop() {
	HAL_TIM_Base_Stop_IT(&htim6);
	HAL_Delay(10);
}


void Sensor_Test_Menu() {

}



void TIM6_Sensor_IRQ() {
    static uint8_t i = 0;
    uint32_t rawValue = 0;

    // 1. 하드웨어 MUX 제어 (생략 없이 제공)
    HAL_ADC_Start(&hadc1);

    // ============================================================
    // [TODO 1] 노이즈 제거 (Median Filter)
    // - ADC를 3회 측정하여 중간값 추출
    // ============================================================
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        rawValue = HAL_ADC_GetValue(&hadc1); // 현재는 1회만 읽음 -> 3회로 변경 필요
    }

    HAL_ADC_Stop(&hadc1);


    // ============================================================
    // [TODO 2] 정규화 (Normalization)
    // - rawValue를 0~255 범위로 변환하여 sensorNormalized[i]에 저장
    // - 보정된 Min(Black)/Max(White) 값 활용
    // ============================================================


    // 3. 탐색 완료 시 위치 계산 (가장 중요한 부분)
    if (i == 15) {
        int32_t weightedSum = 0;
        int32_t sumOfSensorValues = 0;

        int startIdx = 0;
        int endIdx = 15;

        // ============================================================
        // [TODO 3] 윈도잉 (Windowing) 범위 설정 (★필수★)
        // 과제: '이전 positionValue'를 바탕으로 탐색할 센서의 시작(startIdx)과 끝(endIdx)을 결정하세요.
        //
        // 1. 현재 positionValue를 센서 인덱스(0~15)로 대략적으로 환산합니다. (Mapping)
        //    Hint: CenterIndex = (positionValue + Offset) / Step_Size
        // 2. CenterIndex를 기준으로 좌우 WINDOW_HALF_SIZE 만큼 범위를 설정합니다.
        // 3. 인덱스가 0보다 작거나 15보다 커지지 않도록 예외처리(Clamping) 하세요.
        //
        // 목표: 라인이 3번에 있는데 12번 센서의 노이즈를 계산에 넣지 않도록 함.
        // ============================================================


        // ============================================================
        // [TODO 4] 가중 평균 계산 (Weighted Average)
        // 과제: 위에서 구한 startIdx ~ endIdx 범위 내의 센서값만 사용하여 위치를 계산하세요.
        // 수식: Position = Sum(Val * Pos_Weight) / Sum(Val)
        // ============================================================
        for (int k = startIdx; k <= endIdx; k++) {
            // weightedSum += ...
            // sumOfSensorValues += ...
        }
        // 라인을 완전히 잃어버렸을 경우 (이전 값을 유지하거나 0으로 초기화)



    }


}
void Sensor_Test_Raw() {
    // =================================================================
    // [TODO] 센서 Raw 값 디버깅
    // -----------------------------------------------------------------
    // 1. Sensor_Start() 호출
    // 2. while 루프에서 sensorRaw[0] ~ sensorRaw[15] 값을 LCD에 출력
    //    (막대 그래프 형태나 숫자 형태로 표시하여 센서 동작 확인)
    // 3. 버튼 누르면 종료
    // =================================================================
}

void Sensor_Calibration() {
    // =================================================================
    // [TODO] 센서 캘리브레이션 (보정)
    // -----------------------------------------------------------------
    // 로봇을 손으로 좌우로 흔들며 흰색 바닥과 검은색 라인의 최대/최소값을 찾습니다.
    //
    // 1. 초기화
    // 2. 루프 1 (White 측정):
    // 3. 루프 2 (Black 측정):
    // 4. 측정된 값들을 whiteMax[], blackMax[] 전역 배열에 저장
    // =================================================================
}

void Sensor_Normalized() {
    // =================================================================
    // [TODO] 정규화된 값 확인
    // -----------------------------------------------------------------
    // 캘리브레이션 후, sensorNormalized[] 값이 0(바닥) ~ 255(라인) 사이로
    // 잘 변환되는지 LCD로 확인
    // =================================================================
}

void Sensor_State() {
    // =================================================================
    // [TODO] 센서 이진화 상태 확인 (디버깅용)
    // -----------------------------------------------------------------
    // 특정 임계값(Threshold)을 기준으로 센서가 켜졌는지(1) 꺼졌는지(0)
    // 비트마스크 형태로 LCD에 출력 (예: 0000110000000000)
    // =================================================================
}

void Position_Test() {
    // =================================================================
    // [TODO] 최종 위치 계산값 확인
    // -----------------------------------------------------------------
    // 계산된 positionValue를 LCD에 출력합니다.
    // 로봇을 라인 왼쪽/오른쪽으로 움직였을 때 값이 음수/양수로 잘 변하는지,
    // 중심에 있을 때 0에 가까운지 확인
    // =================================================================
}
