/*
 * vl53l0x_platform.c
 * STM32 HAL Driver Implementation
 */

#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "stm32h7xx_hal.h"  // H7 시리즈용 HAL 헤더 (사용 MCU에 맞게 변경)

// main.c 등에서 정의된 I2C 핸들을 가져옵니다.
// 만약 이름이 hi2c1이 아니라면 CubeMX 설정에 맞게 변경하세요.
extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_I2C_HANDLE &hi2c1  // 주석 해제 후 사용하거나, 아래 코드에서 직접 &hi2c1 사용

// I2C 타임아웃 (ms)
#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

// --- 필수 구현 함수들 ---

// 1. 데이터 여러 바이트 쓰기
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    // VL53L0X_Dev_t 구조체에서 I2C 주소 가져오기
    uint8_t deviceAddress = Dev->I2cDevAddr;

    // HAL I2C 메모리 쓰기 함수 호출
    // (핸들, 주소, 레지스터주소, 주소크기(1Byte), 데이터포인터, 데이터길이, 타임아웃)
    if (HAL_I2C_Mem_Write(&hi2c1, deviceAddress, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 1000) != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return VL53L0X_ERROR_NONE;
}

// 2. 데이터 여러 바이트 읽기
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    uint8_t deviceAddress = Dev->I2cDevAddr;

    if (HAL_I2C_Mem_Read(&hi2c1, deviceAddress, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 1000) != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return VL53L0X_ERROR_NONE;
}

// 3. 1바이트 쓰기 Wrapper
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

// 4. 2바이트(Word) 쓰기 Wrapper (Big Endian 처리 필요할 수 있음 -> I2C_Mem_Write가 자동 처리하거나 배열로 변환)
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    uint8_t buff[2];
    buff[0] = (data >> 8) & 0xFF; // MSB
    buff[1] = data & 0xFF;        // LSB
    return VL53L0X_WriteMulti(Dev, index, buff, 2);
}

// 5. 4바이트(DWord) 쓰기 Wrapper
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    uint8_t buff[4];
    buff[0] = (data >> 24) & 0xFF;
    buff[1] = (data >> 16) & 0xFF;
    buff[2] = (data >> 8) & 0xFF;
    buff[3] = data & 0xFF;
    return VL53L0X_WriteMulti(Dev, index, buff, 4);
}

// 6. 1바이트 읽기 Wrapper
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

// 7. 2바이트(Word) 읽기 Wrapper
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
    uint8_t buff[2];
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buff, 2);
    if(status == VL53L0X_ERROR_NONE){
        *data = (buff[0] << 8) | buff[1]; // MSB First
    }
    return status;
}

// 8. 4바이트(DWord) 읽기 Wrapper
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
    uint8_t buff[4];
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buff, 4);
    if(status == VL53L0X_ERROR_NONE){
        *data = (buff[0] << 24) | (buff[1] << 16) | (buff[2] << 8) | buff[3];
    }
    return status;
}

// 9. Read-Modify-Write (UpdateByte)
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t data;

    status = VL53L0X_RdByte(Dev, index, &data);
    if (status != VL53L0X_ERROR_NONE) return status;

    data = (data & AndData) | OrData;
    return VL53L0X_WrByte(Dev, index, data);
}

// 10. 딜레이 함수 (밀리초 단위)
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    HAL_Delay(1); // 1ms 딜레이 (상황에 따라 늘려야 할 수도 있음)
    return VL53L0X_ERROR_NONE;
}

// 11. Lock/Unlock (싱글 스레드 환경이면 빈 함수로 둬도 됨)
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev) {
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev) {
    return VL53L0X_ERROR_NONE;
}
