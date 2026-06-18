#ifndef __QMC5883P_H
#define __QMC5883P_H

#include "stm32f10x.h"  

#define QMC5883P_ADDRESS 0x58
#define PI   3.14159265358979323846f
#define QMC5883P_FS_UT   		200.0f   
#define QMC5883P_LSB_UT (QMC5883P_FS_UT/32768.0f)


typedef struct {
    float    x_offset, y_offset, z_offset; // 硬铁偏置
    float    W[3][3];                      // 软铁校准矩阵
    double   A[10][10];           // 椭球拟合时的最小二乘累积矩阵
    float    m[3];                // 校准后的三轴磁场数据
    float    B;                   // 校准后的目标磁场模长
    uint32_t sample_count;        // 已采集的校准样本数量
} QMC5883P_Calibration_t;



/* 基础驱动 */
void QMC5883P_WriteReg(uint8_t RegAddress, uint8_t Data);
void QMC5883P_WriteRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length);
uint8_t QMC5883P_ReadReg(uint8_t RegAddress);
void QMC5883P_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint16_t Length);


uint8_t QMC5883P_GetID(void);
float EMA(float rawValue, float filteredValue);


void QMC5883P_Init(void);
void QMC5883P_GetData(int16_t *X, int16_t *Y, int16_t *Z);
void QMC5883P_GetData_uT(float *mx,float *my,float *mz);


/* 校准 */
void QMC5883P_StartCalibration(QMC5883P_Calibration_t *calib);
void QMC5883P_UpdateCalibration(QMC5883P_Calibration_t *calib);
void QMC5883P_EndCalibration(QMC5883P_Calibration_t *calib);

/* 输出 */
void QMC5883P_GetCalibData(QMC5883P_Calibration_t *cal);
void QMC5883P_GetCalibData_uT(QMC5883P_Calibration_t *cal);  

#endif

