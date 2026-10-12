/**
 ******************************************************************************
 * @file    BMI088driver.h
 * @version V1.2.0
 * @date    2026.03.05
 * @brief   BMI088驱动模块功能声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

/* Includes ----------------------------------------------------------------- */
#include "stdint.h"
#include "main.h"

/* Defines ------------------------------------------------------------------ */
/* 功能参数 */
#define BMI088_TEMP_FACTOR 				0.125f		// 温度转换因子
#define BMI088_TEMP_OFFSET 				23.0f		// 温度偏移

/* 配置参数 */
#define BMI088_WRITE_ACCEL_REG_NUM 		6			// 加速度计写入寄存器数量
#define BMI088_WRITE_GYRO_REG_NUM 		6			// 陀螺仪写入寄存器数量
#define BMI088_LONG_DELAY_TIME 			80			// 长延时时间
#define BMI088_COM_WAIT_SENSOR_TIME 	150			// 通信等待传感器时间

/* I2C地址 */
#define BMI088_ACCEL_IIC_ADDRESSE 		(0x18 << 1)	// 加速度计I2C地址
#define BMI088_GYRO_IIC_ADDRESSE 		(0x68 << 1)	// 陀螺仪I2C地址

/* 加速度计灵敏度 */
#define BMI088_ACCEL_3G_SEN 			0.0008974358974f
#define BMI088_ACCEL_6G_SEN 			0.00179443359375f
#define BMI088_ACCEL_12G_SEN 			0.0035888671875f
#define BMI088_ACCEL_24G_SEN 			0.007177734375f

/* 陀螺仪灵敏度 */
#define BMI088_GYRO_2000_SEN 			0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 			0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 			0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 			0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 			0.000066579027251980956150958662738366f

/*/ 
#define GxOFFSET -0.000681414269f
#define GyOFFSET -0.00134240754f
#define GzOFFSET -0.00143384014f
#define AxOFFSET 0.299675316f
#define AyOFFSET 0.0720675737f
#define AzOFFSET 0.0f           // Z轴不减去零飘值,因为该零飘值含有重力加速度
?*/

/* 零漂校准值 - 可手动修改 */
#define GxOFFSET 						0.00561714312f
#define GyOFFSET 						0.00108188484f
#define GzOFFSET 						3.32896998e-05f
#define AxOFFSET 						0.215843439f
#define AyOFFSET 						-0.410339475f
#define AzOFFSET 						0.0f		// Z轴不减去零飘值,因为该零飘值含有重力加速度
#define gNORM 						  9.84484291f

/* Enums -------------------------------------------------------------------- */
/**
  * @brief BMI088错误码枚举
  */
enum
{
    BMI088_NO_ERROR = 0x00,							    // 无错误
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,				// 加速度计电源控制错误
    BMI088_ACC_PWR_CONF_ERROR = 0x02,				// 加速度计电源配置错误
    BMI088_ACC_CONF_ERROR = 0x03,					  // 加速度计配置错误
    BMI088_ACC_SELF_TEST_ERROR = 0x04,			// 加速度计自检错误
    BMI088_ACC_RANGE_ERROR = 0x05,					// 加速度计量程错误
    BMI088_INT1_IO_CTRL_ERROR = 0x06,				// 中断1 IO控制错误
    BMI088_INT_MAP_DATA_ERROR = 0x07,				// 中断映射数据错误
    BMI088_GYRO_RANGE_ERROR = 0x08,					// 陀螺仪量程错误
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,			// 陀螺仪带宽错误
    BMI088_GYRO_LPM1_ERROR = 0x0A,					// 陀螺仪低功耗模式1错误
    BMI088_GYRO_CTRL_ERROR = 0x0B,					// 陀螺仪控制错误
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,		// 中断3/4 IO配置错误
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,		// 中断3/4 IO映射错误
    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,				  // 加速度计自检失败
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,				    // 陀螺仪自检失败
    BMI088_NO_SENSOR = 0xFF,						          // 无传感器
};

/* Structs ------------------------------------------------------------------ */
/**
  * @brief IMU数据结构体
  */
typedef struct
{
    float Accel[3];			  // 加速度计数据 (m/s^2)
    float Gyro[3];			  // 陀螺仪数据 (rad/s)
    float TempWhenCali;		// 校准时温度
    float Temperature;		// 当前温度
    float AccelScale;		  // 加速度计刻度系数
    float GyroOffset[3];	// 陀螺仪零点漂移
    float AccelOffset[3];	// 加速度计零点漂移
    float gNorm;			    // 重力加速度模长
} IMU_Data_t;

/* Externs ------------------------------------------------------------------ */
extern IMU_Data_t BMI088;	// IMU数据结构体实例

/* Functions ---------------------------------------------------------------- */
void BMI088_Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate);
uint8_t BMI088_init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate);
void BMI088_Read(IMU_Data_t *bmi088);

/* -------------------------------------------------------------------------- */
#endif /* BMI088DRIVER_H */
