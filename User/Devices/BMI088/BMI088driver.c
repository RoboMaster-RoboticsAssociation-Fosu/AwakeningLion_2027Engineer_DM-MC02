/**
 ******************************************************************************
 * @file    BMI088driver.c
 * @version V1.2.0
 * @date    2026.03.05
 * @brief   BMI088驱动模块功能实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include <math.h>
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "bsp_dwt.h"

/* Defines ----------------------------------------------------------------- */
#if defined(BMI088_USE_SPI)
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

#elif defined(BMI088_USE_IIC)

#endif

/* Global variable --------------------------------------------------------- */
float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;   // 加速度计灵敏度
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;   // 陀螺仪灵敏度
IMU_Data_t BMI088;                              // IMU数据结构体

static uint8_t res = 0;                         // 单个寄存器读取结果
static uint8_t write_reg_num = 0;               // 多寄存器写入计数
static uint8_t error = BMI088_NO_ERROR;         // 错误码
static float gyroDiff[3];                       // 陀螺仪校准数据差异
static float gNormDiff;                         // 加速度模长校准数据差异
static uint8_t caliOffset = 1;                  // 校准标志
static int16_t caliCount = 0;                   // 校准计数

// 加速度计初始化配置表
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

// 陀螺仪初始化配置表
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

/* Static Fun -------------------------------------------------------------- */
#if defined(BMI088_USE_SPI)
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
#elif defined(BMI088_USE_IIC)

#endif
static void Calibrate_MPU_Offset(IMU_Data_t *bmi088);
static uint8_t bmi088_accel_init(void);
static uint8_t bmi088_gyro_init(void);


/* Functions --------------------------------------------------------------- */
/**
  * @brief  BMI088初始化函数(循环尝试直到成功)
  * @param  bmi088_SPI: SPI句柄
  * @param  calibrate: 是否进行校准 (1:是, 0:否)
  * @return 无
  * @note   无
  */
void BMI088_Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    while (BMI088_init(bmi088_SPI, calibrate))
    {
        ;
    }
}

/**
  * @brief  BMI088初始化函数
  * @param  bmi088_SPI: SPI句柄
  * @param  calibrate: 是否进行校准 (1:是, 0:否)
  * @return uint8_t: 初始化错误码
  * @note   无
  */
uint8_t BMI088_init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    BMI088_SPI = bmi088_SPI;
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    if (calibrate)
	{
		//需要校准，进入这里，imu校准的时候要保持水平静止不动
		Calibrate_MPU_Offset(&BMI088);
	}      
    else
    {
        BMI088.GyroOffset[0] = GxOFFSET;
        BMI088.GyroOffset[1] = GyOFFSET;
        BMI088.GyroOffset[2] = GzOFFSET;
			
		BMI088.AccelOffset[0]= AxOFFSET;
		BMI088.AccelOffset[1]= AyOFFSET;
		BMI088.AccelOffset[2]= AzOFFSET;
			
        BMI088.gNorm = gNORM;
        BMI088.AccelScale = 9.81f / BMI088.gNorm;
        BMI088.TempWhenCali = 40;
    }

    return error;
}

/**
  * @brief  读取BMI088传感器数据
  * @param  bmi088: 指向IMU数据结构体的指针
  * @return 无
  * @note   无
  */
void BMI088_Read(IMU_Data_t *bmi088)
{
    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    static int16_t bmi088_raw_temp;
    
	//读取三轴加速度
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
	if (caliOffset)
    {
		bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
		bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale-bmi088->AccelOffset[0];
		bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
		bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale-bmi088->AccelOffset[1];
		bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
		bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;//z轴先不减去零飘值，因为该零飘值含有重力加速度
	}
	else
	{
		bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
		bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
		bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
		bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
		bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
		bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;	
	}
		
	//读取三轴角速度
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* Private functions ------------------------------------------------------- */
/**
  * @brief  校准MPU零点漂移
  * @param  bmi088: 指向IMU数据结构体的指针
  * @return 无
  * @note   校准时需保持IMU水平静止
  */
static void Calibrate_MPU_Offset(IMU_Data_t *bmi088)
{
    uint64_t start_time_us;
    static uint16_t CaliTimes = 20000; // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
	static uint16_t acc_CaliTimes = 20000; // 需要足够多的数据才能得到有效加速度计零偏校准结果
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;

    start_time_us = DWT_GetTimeUs();
    do
    {
        if ((DWT_GetTimeUs() - start_time_us) > 10000000ULL)
        {
            // 校准超时
            bmi088->GyroOffset[0] = GxOFFSET;
            bmi088->GyroOffset[1] = GyOFFSET;
            bmi088->GyroOffset[2] = GzOFFSET;
            bmi088->gNorm = gNORM;
            bmi088->TempWhenCali = 40;
            break;
        }

        DWT_Delay(0.005);
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);
            bmi088->gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];//累积零飘值
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
                }
            }
            else
            {//更新最大最小值
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (bmi088->Gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->Gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                break;
            DWT_Delay(0.0005);
        }

        // 取平均值得到标定结果
        bmi088->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; i++)
            bmi088->GyroOffset[i] /= (float)CaliTimes;

        // 记录标定时IMU温度
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;
    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);

    // 根据标定结果校准加速度计标度因数
    bmi088->AccelScale = 9.81f / bmi088->gNorm;
			
	for (uint16_t i = 0; i < acc_CaliTimes; i++)
    {
		     
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
		bmi088->AccelOffset[0]=bmi088->AccelOffset[0]*0.8f+bmi088->Accel[0]*0.2f;//加速度一阶低通滤波
			 
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
		bmi088->AccelOffset[1]=bmi088->AccelOffset[1]*0.8f+bmi088->Accel[1]*0.2f;//加速度一阶低通滤波
			 
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
		bmi088->AccelOffset[2]=bmi088->AccelOffset[2]*0.8f+bmi088->Accel[2]*0.2f;//加速度一阶低通滤波
		  	 
		DWT_Delay(0.0005);		

	}
    //  bmi088->AccelOffset[0]= AxOFFSET;
    //	bmi088->AccelOffset[1]= AyOFFSET;
    //	bmi088->AccelOffset[2]=0.0f; 
}

/**
  * @brief  BMI088加速度计初始化
  * @param  无
  * @return uint8_t: 初始化错误码
  * @note   无
  */
static uint8_t bmi088_accel_init(void)
{
    // 检查通信
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // 加速度计软件复位
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // 复位后检查通信是否正常
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // 检查 "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    // do
    // {
    //     BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    // } while (res != BMI088_ACC_CHIP_ID_VALUE);

    // 设置加速度计传感器配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            // write_reg_num--;
            // return write_BMI088_accel_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
  * @brief  BMI088陀螺仪初始化
  * @param  无
  * @return uint8_t: 初始化错误码
  * @note   无
  */
static uint8_t bmi088_gyro_init(void)
{
    // 检查通信
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // 陀螺仪软件复位
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);
    // 复位后检查通信是否正常
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // 检查 "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // 设置陀螺仪传感器配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            write_reg_num--;
            // return write_BMI088_gyro_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

#if defined(BMI088_USE_SPI)
/**
  * @brief  SPI写入单个寄存器
  * @param  reg: 寄存器地址
  * @param  data: 写入的数据
  * @return 无
  * @note   无
  */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

/**
  * @brief  SPI读取单个寄存器
  * @param  reg: 寄存器地址
  * @param  return_data: 存储读取数据的指针
  * @return 无
  * @note   无
  */
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

/**
  * @brief  SPI读取多个寄存器
  * @param  reg: 起始寄存器地址
  * @param  buf: 存储读取数据的缓冲区
  * @param  len: 读取长度
  * @return 无
  * @note   无
  */
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

// static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
// {
//      BMI088_read_write_byte( reg );
//      while( len != 0 )
//      {

//         BMI088_read_write_byte( *buf );
//         buf ++;
//         len --;
//     }

// }
#elif defined(BMI088_USE_IIC)

#endif

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
