#ifndef __USER_SYS_CONFIG_H
#define __USER_SYS_CONFIG_H

#include "cmsis_os.h"
#include "user_sys_config.h"
#include <stm32h7xx.h>

/* ϵͳDEBUG��־λ */
#define DEBUG						1

/* ϵͳ��ʱ����ʱ��(ms) */
#define SYS_DELAY_START_TIME		5000

/* Rad ת Ang*/
#define RAD_TO_ANG					(180.f / PI)
/* Ang ת Rad */
#define ANG_TO_RAD					(PI / 180.f)
/* �����ٶ�����(m/s) */


/* ���ҹؽڵ���޽� */
#define Chassis_Leg_Angle_min		(-0.7)
#define Chassis_Leg_Angle_max		(0.7)

/* ��������㱣�� */
#define Save_Motor_Zero_flag		0

#endif
