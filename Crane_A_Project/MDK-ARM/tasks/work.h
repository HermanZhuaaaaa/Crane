#ifndef __WORK_H
#define __WORK_H

#include "stm32f4xx.h"                  // Device header


/**
  * @brief		���Ӹ߶ȿ���
  * @param
  * @retval
  */
void Work_Up(uint8_t addr, uint16_t length);


/**
  * @brief		������µ����ƶ�
  * @param		�����ַaddr,�߶�length
  * @retval		none
  */
void Work_Up_Low_Speed(uint8_t addr, uint16_t length);

/**
  * @brief		����ƶ�ָ������
  * @param		�����ַaddr,�����ת����dir,����length
  * @retval		none
  */
void Work_Move(uint8_t addr ,uint8_t dir,uint16_t length);

/**
  * @brief		�״�!!! ��Ȧ��⵽��ֱ�ӹ�ȡ
  * @param		none
  * @retval		none
  */
void Work_Straight_FirstTime( void);

/**
  * @brief		�״�!!!�����Ȧ���ޣ���Ȧû�У�������Ȧ��ȡ
  * @param		none
  * @retval		none
  */
void Work_OuterToInner_FirstTime(void);
/**
  * @brief		�����ĺ�!!!��Ȧ��⵽��ֱ�ӹ�ȡ
  * @param		������� Class
  * @retval		none
  */
void Work_Straight(uint8_t Class);

/**
  * @brief		�����ĺ�!!!�����Ȧ���ޣ���Ȧû�У�������Ȧ��ȡ
  * @param		������� Class
  * @retval		none
  */
void Work_OuterToInner(uint8_t Class);

/**
  * @brief		�״�!!!�������붯��
  * @param
  * @retval
  */
void Work_PullDown_FirstTime(void);

/**
  * @brief		�����ĺ�!!!�������붯��
  * @param
  * @retval
  */
void Work_PullDown(uint8_t Class,uint8_t length);

#endif
