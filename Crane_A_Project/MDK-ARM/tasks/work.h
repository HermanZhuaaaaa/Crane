#ifndef __WORK_H
#define __WORK_H

#include "stm32f4xx.h"                  // Device header


/**
  * @brief		钩子高度控制
  * @param
  * @retval
  */
void Work_Up(uint8_t addr, uint16_t length);


/**
  * @brief		电机上下低速移动
  * @param		电机地址addr,高度length
  * @retval		none
  */
void Work_Up_Low_Speed(uint8_t addr, uint16_t length);

/**
  * @brief		电机移动指定距离
  * @param		电机地址addr,电机旋转方向dir,长度length
  * @retval		none
  */
void Work_Move(uint8_t addr ,uint8_t dir,uint16_t length);

/**
  * @brief		首次!!! 外圈检测到，直接钩取
  * @param		none
  * @retval		none
  */
void Work_Straight_FirstTime( void);

/**
  * @brief		首次!!!检测外圈有无，外圈没有，进入内圈钩取
  * @param		none
  * @retval		none
  */
void Work_OuterToInner_FirstTime(void);
/**
  * @brief		到中心后!!!外圈检测到，直接钩取
  * @param		电机组数 Class
  * @retval		none
  */
void Work_Straight(uint8_t Class);

/**
  * @brief		到中心后!!!检测外圈有无，外圈没有，进入内圈钩取
  * @param		电机组数 Class
  * @retval		none
  */
void Work_OuterToInner(uint8_t Class);

/**
  * @brief		首次!!!放下砝码动作
  * @param
  * @retval
  */
void Work_PullDown_FirstTime(void);

/**
  * @brief		到中心后!!!放下砝码动作
  * @param
  * @retval
  */
void Work_PullDown(uint8_t Class,uint8_t length);

#endif
