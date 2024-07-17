#ifndef __WORK_H
#define __WORK_H

/**
  * @brief		从内圈移动到外圈进行钩取
  * @param		none
  * @retval	none
  */
void Work_FromInnerToOuter(void);

/**
  * @brief		从外圈移动到内圈进行钩取
  * @param		none
  * @retval	none
  */
void Work_FromOuterToInner(void);

/**
  * @brief		初始化
  * @param
  * @retval
  */
void Work_SteppingMotorInit(void);
#endif
