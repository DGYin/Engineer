/* Includes ------------------------------------------------------------------*/
#include "vofa.h"
#if defined(VOFA_USE_USB_CDC)
#include "usbd_cdc_if.h"
#endif

vofa_t vofa;

/* Private function prototypes -----------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined AT THE END of this file
 *   and are strictly related to the hardware platform used.
 *
 */
uint8_t Vofa_PlatformTransmit(void *peripheral_handle, uint8_t *pByte,
                              uint16_t byteSize);

VOFA_RETURN_T Vofa_HandleInit(vofa_t *vofa, vofaParameter_t initParam) {
  uint32_t ret = VOFA_OK;
  memcpy(&vofa->parameter, &initParam, sizeof(initParam));
  vofa->parameter.ctx.peripheral_handle = initParam.peripheral_handle;
  vofa->parameter.ctx.pPlatformTransmit = Vofa_PlatformTransmit;
  return ret;
}

/* User functions -----------------------------------------------------------*/
/**
 * @brief 一个句柄的初始化范例
 *
 * @param vofa
 * @return VOFA_RETURN_T
 */
VOFA_RETURN_T Vofa_InitExample(vofa_t *vofa) {
  uint32_t ret;
  // usb发送无需句柄，所以随便传个空句柄进去就好了
  void *usbCdc;
  vofaParameter_t initParam;
  initParam.peripheral_handle = &usbCdc;
  return ret = Vofa_HandleInit(vofa, initParam);
}

/**
 * @brief 用于将浮点数数据发给Vofa+上位机，每个浮点数会分为一个通道
 *
 * @param vofa
 * @param pFloat
 * @param floatNum
 * @return VOFA_RETURN_T
 */
VOFA_RETURN_T Vofa_JustFloatInSeperatedChannelTransmit(vofa_t *vofa,
                                                       float *pFloat,
                                                       uint8_t floatNum) {
  // 创建发送数组，里面装着使用串口发送的原始数据。
  uint16_t transSize = floatNum * sizeof(float) + 4;
  uint8_t temp[transSize];
  // 数据装填。
  memcpy(&temp, pFloat, floatNum * sizeof(float));
  // 根据Vofa+的上位机协议，要给浮点数后面加上帧尾。
  uint32_t tail = 0x7F800000U; // 小端在前，所以是反过来的。
  uint8_t *add = (uint8_t *)((&temp) + (floatNum * 4));
  memcpy(&temp[floatNum * sizeof(float)], &tail, 4);
  // 调用发送。
  vofa->parameter.ctx.pPlatformTransmit(vofa->parameter.ctx.peripheral_handle,
                                        temp, transSize);
}

/* Platform functions -------------------------------------------------------*/
/**
 * @brief 用于发送的函数，用户可以根据自己的实际平台进行更改
 *
 * @param peripheral_handle
 * @param pByte
 * @param byteSize
 * @return uint8_t
 */
uint8_t Vofa_PlatformTransmit(void *peripheral_handle, uint8_t *pByte,
                              uint16_t byteSize) {
  uint32_t ret;
#if defined(VOFA_USE_USB_CDC)
  return ret = CDC_Transmit_FS(pByte, byteSize);
#endif
}
