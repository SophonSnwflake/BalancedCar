#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  发送姿态消息：帧头(0xAA,0xFF) + 类型(0x03) + 长度(7) + [Yaw,Pitch,Roll]*2字节
 *         末尾追加2字节校验：sum（和校验，mod256）与 sum2（附加校验，对sum的前缀和再求和，mod256）
 * @param  Pitch  俯仰角，单位：度（函数内×100转为int16_t）
 * @param  Yaw    航向角，单位：度（函数内×100转为int16_t）
 * @param  Roll   横滚角，单位：度（函数内×100转为int16_t）
 * @note   依赖外部的 Serial_SendByte(uint8_t)；若名字不同可在本头文件底部修改映射。
 */
void SendAMessage(float Pitch, float Yaw, float Roll);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
