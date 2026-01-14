#include "telemetry.h"
#include <stdint.h>

// 发送1字节，并把它加入两级累加器：
// acc1 累加“和校验”的总和（未截断）；
// acc2 累加“和校验”的前缀和（未截断）。
static inline void SendAndAcc(uint8_t b, uint32_t *acc1, uint32_t *acc2)
{
    Serial_SendByte(b);
    *acc1 += b;       // 和校验累加（不截断）
    *acc2 += *acc1;   // 附加校验：对当前“和校验”的值再累加（不截断）
}

// 发 int16（小端：低字节在前，高字节在后）
static inline void SendI16_LE(int16_t v, uint32_t *acc1, uint32_t *acc2)
{
    uint16_t u = (uint16_t)v;
    SendAndAcc((uint8_t)(u      & 0xFF), acc1, acc2); // low
    SendAndAcc((uint8_t)((u>>8) & 0xFF), acc1, acc2); // high
}

void SendAMessage(float Pitch, float Yaw, float Roll)
{
    int16_t PitchTemp, YawTemp, RollTemp;

    // 全程累加用宽类型，不做按步截断
    uint32_t sum_acc  = 0;   // “和校验”的累加器（宽）
    uint32_t add_acc  = 0;   // “附加校验”的累加器（宽）

    // 帧头 + 类型/长度
    SendAndAcc(0xAA, &sum_acc, &add_acc);
    SendAndAcc(0xFF, &sum_acc, &add_acc);
    SendAndAcc(0x03, &sum_acc, &add_acc);
    SendAndAcc(7,    &sum_acc, &add_acc);

    // 数据区：角度×100 成定点，按“小端”发
    YawTemp   = (int16_t)(Yaw   * 100.0f);
    SendI16_LE(YawTemp,   &sum_acc, &add_acc);

    PitchTemp = (int16_t)(Pitch * 100.0f);
    SendI16_LE(PitchTemp, &sum_acc, &add_acc);

    RollTemp  = (int16_t)(Roll  * 100.0f);
    SendI16_LE(RollTemp,  &sum_acc, &add_acc);

    // ===== 最后一步：只取低 8 位再发送 =====
    uint8_t sum8 = (uint8_t)(sum_acc & 0xFF);  // 和校验低8位
    uint8_t add8 = (uint8_t)(add_acc & 0xFF);  // 附加校验低8位

    Serial_SendByte(sum8);
    Serial_SendByte(add8);
}
