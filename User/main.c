#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "Timer.h"
#include "PID.h"    
#include "BlueSerial.h"
#include "Serial.h"
#include "NRF24L01.h"

#include "Store.h"
#include <math.h>
#include <stdio.h>
// #include "telemetry.h"








PID_t AnglePID = {	
	.Kp = 4.2,
	.Ki = 0,
	.Kd = 10,//7
	.Target = -10,
	.OutMax = 100,
	.OutMin = -100,
	
	.OutOffset = 3,
};

PID_t SpeedPID = {	
	.Kp = 2.5,//4
	.Ki = 0.0125,//0.1
	.Kd = 0,
	
	.OutMax = 20,
	.OutMin = -20,
};


PID_t TurnPID = {	
	.Kp = 2,
	.Ki = 1,
	.Kd = 0,
	
	.OutMax = 10,
	.OutMin = -10,
};

// 速度控制相关变量
float TargetSpeed = 0;              // 遥控器设置的原始目标速度
float CurrentTargetSpeed = 0.0;       // 平滑后的实际目标速度
float TargetTurn = 0.0;               // 遥控器设置的原始转向
float CurrentTargetTurn = 0.0;        // 平滑后的实际转向
float SmoothFactor = 0.15;            // 平滑系数，根据实际调试



uint8_t RunFlag=1;
volatile uint8_t GetRXDataFlag = 0;
volatile uint8_t AngleFlag     = 0;

uint8_t RunSpeedByLoRa=0;
uint8_t RunFlagUpdate;
uint8_t DebugFlag;
int16_t PWML, PWMR;

//按键测试
int16_t AX, AY, AZ, GX, GY, GZ;
int16_t a,b;
uint8_t KeyNum;
float AngleAcc;
float AngleDelta;
float AngleAcc_Cali;
float AngleAcc_Filter;
float Angle;
float AveSpeed, DifSpeed;
float AngleAcc_Offset;
float AvePWM, DifPWM;//平均PWM，差速PWM
float SpeedByLoRa;
float SpeedLeft, SpeedRight;

int16_t AX_Offset, AZ_Offset, GY_Offset;
int16_t GY_Cali;

#define ANGLE_T				10//刷新率
#define SPEED_T				20

#define ANGLE_TARGET		-4.5//目标角度
#define RX_SPEED            100
#define VOLOCITY            1
#define RUNTIME             500
#define ZHIHOUXING          200


int main(void)

{
    Serial_Init();
    MPU6050_Init();
    Motor_Init();
    Encoder_Init();
    Timer_Init();
    Key_Init();
    PID_Init(&AnglePID);
    OLED_Init();

    // Serial_Printf("Hello, World!\r\n");
    uint8_t id = MPU6050_GetID();
    Serial_Printf("WHO_AM_I=0x%02X\r\n", id);
    // OLED_ShowString(1, 0, "Hello World", OLED_8X16);
    
    int32_t a = 0,
            b = 0;

    while (1)
    {
        
        // Serial_Printf(",%.2f", AnglePID.Target);
        
        if (Serial_GetRxFlag()) {               // 有新字节就立刻取
            uint8_t ch = Serial_GetRxData();
            if (ch == 0xAA) {
                GetRXDataFlag = 1;
            }
            else if (ch == 0xFF) {
                GetRXDataFlag = 0;

            } else if (ch == 0xBB) {
                GetRXDataFlag = 2;
            } else if (ch == 0xEE) {
                GetRXDataFlag = 0;
            } else if (ch == 0xCC) {
                AngleFlag = 1;
            } else if (ch == 0x99) {
                AngleFlag = 0;
            } else if (ch == 0xDD) {
                AngleFlag = 2;
            } else if (ch == 0x88) {
                AngleFlag = 0;
            }
        }
        OLED_Printf(0, 12, OLED_6X8, "Angle:%05.3f", Angle);
        OLED_Printf(0, 24, OLED_6X8, "Flag:%d", GetRXDataFlag);
        OLED_Printf(0, 36, OLED_6X8, "OutSpeed:%05.3f", SpeedPID.Out);
        OLED_Printf(0, 48, OLED_6X8, "Speed:%05.3f", AveSpeed);

        OLED_Update();
    }
}







void TIM1_UP_IRQHandler(void)
{
	static uint16_t SensorCount0, SensorCount1, RunCount0, RunCount1;

    static uint16_t RunTime,RunTime2;
    static uint16_t SmoothCount = 0;


    
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		/*定时器定时中断，每隔1ms自动执行一次*/
		
		/*驱动非阻塞式按键*/
		Key_Tick();
		
		/*进入调试模式后，不执行后续PID程序，以免干扰调试*/
		if (DebugFlag) {return;}
		
		SensorCount0 ++;
		if (SensorCount0 >= ANGLE_T)		//每隔ANGLE_T定义的时间执行一次
		{
			SensorCount0 = 0;
			
			/*姿态解析，得到平衡车角度*/
			
			/*获取陀螺仪加速度计原始数据*/
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			/*陀螺仪数据校准*/
			GY_Cali = GY + GY_Offset;
			
			/*计算加速度计角度*/
			AngleAcc = -atan2(AX, AZ) / 3.1415926535 * 180;
			
			/*计算陀螺仪角度增量*/
			AngleDelta = GY_Cali / 32768.0 * 2000.0 * (ANGLE_T / 1000.0);
			
			/*中心角度校准*/
			AngleAcc_Cali = AngleAcc + AngleAcc_Offset;
			
			/*对加速度计角度进行一阶低通滤波，使其更平滑*/
			float Alpha0 = 0.8;
			AngleAcc_Filter = Alpha0 * AngleAcc_Filter + (1 - Alpha0) * AngleAcc_Cali;
			
			/*角度累加陀螺仪角度增量，得到新的角度值*/
			Angle += AngleDelta;
			
			/*新的角度值与加速度计角度进行互补滤波，抑制漂移*/
			float Alpha1 = fabs(DifSpeed) / 5.0 * 0.02 + 0.005;		//根据差速动态调整滤波参数
			if (Alpha1 > 0.02) {Alpha1 = 0.02;}						//参数限幅
			Angle = Alpha1 * AngleAcc_Filter + (1 - Alpha1) * Angle;//互补滤波得到角度值
			
			/*平衡车倒地后，自动停止*/
			if (Angle > 50 || Angle < -50)
			{
				if (RunFlag)
				{
					RunFlag = 0;
					RunFlagUpdate = 1;
				}
			}
		}

        
		
		SensorCount1 ++;
		if (SensorCount1 >= SPEED_T)		//每隔SPEED_T定义的时间执行一次
		{
			SensorCount1 = 0;
			
			/*通过编码器获取电机旋转速度*/
			SpeedLeft = Encoder_Get(1) / 408.0 / (SPEED_T / 1000.0);
			SpeedRight = Encoder_Get(2) / 408.0 / (SPEED_T / 1000.0);
			
			/*获取均速和差速*/
			AveSpeed = (SpeedLeft + SpeedRight) / 2.0;
			DifSpeed = SpeedLeft - SpeedRight;
		}
		
		/*PID控制*/
		if (RunFlag)
		{
			RunCount0 ++;
			if (RunCount0 >= ANGLE_T)		//每隔ANGLE_T定义的时间执行一次
			{
                // SpeedPID.Target = 0;
                RunCount0 = 0;
				
				/*角度环PID计算*/
				AnglePID.Actual = Angle + ANGLE_TARGET;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;		//角度环输出作用于平均PWM
				
				/*平均PWM和差分PWM，合成左轮PWM和右轮PWM*/
				PWML = AvePWM + DifPWM;
				PWMR = AvePWM - DifPWM;
				
				/*PWM限幅*/
				if (PWML > 100) {PWML = 100;} else if (PWML < - 100) {PWML = -100;}
				if (PWMR > 100) {PWMR = 100;} else if (PWMR < - 100) {PWMR = -100;}
				
				/*PWM输出至电机*/
                Motor_SetPWM(1, PWML);
                Motor_SetPWM(2, PWMR);
            }
			
			RunCount1 ++;
			if (RunCount1 >= SPEED_T)		//每隔SPEED_T定义的时间执行一次
			{
				RunCount1 = 0;
				 
                // SpeedPID.Target = 0;
                // SpeedPID.Target = CurrentTargetSpeed;   

				/*速度环PID计算*/
				SpeedPID.Actual = AveSpeed;
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;	//速度环输出作用于角度环输入
				
				/*转向环PID计算*/
				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;		//转向环输出作用于差分PWM
			}
		}
		else
		{
			Motor_SetPWM(1, 0);				//RunFlag为0时，电机停止
			Motor_SetPWM(2, 0);
		}
        if (GetRXDataFlag ==1)
        {

            SpeedPID.Target = 4;
        }        
        else if(GetRXDataFlag == 0)
        {
            SpeedPID.Target = 0;
        } 
		else if (GetRXDataFlag == 2) {
            SpeedPID.Target = -4;
        } 
		
		if (AngleFlag == 1) {
            TurnPID.Target = -4;
        } else if (AngleFlag == 2) {
            TurnPID.Target = 4;
        } else if (AngleFlag == 0) {
            TurnPID.Target = 0;
        }

        // if(GetRXDataFlag == 2)
        // {
        //     RunTime2 ++;
        //     if(RunTime>= RUNTIME)
        //     {
        //         RunTime2 = 0;
        //         GetRXDataFlag = 0;
        //     }
        // }
        
	}
}














// void TIM1_UP_IRQHandler(void)
// {
// 	static uint16_t SensorCount0, SensorCount1,RunCount0,RunCount1,RXCount0;
//     if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
// 	{
// 		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//         Key_Tick();
//         SensorCount0++;
        
//         if(SensorCount0>=ANGLE_T)
//         {
//             SensorCount0=0;
//             // 获取陀螺仪加速度计原始数据
//             MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//             //陀螺仪数据校准
//             GY_Cali = GY + GY_Offset;
//             //计算加速度计角度
//             AngleAcc = -atan2(AX, AZ) / 3.1415926535 * 180;
//             //计算陀螺仪角度增量
//             AngleDelta = GY_Cali / 32768.0 * 2000.0 * (ANGLE_T / 1000.0);
//             //中心角度校准
//             AngleAcc_Cali = AngleAcc + AngleAcc_Offset;//中心角度值校准
//             //对加速度计角度进行一阶低通滤波，使其更平滑
//             float Alpha0 = 0.8;
// 			AngleAcc_Filter = Alpha0 * AngleAcc_Filter + (1 - Alpha0) * AngleAcc_Cali;
//             //角度累加陀螺仪角度增量，得到新的角度值
//             Angle += AngleDelta;
//             // 新的角度值与加速度计角度进行互补滤波，抑制漂移
//             float Alpha1 = fabs(DifSpeed) / 5.0 * 0.02 + 0.005;		//根据差速动态调整滤波参数
// 			if (Alpha1 > 0.02) {Alpha1 = 0.02;}						//参数限幅
// 			Angle = Alpha1 * AngleAcc_Filter + (1 - Alpha1) * Angle;
//             Serial_Printf("Angle: %.2f\r\n", Angle);
//              if (Angle > 50 || Angle < -50)
// 			{
// 				if (RunFlag)
// 				{
//                     RunFlag = 0;
					
// 				}
// 			}

//         }
        
//         SensorCount1 ++;
// 		if (SensorCount1 >= SPEED_T)		//每隔SPEED_T定义的时间执行一次
// 		{
// 			SensorCount1 = 0;
			
// 			/*通过编码器获取电机旋转速度*/
// 			SpeedLeft = Encoder_Get(1) / 408.0 / (SPEED_T / 1000.0);
// 			SpeedRight = Encoder_Get(2) / 408.0 / (SPEED_T / 1000.0);
			
// 			/*获取均速和差速*/
// 			AveSpeed = (SpeedLeft + SpeedRight) / 2.0;
// 			DifSpeed = SpeedLeft - SpeedRight;
//             Serial_Printf("AveSpeed: %.2f", AveSpeed);
// 		}
        
//         if (RunFlag)
//         {
//             RunCount0 ++;
//             RXCount0++;
//             if(RunCount0>=ANGLE_T)
//             {
//                 AnglePID.Kp = 7.2;
//                 AnglePID.Kd = 9.6;
//                 AnglePID.Target = ANGLE_TARGET;
//                 RunCount0=0;
//                 AnglePID.Actual = -Angle;
//                 PID_Update(&AnglePID);
//                 AvePWM = AnglePID.Out;
//                 if(AvePWM >100){AvePWM=100;}
//                 else if(AvePWM <-100){AvePWM=-100;}
//                 Motor_SetPWM(1, AvePWM);
//                 Motor_SetPWM(2, AvePWM);
                
//             }
            
//             // RunCount1 ++;
//             // if(RunCount1>=SPEED_T)
//             // {
                
//             //     SpeedPID.Target = 0;
//             //     SpeedPID.Kp = 2;
//             //     SpeedPID.Ki = 0;
//             //     SpeedPID.Kd = 0;
//             //     RunCount1=0;
//             //     SpeedPID.Actual = AveSpeed;
//             //     PID_Update(&SpeedPID);
//             //     AnglePID.Target = SpeedPID.Out;



//             // }
        
		
// 	    }
//         else
//         {
//             Motor_SetPWM(1, 0);				//RunFlag为0时，电机停止
// 			Motor_SetPWM(2, 0);
//         }
//     }
// }





// static inline void SendAndAcc(uint8_t b, uint8_t *sum, uint8_t *sum2)
// {
//     Serial_SendByte(b);
//     *sum  = (uint8_t)(*sum + b);     // 和校验
//     *sum2 = (uint8_t)(*sum2 + *sum); // 附加校验（对“和校验”的前缀和再求和）
// }

// void SendAMessage(float Pitch, float Yaw, float Roll)
// {
//     int16_t PitchTemp, YawTemp, RollTemp;
//     uint8_t sum  = 0;   // 和校验
//     uint8_t sum2 = 0;   // 附加校验

//     // 帧头 + 类型/长度之类（按你现有的字段顺序算入校验）
//     SendAndAcc(0xAA, &sum, &sum2);
//     SendAndAcc(0xFF, &sum, &sum2);
//     SendAndAcc(0x03, &sum, &sum2);
//     SendAndAcc(7,    &sum, &sum2);

//     // 数据区：*100 放大，发送时低字节在前，高字节在后（小端次序）
//     YawTemp   = (int16_t)(Yaw   * 100.0f);
//     SendAndAcc((uint8_t)(YawTemp      & 0xFF), &sum, &sum2); // low
//     SendAndAcc((uint8_t)((YawTemp>>8) & 0xFF), &sum, &sum2); // high

//     PitchTemp = (int16_t)(Pitch * 100.0f);
//     SendAndAcc((uint8_t)(PitchTemp      & 0xFF), &sum, &sum2);
//     SendAndAcc((uint8_t)((PitchTemp>>8) & 0xFF), &sum, &sum2);

//     RollTemp  = (int16_t)(Roll  * 100.0f);
//     SendAndAcc((uint8_t)(RollTemp      & 0xFF), &sum, &sum2);
//     SendAndAcc((uint8_t)((RollTemp>>8) & 0xFF), &sum, &sum2);

//     // 末尾追加两个校验字节：先“和校验”，再“附加校验”
//     Serial_SendByte(sum);
//     Serial_SendByte(sum2);
// }