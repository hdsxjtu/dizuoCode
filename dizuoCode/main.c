#include <ny8.h>
#include "ny8_constant.h"

/*
 * 目标 MCU: NY8BE62DS8
 * 功能描述: 烟感探测器底座信号解析
 * - 0/4 (0%), 2/4 (50%), 3/4 (75%) : 故障状态 -> 故障继电器吸合。
 * - 1/4 (25%) : 正常待机状态 -> 继电器全断开。
 * - 4/4 (100%) : 火警状态 -> 火警继电器吸合，并永久自锁，直到断电。
 * - 安全机制：任何状态的改变（包括触发故障、恢复正常、触发火警），都必须连续2次(即2秒)检测到相同状态才执行。
 */

// #define ENABLE_TEST_MODE

// ================= 宏定义与参数配置 =================
#define FAULT_RELAY_PIN 4 // PA4 (7脚) - 对应故障继电器
#define FIRE_RELAY_PIN  2 // PA2 (6脚) - 对应火警继电器
#define OPTO_PIN        1 // PB1 (5脚) - 对应光耦输入

#define FAULT_RELAY_ON()  PORTA |= (1 << FAULT_RELAY_PIN)
#define FAULT_RELAY_OFF() PORTA &= ~(1 << FAULT_RELAY_PIN)

#define FIRE_RELAY_ON()  PORTA |= (1 << FIRE_RELAY_PIN)
#define FIRE_RELAY_OFF() PORTA &= ~(1 << FIRE_RELAY_PIN)

#define READ_OPTO() ((PORTB >> OPTO_PIN) & 0x01)

#define T0_INIT_VAL 176 // 32kHz下 10ms初值

// ================= 全局变量 =================
volatile unsigned char flag_10ms = 0;
unsigned char cycle_cnt          = 0;
unsigned char high_cnt           = 0;
unsigned char parsed_state       = 0;
unsigned char last_parsed_state  = 0xFF;
unsigned char active_state       = 0xFF;

// 火警自锁专属标志位 (0=未触发, 1=已触发且死锁)
unsigned char fire_alarm_latched = 0;

// ================= 中断服务函数 =================
void isr(void) __interrupt(0)
{
    if (INTFbits.T0IF)
    {
        TMR0          = T0_INIT_VAL;
        INTFbits.T0IF = 0;
        flag_10ms     = 1;
    }
}

// ================= 初始化函数 =================
void system_init()
{
    IOSTA = 0xEB; // PA4, PA2 输出，其余输入
    IOSTB = 0xFF; // PB 全输入
    PORTA = 0x00; // 继电器默认断开
    PORTB = 0x00;
    BPHCON &= ~(1 << OPTO_PIN); // 开启 PB1 内部上拉电阻

    // 切换至低频 32kHz LRC 运行以降低功耗
    OSCCR = 0x02; // SELHOSC = 0 (使用低频), STPHOSC = 1 (停止高频)

    // 配置 Timer0 时钟源为低频 LRC (32kHz)，预分频 1:4
    // LCKTM0 = 1, T0CS = 1, PS0WDT = 0, PS0SEL = 001 -> 0xA1
    T0MD = 0xA1;
    TMR0 = T0_INIT_VAL;

    INTF          = 0x00;
    INTEbits.T0IE = 1;
    ENI();
}

// ================= 主函数 =================
void main(void)
{
    system_init();

    while (1)
    {
        CLRWDT();

        if (flag_10ms == 1)
        {
            flag_10ms = 0;
            cycle_cnt++;

#ifdef ENABLE_TEST_MODE
            // 让 PA2 和 PA4 实时追踪 PB1 的状态
            if (READ_OPTO() == 1)
            {
                FIRE_RELAY_ON();
                FAULT_RELAY_ON();
            }
            else
            {
                FIRE_RELAY_OFF();
                FAULT_RELAY_OFF();
            }
#else
            // 采样光耦状态。
            if (READ_OPTO() == 1)
            {
                high_cnt++;
            }

            // 满 100 次采样 (1 秒结算一次)
            if (cycle_cnt >= 100)
            {
                // 步骤 1: 解析本次波形状态
                if (high_cnt <= 8)
                {
                    parsed_state = 0; // 0/4 (无探测器或断线)
                }
                else if (high_cnt > 17 && high_cnt < 33)
                {
                    parsed_state = 1; // 1/4 (正常)
                }
                else if (high_cnt > 42 && high_cnt < 58)
                {
                    parsed_state = 2; // 2/4 (故障类型 A)
                }
                else if (high_cnt > 67 && high_cnt < 83)
                {
                    parsed_state = 3; // 3/4 (故障类型 B)
                }
                else if (high_cnt >= 92)
                {
                    parsed_state = 4; // 4/4 (火警！)
                }
                else
                {
                    parsed_state = 0xFF; // 干扰模糊地带
                }

                // ================== 核心控制逻辑 ==================

                // 【优先级 1】：火警一旦触发，绝对死锁，不再处理任何状态变化
                if (fire_alarm_latched == 1)
                {
                    // 维持原状，啥也不干 (需要人工断电复位)
                }
                else
                {
                    // 【优先级 2】：任何状态的改变（包括触发故障和恢复正常），都需要连续2秒确认防抖
                    if (parsed_state != 0xFF && parsed_state == last_parsed_state)
                    {
                        if (parsed_state != active_state)
                        {
                            active_state = parsed_state;

                            if (active_state == 4)
                            {
                                // 确认火警！吸合火警继电器并拉起自锁标志
                                FIRE_RELAY_ON();
                                FAULT_RELAY_OFF();
                                fire_alarm_latched = 1;
                            }
                            else if (active_state == 1)
                            {
                                // 确认恢复正常！(1/4) 连续两秒信号正常，断开所有继电器
                                FAULT_RELAY_OFF();
                                FIRE_RELAY_OFF();
                            }
                            else if (active_state == 0 || active_state == 2 || active_state == 3)
                            {
                                // 确认故障！(0/4, 2/4, 3/4)
                                FAULT_RELAY_ON();
                                FIRE_RELAY_OFF();
                            }
                        }
                    }
                }

                // 记录本次状态供下个周期比对 (火警死锁后此记录不再发挥实质作用)
                last_parsed_state = parsed_state;

                // 一轮统计结束，清零计数器
                cycle_cnt = 0;
                high_cnt  = 0;
            }
#endif
        }
        else
        {
            // 进入 Standby 模式 (OPMD[1:0] = 10b, STPHOSC = 1, SELHOSC = 0 -> 0x0A)
            OSCCR = 0x0A;
        }
    }
}