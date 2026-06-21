#include <NY8.h>
#include "NY8_constant.h"
/*
 * 目标 MCU: NY8BE62DS8
 * 功能描述:
 * - 利用 Timer0 产生 10ms 精确定时中断
 * - 每 10ms 读取一次 PB1 (光耦) 状态
 * - 统计 1 秒(100次)内的高电平次数，解析出 1Hz 方波的 1/4, 2/4, 3/4, 4/4 状态
 * - 安全机制：连续两次(即2秒)检测到相同状态后，才执行继电器切换动作
 * - 低功耗设计：空闲时进入休眠 (sleep)
 */

// ================= 宏定义与参数配置 =================
#define RELAY1_PIN 4 // PA4
#define RELAY2_PIN 2 // PA2
#define OPTO_PIN 1   // PB1

#define RELAY1_ON() PORTA |= (1 << RELAY1_PIN)
#define RELAY1_OFF() PORTA &= ~(1 << RELAY1_PIN)

#define RELAY2_ON() PORTA |= (1 << RELAY2_PIN)
#define RELAY2_OFF() PORTA &= ~(1 << RELAY2_PIN)

#define READ_OPTO() ((PORTB >> OPTO_PIN) & 0x01)

// 定时器相关宏 (当前配置：Timer0时钟源为 I_LRC 32kHz)
// 32kHz 频率下，预分频设为 1:4。每步耗时 = 4 / 32000 秒 = 125us
// 10ms 需要的步数 = 10000us / 125us = 80 步
// 定时器初值 = 256 - 80 = 176
#define T0_INIT_VAL 176

// ================= 全局变量 =================
volatile unsigned char flag_10ms = 0;   // 10ms 到达标志位 (加volatile防止编译器过度优化)
unsigned char cycle_cnt = 0;            // 周期计数器 (0~100)
unsigned char high_cnt = 0;             // 高电平次数统计
unsigned char parsed_state = 0;         // 当前周期解析出的状态
unsigned char last_parsed_state = 0xFF; // 上一周期解析出的状态 (0xFF代表初始未知)
unsigned char active_state = 0xFF;      // 当前实际生效并输出给继电器的状态

// ================= 中断服务函数 =================
void isr(void) __interrupt(0)
{
    // 检查是否是 Timer0 溢出中断
    if (INTFbits.T0IF)
    {
        TMR0 = T0_INIT_VAL; // 重新加载定时器初值
        INTFbits.T0IF = 0;  // 清除中断标志位

        flag_10ms = 1; // 通知主程序：10ms 时间到！
    }
}

// ================= 初始化函数 =================
void system_init()
{
    // 1. IO 口初始化
    IOSTA = 0xEB; // PA4, PA2 输出，其余输入
    IOSTB = 0xFF; // PB 全输入
    PORTA = 0x00; // 继电器默认断开
    PORTB = 0x00;
    BPHCON &= ~(1 << OPTO_PIN); // 开启 PB1 内部上拉电阻

    // 2. Timer0 初始化 (时钟源配置为 I_LRC 32kHz，预分频 1:4)
    // T0MD 寄存器:
    // Bit 3 = 0 (Prescaler 分配给 Timer0)
    // Bit 2:0 = 001 (预分频比 1:4)
    T0MD = 0x01;

    TMR0 = T0_INIT_VAL; // 写入初值

    // 3. 中断开启
    INTF = 0x00;       // 清除所有中断标志
    INTEbits.T0IE = 1; // 允许 Timer0 溢出中断
    ENI();             // 开启全局中断
}

// ================= 主函数 =================
void main(void)
{
    system_init();

    while (1)
    {
        CLRWDT(); // 喂狗

        // 当 10ms 时间到达时，执行一次采样任务
        if (flag_10ms == 1)
        {
            flag_10ms = 0; // 清除标志位

            cycle_cnt++; // 总采样次数 +1

            // 采样光耦状态。假设光耦导通时外部拉低为 1，断开时为 0
            if (READ_OPTO() == 1)
            {
                high_cnt++;
            }

            // 满 100 次采样 (即经过了 1 秒，一个完整的 1Hz 周期)
            if (cycle_cnt >= 100)
            {
                // 步骤 1: 解析本次波形状态 (加入一定的容错区间 ±8 次)
                if (high_cnt <= 8)
                {
                    parsed_state = 0; // 0/4 (常高或常低)
                }
                else if (high_cnt > 17 && high_cnt < 33)
                {
                    parsed_state = 1; // 1/4 状态
                }
                else if (high_cnt > 42 && high_cnt < 58)
                {
                    parsed_state = 2; // 2/4 状态
                }
                else if (high_cnt > 67 && high_cnt < 83)
                {
                    parsed_state = 3; // 3/4 状态
                }
                else if (high_cnt >= 92)
                {
                    parsed_state = 4; // 4/4 状态
                }
                else
                {
                    parsed_state = 0xFF; // 模糊地带，视为无效的干扰状态
                }

                // 步骤 2: 连续两次状态一致，且不同于当前执行状态时，才改变继电器
                if (parsed_state != 0xFF && parsed_state == last_parsed_state)
                {
                    if (parsed_state != active_state)
                    {
                        active_state = parsed_state; // 更新当前生效状态

                        // --- 根据安全确认后的 active_state 执行继电器动作 ---
                        if (active_state == 1)
                        {
                            RELAY1_ON();
                            RELAY2_OFF();
                        }
                        else if (active_state == 2)
                        {
                            RELAY1_OFF();
                            RELAY2_ON();
                        }
                        else if (active_state == 3)
                        {
                            RELAY1_ON();
                            RELAY2_ON();
                        }
                        else
                        {
                            RELAY1_OFF();
                            RELAY2_OFF();
                        }
                    }
                }

                // 步骤 3: 记录本次状态，供下一个周期进行对比
                last_parsed_state = parsed_state;

                // 步骤 4: 一轮统计结束，清零计数器
                cycle_cnt = 0;
                high_cnt = 0;
            }
        }
        else
        {
            // --- 低功耗设计核心 ---
            // 如果 10ms 还没到，让 CPU 进入休眠模式停止执行代码。
            __asm__("sleep");
        }
    }
}