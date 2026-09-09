#include <ny8.h>
#include "ny8_constant.h"

/*
 * 目标 MCU: NY8BE62DS8
 * 功能描述: 烟感探测器底座信号解析
 * - 0/4 (0%), 2/4 (50%), 3/4 (75%) : 故障状态 -> 故障继电器吸合。
 * - 1/16 (6.25%) 或 1/4 (25%) : 正常待机状态 -> 继电器全断开。
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

// 开启此宏：PB2 (4脚) 每10ms翻转一次电平供示波器测量；量产时注释此行即可关闭测试输出以实现极限省电
#define ENABLE_DEBUG_PIN_TOGGLE

#ifdef ENABLE_DEBUG_PIN_TOGGLE
#define DEBUG_PIN          2 // PB2 (4脚) - 示波器测试引脚 (每次10ms翻转一次电平)
#define DEBUG_PIN_TOGGLE() PORTB ^= (1 << DEBUG_PIN)
#else
#define DEBUG_PIN_TOGGLE()
#endif

#define T1_INIT_VAL 159 // 32kHz 2T (FINST 16kHz) 下 10ms 初值: 160 次计数 (0~159), 160 * 0.0625ms = 10.0ms

// ================= 全局变量 =================
volatile unsigned char flag_10ms = 0;
unsigned char cycle_cnt          = 0; // 满 100 次结算，8位即可 (0~255)
unsigned char high_cnt           = 0; // 8位即可，运算更快更轻量
unsigned char parsed_state       = 0;
unsigned char last_parsed_state  = 0xFF;
unsigned char active_state       = 0xFF;

// 火警自锁专属标志位 (0=未触发, 1=已触发且死锁)
unsigned char fire_alarm_latched = 0;

// ================= 中断服务函数 =================
void isr(void) __interrupt(0)
{
    if (INTFbits.T1IF)
    {
        INTFbits.T1IF = 0; // 清除 Timer1 中断标志 (硬件自动重填初值，零丢拍)
        DEBUG_PIN_TOGGLE(); // 示波器测试：每次10ms到达硬件翻转一次 PB2 (高/低电平各10ms，50Hz方波)
        flag_10ms     = 1;
    }
}

// ================= 初始化函数 =================
void system_init()
{
    IOSTA = 0xEB; // PA4, PA2 输出，其余输入
#ifdef ENABLE_DEBUG_PIN_TOGGLE
    IOSTB = 0xFB; // PB2 (4脚) 输出，其余输入 (1111 1011b)
    BPHCON &= ~(1 << OPTO_PIN); // 开启 PB1 内部上拉电阻 (保证光耦电平干净陡峭，与Timer0版本一致)
    BPHCON |= (1 << DEBUG_PIN); // 禁用 PB2 内部上拉电阻
#else
    IOSTB = 0xFF; // PB 全输入
    BPHCON &= ~(1 << OPTO_PIN); // 开启 PB1 内部上拉电阻 (保证光耦电平干净陡峭，与Timer0版本一致)
#endif
    PORTA = 0x00; // 继电器默认断开
    PORTB = 0x00;

    // 切换至低频 32kHz LRC 运行以降低功耗
    OSCCR = 0x02; // SELHOSC = 0 (使用低频), STPHOSC = 1 (停止高频)

    // 配置 Timer1 硬件自动重载模式 (精准 10.0ms)
    // 1. 时钟源选择 FINST (2T模式下为 16kHz)，禁用预分频 (/PS1EN = 1) -> 1:1 无分频，每计数 62.5us
    //    按官方规范：/PS1EN=1 时 PS1SEL[2:0] 必须设为 111b (0x0F) 以防中断误触发
    T1CR2 = 0x0F;

    // 2. 装载 10ms 初值 (160次计数值: 160 * 62.5us = 10.0ms)
    //    手册规定: 先写高 2 位 TMRH[5:4]，再写低 8 位 TMR1
    TMRH &= 0xCF;       // 清空高 2 位 TMRH[5:4] (初值 159 小于 256)
    TMR1 = T1_INIT_VAL; // 159 倒数至 0 共经历 160 个周期 (10.0ms)

    // 3. 启动 Timer1，开启硬件自动重载 (T1RL=1, T1EN=1, PWM1OEN=0)
    T1CR1 = 0x03;

    INTF          = 0x00;
    INTEbits.T1IE = 1;
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
            // 采样光耦状态
            if (READ_OPTO() == 1)
            {
                high_cnt++;
            }

            // 满 100 次采样 (10ms * 100 = 1 秒结算一次)
            if (cycle_cnt >= 100)
            {
                // 步骤 1: 解析本次波形状态 (10ms 采样，总数 100 次)
                if (high_cnt <= 2)
                {
                    parsed_state = 0; // 0/4 (无探测器或断线，留 0~2 次抗杂波裕量)
                }
                else if (high_cnt >= 4 && high_cnt <= 35)
                {
                    // 1: 正常待机
                    // 完美兼容 1/16 占空比 (理论 6.25 次，涵盖 4~15 次)
                    // 同时完美兼容 1/4 占空比 (理论 25 次，涵盖 16~35 次)
                    parsed_state = 1;
                }
                else if (high_cnt >= 42 && high_cnt <= 58)
                {
                    parsed_state = 2; // 2/4 (故障类型 A，50% 理论50次)
                }
                else if (high_cnt >= 67 && high_cnt <= 83)
                {
                    parsed_state = 3; // 3/4 (故障类型 B，75% 理论75次)
                }
                else if (high_cnt >= 90)
                {
                    parsed_state = 4; // 4/4 (火警！100% 理论100次)
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
                                // 确认恢复正常！(1/16 或 1/4) 连续两秒信号正常，断开所有继电器
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

                // 记录本次状态供下个周期比对
                last_parsed_state = parsed_state;

                // 一轮统计结束，清零计数器
                cycle_cnt = 0;
                high_cnt  = 0;
            }
#endif
        }
    }
}