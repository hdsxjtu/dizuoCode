#include <ny8.h>
#include "ny8_constant.h"

/*
 * 固件版本: V1.1.0 (Build 20260909)
 * 目标 MCU: NY8BE62DS8
 * 功能描述: 烟感探测器底座信号解析 (工业级抗温漂自适应 + 抗脉冲群架构)
 * - 时钟架构: 32kHz (I_LRC) 2T 模式 (FINST 16kHz)，超低功耗无休眠架构 (整机约9uA)
 * - 抗干扰机制:
 *   * 抗脉冲群 (EFT/B): 连续 2 拍 (20ms) 电平一致去毛刺滤波，彻底隔离瞬态尖峰毛刺
 *   * 抗温漂 (全温区): 信号自身上升沿自适应周期同步，占空比相对比例计算，分子分母温漂误差 100% 抵消
 * - 状态映射:
 *   * 0/4 (0%)          : 故障/断线 -> 故障继电器吸合
 *   * 1/16 (6.25%) 或 1/4 (25%) : 正常待机 -> 双继电器全断开 (3%~35% 完美宽容度)
 *   * 2/4 (50%)         : 故障 A -> 故障继电器吸合 (40%~60%)
 *   * 3/4 (75%)         : 故障 B -> 故障继电器吸合 (65%~85%)
 *   * 4/4 (100%)        : 火警 -> 火警继电器吸合，永久死锁直到断电 (>=88%)
 * - 安全防抖: 连续 2 个完整周期确认相同状态后执行继电器联动
 */

// ================= 固件版本定义 =================
#define FIRMWARE_VER_MAJOR  1
#define FIRMWARE_VER_MINOR  1
#define FIRMWARE_VER_PATCH  0
#define FIRMWARE_VER_STRING "V1.1.0"

// ================= 宏定义与参数配置 =================
#define FAULT_RELAY_PIN 2 // PA2 (6脚) - 对应故障继电器
#define FIRE_RELAY_PIN  4 // PA4 (7脚) - 对应火警继电器
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

// 滤波与边沿检测变量 (抗脉冲群 EFT/B)
unsigned char last_raw_sample = 0; // 上一次原始采样值
unsigned char opto_debounced  = 0; // 去毛刺后的稳定电平 (连续2拍确认)
unsigned char last_debounced  = 0; // 上一次稳定电平 (用于抓上升沿)

// 周期与高电平积分统计 (抗温漂自适应)
unsigned char has_synced = 0; // 相位对齐标志 (0=尚未对齐首个上升沿, 1=已对齐)
unsigned char cycle_cnt  = 0; // 当前周期总采样点数 (标称100，自适应60~135)
unsigned char high_cnt   = 0; // 当前周期高电平采样点数

// 状态判决变量
unsigned char parsed_state      = 0;
unsigned char last_parsed_state = 0xFF;
unsigned char active_state      = 0xFF;

// 火警自锁专属标志位 (0=未触发, 1=已触发且死锁)
unsigned char fire_alarm_latched = 0;

// 固件版本常量 (固化在 ROM 中供固件版本追溯与防混淆)
const char FIRMWARE_VER[] = FIRMWARE_VER_STRING;

// ================= 中断服务函数 =================
void isr(void) __interrupt(0)
{
    if (INTFbits.T1IF)
    {
        INTFbits.T1IF = 0;  // 清除 Timer1 中断标志 (硬件自动重填初值，零丢拍)
        DEBUG_PIN_TOGGLE(); // 示波器测试：每次10ms到达硬件翻转一次 PB2 (高/低电平各10ms，50Hz方波)
        flag_10ms = 1;
    }
}

// ================= 初始化函数 =================
void system_init()
{
    IOSTA = 0xEB; // PA4, PA2 输出，其余输入
#ifdef ENABLE_DEBUG_PIN_TOGGLE
    IOSTB = 0xFB; // PB2 (4脚) 输出，其余输入 (1111 1011b)
#else
    IOSTB = 0xFF; // PB 全输入
#endif
    BPHCON = 0xFF; // 禁用 PB 所有引脚的内部上拉电阻 (PB1 光耦引脚不开启内部上拉)

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

            // 1. 读取光耦引脚电平
            unsigned char raw_sample = READ_OPTO();

            // 2. 【防脉冲群干扰】连续 2 拍（20ms）一致去毛刺滤波
            // 真实信号最短 62.5ms (>=6拍)，脉冲群干扰通常 <15ms
            // 连续2拍电平一致才认可跳变，彻底消除单拍脉冲群毛刺
            if (raw_sample == last_raw_sample)
            {
                opto_debounced = raw_sample;
            }
            last_raw_sample = raw_sample;

            // 3. 检测是否为有效上升沿 (0 -> 1)
            unsigned char is_rising = (opto_debounced == 1 && last_debounced == 0);
            last_debounced          = opto_debounced;

            // 4. 统计累加
            cycle_cnt++;
            if (opto_debounced == 1)
            {
                high_cnt++;
            }

            // 5. 【自适应周期结算与状态判定】
            // 触发结算的两种情况：
            //   情况 A: 周期信号到达完整周期 (上升沿到达，且周期满足门限 cycle_cnt >= 60) -> 免疫时钟高低温温漂！
            //   情况 B: 静态直流信号超时 (常高或常低无跳变，cycle_cnt >= 135，约1.35秒无上升沿) -> 0% 断线故障 或 100% 火警
            unsigned char do_settle  = 0;
            unsigned char prev_cycle = 0;
            unsigned char prev_high  = 0;

            if (is_rising)
            {
                if (has_synced && cycle_cnt >= 60)
                {
                    // 已同步且满足完整周期门限：结算上一完整周期的统计结果
                    prev_cycle = cycle_cnt - 1;
                    prev_high  = high_cnt - 1;
                    do_settle  = 1;
                }

                // 无论是冷启动首次捕获上升沿，还是正常周期结算，都以此上升沿作为新周期的起始第 1 拍
                has_synced = 1;
                cycle_cnt  = 1;
                high_cnt   = 1;
            }
            else if (cycle_cnt >= 135)
            {
                // 超时静态信号结算 (0% 断线 或 100% 火警)
                prev_cycle = cycle_cnt;
                prev_high  = high_cnt;
                do_settle  = 1;

                // 静态信号无边沿跳变，清空同步标志
                has_synced = 0;
                cycle_cnt  = 0;
                high_cnt   = 0;
            }

            if (do_settle)
            {
                // ================== 极速 8 位自适应状态判决 (无乘除法，耗时 <1ms) ==================
                // 利用纯 8 位移位快速得到自适应比例基准 (随温漂 prev_cycle 动态自适应缩放)：
                // - quarter        = 1/4 (25.0%)
                // - one_eighth     = 1/8 (12.5%) 容错裕量
                // - half           = 2/4 (50.0%)
                // - three_quarters = 3/4 (75.0%)
                // 仅需约 13 条机器指令 (0.8ms)，绝不占用 10ms 时间片，从根本上保证每一拍都能 100% 执行完毕！
                unsigned char quarter        = prev_cycle >> 2;
                unsigned char one_eighth     = quarter >> 1;
                unsigned char half           = prev_cycle >> 1;
                unsigned char three_quarters = half + quarter;

                if (prev_high <= 2)
                {
                    // 0/4 (0%): 无探测器或断线故障 (允许 0~2 拍杂波裕量)
                    parsed_state = 0;
                }
                else if (prev_high <= (quarter + one_eighth))
                {
                    // 1: 正常待机 (理论占空比 <= 37.5%)
                    // 完美覆盖 1/16 (6.25%) 与 1/4 (25.0%)
                    parsed_state = 1;
                }
                else if (prev_high <= (half + one_eighth))
                {
                    // 2/4: 故障类型 A (理论占空比 37.5% ~ 62.5%，覆盖 50.0%)
                    parsed_state = 2;
                }
                else if (prev_high <= (three_quarters + one_eighth))
                {
                    // 3/4: 故障类型 B (理论占空比 62.5% ~ 87.5%，覆盖 75.0%)
                    parsed_state = 3;
                }
                else
                {
                    // 4/4: 火警！(理论占空比 > 87.5%，覆盖 100.0%)
                    parsed_state = 4;
                }

                // ================== 核心控制逻辑 ==================

                // 【优先级 1】：火警一旦触发，绝对死锁，不再处理任何状态变化
                if (fire_alarm_latched == 1)
                {
                    // 维持原状，啥也不干 (需要人工断电复位)
                }
                else
                {
                    // 【优先级 2】：任何状态的改变（包括触发故障和恢复正常），都需要连续2个独立周期确认防抖
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
                                // 确认恢复正常！(1/16 或 1/4) 连续两周期信号正常，断开所有继电器
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
            }
        }
    }
}