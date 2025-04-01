/******************************************************************************
;  *       @型号                 : MC32F7361
;  *       @创建日期             : 2021.12.21
;  *       @公司/作者            : SINOMCU-FAE
;  *       @晟矽微技术支持       : 2048615934
;  *       @晟矽微官网           : http://www.sinomcu.com/
;  *       @版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------
;  *
******************************************************************************/

#include "user.h"

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++)
    {
        INDF0 = 0x00;
    }
    FSR0 = 0xFF;
    INDF0 = 0x00;
}
/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : IO初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io口数据位
    OEP0 = 0x3F;   // io口方向 1:out  0:in
    PUP0 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP0 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P0ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP1 = 0x00;   // io口数据位
    OEP1 = 0xFF;   // io口方向 1:out  0:in
    PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP2 = 0x00; // io口数据位
    OEP2 = 0x0F; // io口方向 1:out  0:in
    PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
    PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

    PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
    DRVCR = 0x80; // 普通驱动
}

/************************************************
;  *    @函数名            : ADC_Init
;  *    @说明              : ADC初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void adc_config(void)
{
#if USE_MY_DEBUG
    // 检测充电座放电电流的引脚：
    P11OE = 0; // 输入模式
    P11DC = 1; // 模拟模式

    // // 测试时，使用 P05 AN4 代替 1/4VDD 通道，检测的ad值不变
    // P05OE = 0;
    // P05DC = 1; // 模拟模式
#else
    // 检测充电座放电电流的引脚：
    P15OE = 0; // 输入模式
    P15DC = 1; // 模拟模式
#endif

    ADCR0 = 0x0B; // 12位精度，使能adc
    ADCR1 = 0x80; // adc转换时钟选择 FHIRC/32，使用內部2.0V参考电压
    ADCR2 = 0x0F; // 采样时间，只能固定是15 个 ADCLK
}

// 控制充电的pwm

void timer0_pwm_config(void)
{
    // T0CR = 0x49; // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
// T0CNT = 50 - 1;
#if 0
    // 250KHz：
    T0CR = 0x49; // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    // 理论上重装载值应该是128，但是时钟是RC振荡得来的，不准确，需要加上补偿
    T0LOAD = (128 + 10) - 1; //
    // T0DATA = 23 + 10;
    T0DATA = 59 + 10;
    // T0DATA = 45 + 10;
#endif

#if 1
    T0CR = 0x49;  // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    T0LOAD = 118; // 271KHz
    // T0LOAD = 135; // 235.2KHz
    // T0LOAD = 143; // 222，有时是224KHz
    // T0LOAD = 144;  // 220-222KHz
    // T0LOAD = 146;  // 218KHz
    // T0LOAD = 154; // 206.8KHz
    // T0LOAD = 158; // 201.6KHz
    // T0LOAD = 94; // 在358 - 363 KHz
    T0DATA = 41;

    // T0CR = (0x01 << 6) | (0x01 << 3); // 使能PWM,FTMR,不分频 (约每0.000000015625us计数一次，实际的肯定会与计算的有误差，需要加上补偿)
    // // T0LOAD = 188; // 363 ~ 369 KHz
    // T0LOAD = 190; // 平均值约为360.7KHz
    // T0DATA = 28;
#endif

#if 0
    // 214-216KHz ， 10%
    T0CR = 0x49;  // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    T0LOAD = 148; //
    T0DATA = 15;
#endif

#if 0
    // 186KHz
    T0CR = 0x49; // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    T0LOAD = 184; //
    T0DATA = 74;
#endif

#if 0
    // 135KHz
    T0CR = 0x49; // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    T0LOAD = 255 - 1; //
    T0DATA = 127;
#endif

#if 0
    // 125KHz
    T0CR = 0x4A;  // 使能PWM,FTMR,4分频 (约每0.06250us计数一次)
    T0LOAD = 137; //
    T0DATA = 14;
#endif

    PWMCR0 = 0x00; // 正向输出
    PWMCR1 = 0x18; // 时钟源FHOSC × 2  普通模式
    // PWM0OPS = 0; // 选择P16端口输出 (可以不写，默认就是0)

    // T0EN = 1;
    T0EN = 0;   // 关闭定时器
    PWM0EC = 0; // 不使能PWM0输出
}

void set_timer0_pwm_when_detecting(void)
{
    T0EN = 0;     // 关闭定时器
    PWM0EC = 0;   // 不使能PWM0输出
    T0CR = 0x49;  // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    T0LOAD = 114; // 约282KHz
    // T0DATA = 17;
    T0DATA = 35;
    T0EN = 1;   // 使能定时器
    PWM0EC = 1; // 使能PWM0输出
    delay_ms(1);
}

void set_timer0_pwm_when_charging(void)
{
    T0EN = 0;   // 关闭定时器
    PWM0EC = 0; // 不使能PWM0输出

    // T0CR = (0x01 << 6) | (0x01 << 3); // 使能PWM,FTMR,不分频 (约每0.000000015625us计数一次，实际的肯定会与计算的有误差，需要加上补偿)
    // // T0LOAD = 190;                     // 平均值约为360.7KHz(实际只有333K~334K)
    // T0LOAD = 177; //
    // T0DATA = 41;

    T0CR = 0x49; // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    // T0LOAD = 95; // 实际测试是 333K
    // T0DATA = 19;
    // T0LOAD = 96; //
    // T0DATA = 19;
    T0LOAD = 97; // 实际测试是 326K
    T0DATA = 19;

    // T0LOAD = 103; // 约310KHz
    // T0DATA = 26;
    // T0LOAD = 111; // 约288KHz
    // T0DATA = 26;

    T0EN = 1;   // 使能定时器
    PWM0EC = 1; // 使能PWM0输出
    delay_ms(1);
}

// 定时器3
void timer3_config(void)
{
    T3LOAD = 135 - 1; // FCPU 64分频后，这里是1ms触发一次中断（用计算出来的值会有误差，这里加上了一些补偿）
    T3CR = 0x86;      // 使能定时器，时钟源选择FCPU，64分频
    T3IE = 1;
}

void adc_sel_channel(u8 adc_channel)
{
    static u8 last_adc_channel = ADC_CHANNEL_NONE;

    if (last_adc_channel == adc_channel)
    {
        return;
    }

    last_adc_channel = adc_channel;

    switch (adc_channel)
    {
    case ADC_CHANNEL_BAT:
#if USE_MY_DEBUG
        // // 测试时，用 P05 AN4 代替 1/4VDD 通道
        // ADCHS3 = 0;
        // ADCHS2 = 1;
        // ADCHS1 = 0;
        // ADCHS0 = 0;
        // 选择 1/4 VDD 通道
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
#else
        // 选择 1/4 VDD 通道
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
#endif
        ADCR1 = 0x80; // adc转换时钟选择 FHIRC/32，使用內部2.0V参考电压
        break;

    case ADC_CHANNEL_LOAD:
#if USE_MY_DEBUG
        // P11 AN6
        ADCHS3 = 0;
        ADCHS2 = 1;
        ADCHS1 = 1;
        ADCHS0 = 0;
#else
        // P15 AN9，检测充电座 放电
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 0;
        ADCHS0 = 1;
#endif
        ADCR1 = 0x83; // adc转换时钟选择 FHIRC/32，使用VDD作为参考电压
        break;

    default:
        break;
    }

    delay_ms(1); // 等待adc稳定
}

// 获取adc单次转换后的值
u16 adc_get_val(void)
{
    u8 i = 0; // adc采集次数的计数
    u16 g_temp_value = 0;
    u32 g_tmpbuff = 0;
    u16 g_adcmax = 0;
    u16 g_adcmin = 0xFFFF;

    // 采集20次，去掉前两次采样，再去掉一个最大值和一个最小值，再取平均值
    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
        while (!ADEOC)
            ;                // 等待转换完成
        g_temp_value = ADRH; // 取出转换后的值
        g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
        if (i < 2)
            continue; // 丢弃前两次采样的
        if (g_temp_value > g_adcmax)
            g_adcmax = g_temp_value; // 最大
        if (g_temp_value < g_adcmin)
            g_adcmin = g_temp_value; // 最小
        g_tmpbuff += g_temp_value;
    }
    g_tmpbuff -= g_adcmax;           // 去掉一个最大
    g_tmpbuff -= g_adcmin;           // 去掉一个最小
    g_temp_value = (g_tmpbuff >> 4); // 除以16，取平均值

    return g_temp_value;
}

// 获取adc单次转换后最大的值
// u16 adc_get_max_val(void)
// {
//     u8 i = 0; // adc采集次数的计数
//     volatile u16 g_temp_value = 0;
//     volatile u16 g_adcmax = 0;

//     // 采集40次
//     for (i = 0; i < 40; i++)
//     {
//         ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
//         while (!ADEOC)
//             ;                // 等待转换完成
//         g_temp_value = ADRH; // 取出转换后的值
//         g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);

//         if (g_temp_value > g_adcmax)
//             g_adcmax = g_temp_value; // 最大
//     }

//     return g_adcmax;
// }

// // 获取adc单次转换后的值
// u16 adc_get_val_once(void)
// {
//     u16 g_temp_value = 0;
//     ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
//     while (!ADEOC)
//         ;                // 等待转换完成
//     g_temp_value = ADRH; // 取出转换后的值
//     g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
//     return g_temp_value;
// }

/************************************************
;  *    @函数名            : Sys_Init
;  *    @说明              : 系统初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    timer0_pwm_config();
    timer3_config();
    adc_config();

#if USE_MY_DEBUG
    // 检测 充电的引脚：
    // P03PD = 1; // 使能下拉电阻 (有外部下拉，并且不能打开下拉电阻，会检测不到)
    P03OE = 0; // 输入模式
#else
    // 检测 充电的引脚：
    // P04PD = 1; // 使能下拉电阻 (有外部下拉，并且不能打开下拉电阻，会检测不到)
    P04OE = 0; // 输入模式
#endif

    // 检测霍尔元器件的引脚:
    P13PU = 0; // 上拉  --  不能删掉这一句，可能会导致进度睡眠又唤醒
    P13OE = 0; // 输入模式
    // P13KE = 1; // 使能键盘中断

    GIE = 1;
}

#if 0  // 充电检测
/**
 * @brief 充电检测
 *
 * @return * u8
 *              0 -- 未在充电
 *              1 -- 正在充电
 *              2 -- 还在检测是否有充电（还未添加该项）
 */
u8 is_in_charging(void)
{
    static u8 last_status = 0;
    u8 ret = 0;
    u8 i = 0;
    u8 cnt = 0;

    if (CHARGE_PIN)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }

    if (ret == last_status)
    {
        return ret;
    }

    if (CHARGE_PIN)
    {
        cnt = 0;
        for (i = 0; i < 200; i++)
        {
            if (CHARGE_PIN)
            {
                cnt++;
            }

            delay_ms(1);
        }

        if (cnt >= 180)
        {
            // 如果检测到确实在充电
            ret = 1;
        }
    }
    else
    {
        cnt = 0;
        for (i = 0; i < 200; i++)
        {
            if (0 == CHARGE_PIN)
            {
                cnt++;
            }

            delay_ms(1);
        }

        if (cnt >= 180)
        {
            // 如果检测到不在充电
            ret = 0;
        }
    }

    last_status = ret;

    return ret;
}
#endif // 充电检测

#if 0  // 是否打开保护盖
u8 is_open_lid(void)
{
    static u8 last_status = 0;
    u8 ret = 0;
    u8 i = 0;
    u8 cnt = 0;
    if (P13D)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }

    // 如果没有发生变化
    if (ret == last_status)
    {
        return ret;
    }

    if (P13D) // 如果收纳盒是开启的
    {
        // cnt = 0;
        for (i = 0; i < 200; i++)
        {
            if (P13D)
            {
                cnt++;
            }

            delay_ms(1);
        }

        if (cnt >= 200)
        {
            ret = 1;
        }
    }
    else // 如果收纳盒是关闭的
    {
        // cnt = 0;
        for (i = 0; i < 200; i++)
        {
            if (0 == P13D)
            {
                cnt++;
            }

            delay_ms(1);
        }

        if (cnt >= 200)
        {
            ret = 0;
        }
    }

    last_status = ret;

    return ret;
}
#endif // 是否打开保护盖

// 是否检测到负载
// 返回值 0--没有负载  1--有负载 2--还在检测中
u8 is_detect_load(void)
{
    u8 ret = 2; // 表示还在检测中
    volatile u32 tmp_u32 = bat_adc_val;
    tmp_u32 = tmp_u32 * 2000 * 4 / 4096;
    volatile u32 cmp_val = (tmp_u32 - CHARGING_CURRENT_ADC_VAL) * 4096 / tmp_u32;

    if (0 == flag_is_enable_detect_load)
    {
        // 如果是刚进入负载检测
        // 初始化变量和标志位
        detect_load_cnt = 0;
        undetect_load_cnt = 0;
        flag_tim_set_scan_time_is_arrive = 0;
        flag_is_enable_detect_load = 1; // 使能该标志位，利用定时器开始计时
    }

    adc_sel_channel(ADC_CHANNEL_LOAD);
    adc_val = adc_get_val();

    if (
#if 0
        (flag_is_being_charged && 114 == T0LOAD && adc_val < DETECT_LOAD_ADC_VAL_WHEN_BE_CHARGING) || /* 充电座正在被充电，且使用282KHz的PWM检测负载 */
        // (flag_is_being_charged && 114 != T0LOAD && adc_val < 3725) ||                                 /* 充电座正在被充电，且使用326KHz的PWM检测负载是否断开 */
        (flag_is_being_charged && 114 != T0LOAD && adc_val < cmp_val) ||                                 /* 充电座正在被充电，且使用326KHz的PWM检测负载是否断开 */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val < 3882) /* 充电座未被充电，且使用 326KHz 的PWM检测负载是否断开 */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val < 3700) /* 充电座未被充电，且使用 326KHz 的PWM检测负载是否断开 */
        (0 == flag_is_being_charged && 114 == T0LOAD && adc_val < 3649) || /* 充电座未被充电，且使用 282KHz 的PWM检测负载是否断开 */
        (0 == flag_is_being_charged && 114 != T0LOAD && adc_val < cmp_val) /* 充电座未被充电，且使用 326KHz 的PWM检测负载是否断开 */
#endif

        adc_val < cmp_val)
    {
        detect_load_cnt++;
    }
    else if (

#if 0
        (flag_is_being_charged && 114 == T0LOAD && adc_val > UNDETECT_LOAD_ADC_VAL_WHEN_BE_CHARGING) || /* 充电座正在被充电，且使用282KHz的PWM检测负载 */
        // (flag_is_being_charged && 114 != T0LOAD && adc_val > 3910) ||                                   /* 充电座正在被充电，且使用326KHz的PWM检测负载是否断开 */
        (flag_is_being_charged && 114 != T0LOAD && adc_val > cmp_val) ||                                   /* 充电座正在被充电，且使用326KHz的PWM检测负载是否断开 */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val > 3983) /* 充电座未被充电，且使用 326KHz 的PWM检测负载是否断开 */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val > 3780) /* 充电座未被充电，且使用 326KHz 的PWM检测负载是否断开 */
        (0 == flag_is_being_charged && 114 == T0LOAD && adc_val > 3872) || /* 充电座未被充电，且使用 282KHz 的PWM检测负载是否断开 */
        (0 == flag_is_being_charged && 114 != T0LOAD && adc_val > cmp_val) /* 充电座未被充电，且使用 326KHz 的PWM检测负载是否断开 */
#endif

        adc_val > cmp_val)
    {
        undetect_load_cnt++;
    }

    if (flag_tim_set_scan_time_is_arrive)
    {
        flag_tim_set_scan_time_is_arrive = 0;
        flag_is_enable_detect_load = 0; // 清零该标志位，关闭对应的定时器计时功能
        delay_ms(1);

        if (detect_load_cnt > undetect_load_cnt)
        {
            // 如果进入到这里，说明可能有负载
            if ((detect_load_cnt - undetect_load_cnt) > (undetect_load_cnt / 2))
            {
                // 如果检测到负载的计数 - 未检测到负载的计数 大于 未检测到负载计数的一半
                ret = 1;
            }
        }
        else
        {
            // 如果进入到这里，说明可能断开/没有负载
            // if ((undetect_load_cnt - detect_load_cnt) > (detect_load_cnt / 2))
            // {
            //     // 如果未检测到负载的计数 - 检测到负载的计数 大于 检测到负载计数的一半
            //     ret = 0;
            // }

            // 直接认为没有负载：
            ret = 0;
        }

        detect_load_cnt = 0;
        undetect_load_cnt = 0;

        // last_status = 0;
        // pulse_cnt = 0;
    }

    return ret;
}

// 关闭给主机的充电：
void stop_charging_the_host(void)
{
    LED_GREEN_OFF();
    PWM_DISABLE(); // 关闭控制充电的pwm
}

void low_power_scan_handle(void)
{
    if (CUR_STATUS_NONE == cur_dev_status ||    /* 无任何状态 */
        CUR_STATUS_POWER_OFF == cur_dev_status) /* 低电量关机 */
    {
    low_power_label:
        // 进入低功耗
        GIE = 0; // 禁用所有中断
        LED_GREEN_OFF();
        LED_RED_OFF();
        PWM_DISABLE();
        // LED引脚配置为输入:
        // P00OE = 0;
        // P17OE = 0;
        // PWM引脚输出低电平:
        P16D = 0;
        // 关闭定时器 3
        T3EN = 0;
        T3IE = 0;
// 关闭adc，检测ad的引脚配置为输入:
#if USE_MY_DEBUG
        // 检测充电座放电电流的引脚：
        P11OE = 0; // 输入模式
        P11DC = 0; // 数字模式
#else
        // 检测充电座放电电流的引脚：
        P15OE = 0; // 输入模式
        P15DC = 0; // 数字模式
#endif

        // adc通道选择一个不是 1/4 VDD的
        ADCHS3 = 0;
        ADCHS2 = 0;
        ADCHS1 = 0;
        ADCHS0 = 0;
        ADEN = 0;
        // 检测充电的引脚和检测霍尔的引脚配置为键盘中断触发
        P13KE = 1;
// 充电检测引脚配置为键盘中断触发
#if USE_MY_DEBUG
        P03KE = 1;
#else
        P04KE = 1;
#endif
        KBIF = 0;
        KBIE = 1;
        LVDEN = 0; // 是否关闭LVD都不影响待机电流
        HFEN = 0;  // 关闭高速时钟
        LFEN = 0;  // 关闭低速时钟
        // 休眠前注意关闭不用的外设
        Nop();
        Nop();
        Stop();
        Nop();
        Nop();
        HFEN = 1; // 开启高速时钟
        LVDEN = 1;
#if USE_MY_DEBUG
        P03KE = 0;
#else
        P04KE = 0;
#endif
        P13KE = 0; // 关闭键盘中断
        KBIE = 0;
        KBIF = 0;

        // 判断是否是霍尔或者充电唤醒，如果不是，回到低功耗
        if (CHARGE_PIN || 0 == P13D)
        {
            // 如果充电检测引脚是高电平 或者 霍尔检测脚是低电平(关闭了收纳盒)
            // 退出低功耗
        }
        else
        {
            // 回到低功耗
            goto low_power_label;
        }

        // DEBUG_PIN = ~DEBUG_PIN; // 测试是否能唤醒单片机
        GIE = 1;
        Sys_Init();
        delay_ms(1);

        if (CHARGE_PIN)
        {
            flag_is_being_charged = 1;
            cur_dev_status = CUR_STATUS_BE_CHARGING; // 直接进入被充电的状态，不检测负载
        }

        if (0 == P13D)
        {
            flag_is_open_lid = 0; // 表示盖上了保护盖
        }
        else
        {
            flag_is_open_lid = 1; // 表示打开了保护盖
        }
    }
}

/************************************************
;  *    @函数名            : main
;  *    @说明              : 主程序
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
// volatile u32 timer3_cnt; // 测试时使用(测试充电座能否检测到负载)
void main(void)
{
    Sys_Init();
    delay_ms(10);

    LED_GREEN_ON();
    delay_ms(1000);
    LED_GREEN_OFF();

#if 0  // 测试时使用
    // flag_is_low_bat = 1; // 测试时使用
    // LED_GREEN_ON();
    PWM_ENABLE();
    T3EN = 0;

    // delay_ms(4000); // 等待检测负载的电流稳定
    // T3EN = 1;       // 测试时使用
#endif // 测试时使用

    // set_timer0_pwm_when_charging(); // // 测试时使用

    flag_is_open_lid = 1;                  // 表示保护盖打开
    flag_is_being_charged = 0;             // 表示没有在充电
    cur_dev_status = CUR_STATUS_POWER_OFF; // 上电之后便关机

    while (1)
    {
#if 1

        if (CUR_STATUS_NONE == cur_dev_status)
        {
            if (0 == flag_is_open_lid) // 如果没有打开盖子
            {
                // 如果设备可以开机，检测有没有负载，如果没有 负载 -> 关机
                // 如果给负载充满电 -> 关机
                set_timer0_pwm_when_detecting();
                // 打开充电
                PWM_ENABLE();
                LED_GREEN_ON(); //
                // delay_ms(4000); // 等待电平稳定 -- 这期间可以检测有没有打开保护盖
                {
                    u16 i = 0;
                    // for (i = 0; i < 4000; i++)
                    for (i = 0; i < 500; i++) // 等待500ms
                    {
                        // if (is_open_lid()) // 如果打开了保护盖
                        if (flag_is_open_lid) // 如果打开了保护盖
                        {
                            stop_charging_the_host(); // 关闭pwm，关闭绿灯
                            break;
                        }

                        delay_ms(1);
                    }
                }

                adc_sel_channel(ADC_CHANNEL_LOAD);
                flag_is_enable_detect_load = 0; // 清除该标志位，准备进入检测负载

                while (1) // 连续检测 xx s，判断有没有负载
                {

#if 0 // 在该代码块内使用独立的检测负载的方式，未使用定时器来辅助计时
                        static volatile u16 __detect_ms_cnt = 0;
                        adc_val = adc_get_val();

                        if (adc_val < 3649)
                        {
                            detect_load_cnt++;
                        }
                        else if (adc_val > 3872)
                        {
                            undetect_load_cnt++;
                        }

                        delay_ms(1);
                        __detect_ms_cnt++;

                        if (__detect_ms_cnt >= 2500)
                        {
                            __detect_ms_cnt = 0;

                            if (detect_load_cnt > undetect_load_cnt)
                            {
                                // 如果进入到这里，说明可能有负载
                                if ((detect_load_cnt - undetect_load_cnt) > (undetect_load_cnt / 2))
                                {
                                    // 如果检测到负载的计数 - 未检测到负载的计数 大于 未检测到负载计数的一半
                                    // 如果检测到有负载:
                                    // detect_load_cnt = 0;
                                    // undetect_load_cnt = 0;
                                    cur_dev_status = CUR_STATUS_IN_CHARGING;
                                    set_timer0_pwm_when_charging();
                                    // break;
                                }
                            }
                            else
                            {
                                // 如果进入到这里，说明可能断开/没有负载
                                // if ((undetect_load_cnt - detect_load_cnt) > (detect_load_cnt / 2))
                                {
                                    // 如果未检测到负载的计数 - 检测到负载的计数 大于 检测到负载计数的一半
                                    // 如果检测到 没有负载
                                    // detect_load_cnt = 0;
                                    // undetect_load_cnt = 0;
                                    stop_charging_the_host(); // 关闭pwm，关闭绿灯
                                    // break;
                                }
                            }

                            if (flag_is_being_charged) // 如果检测到了充电器
                            {
                                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                            }

                            detect_load_cnt = 0;
                            undetect_load_cnt = 0;
                            break;
                        }

#endif // 在该代码块内使用独立的检测负载的方式，未使用定时器来辅助计时

                    ret_u8 = is_detect_load();
                    if (0 == ret_u8) // 如果未检测到负载
                    {
                        stop_charging_the_host(); // 关闭pwm，关闭绿灯
                        break;                    // 退出检测负载的循环
                    }
                    else if (1 == ret_u8) // 如果检测到负载
                    {
                        cur_dev_status = CUR_STATUS_IN_CHARGING;
                        set_timer0_pwm_when_charging();
                        break; // 退出检测负载的循环
                    }

                    if (flag_is_open_lid)
                    {
                        // 如果打开了收纳盒，退出
                        detect_load_cnt = 0;
                        undetect_load_cnt = 0;
                        // __detect_ms_cnt = 0;
                        // flag_is_enable_detect_load = 0; // 清零该标志位，关闭对应的定时器计时功能
                        // delay_ms(1);
                        stop_charging_the_host(); // 关闭pwm，关闭绿灯
                        break;                    // 退出检测负载的循环
                    }

                    // 还不能直接进入被充电的状态，会无法关闭绿灯，该状态里面没有相应的处理
                    // else if (flag_is_being_charged) // 如果在检测是否有负载期间 发现有充电器插入
                    // {
                    //     cur_dev_status = CUR_STATUS_BE_CHARGING; //
                    //     break;                                   // 退出检测负载的循环
                    // }
                } // while (1) // 连续检测 xx s，判断有没有负载
            }

            if (flag_is_being_charged) // 如果检测到了充电器
            {
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                if (1 == LED_GREEN_PIN)                  // 如果绿灯还在点亮，说明还有负载
                {
                    flag_is_detect_load_when_charged = 1; // 表示在充电时检测到了负载
                }
            }

            // loop_cnt++;
            // if ((cur_dev_status != CUR_STATUS_NONE) || /* 如果状态有变化，跳出 (CUR_STATUS_NONE == cur_dev_status) 的代码块 */
            //     loop_cnt >= 2000)                      /* 如果超过了 xx s，也跳出当前的代码块 */
            // {
            //     loop_cnt = 0;
            //     break;
            // }

            // delay_ms(1);
            // } // while (1)

            //             ret_u8 = is_in_charging(); // 判断是否有外部的5V输入
            //             if (ret_u8)
            //             {
            // #if 0  // 被充电时不给主机充电的版本
            //        // 如果充电座正在被充电，关闭给主机的充电:
            //                 stop_charging_the_host();
            // #endif // 被充电时不给主机充电的版本

            //                 cur_dev_status = CUR_STATUS_BE_CHARGING; //
            //                 delay_ms(1);
            //                 continue; //
            //             }

            // 如果执行到这里还是没有改变状态，进入低功耗
            // 函数内部会判断是不是处于NONE状态，才进入低功耗：
            low_power_scan_handle();
        } // if (CUR_STATUS_NONE == cur_dev_status)
        else if (CUR_STATUS_IN_CHARGING == cur_dev_status)
        {
            // 如果正在给主机充电，检测：
            // 主机是否充满电
            // 是否插入了充电器
            // 是否打开了盖子
            // 检测是否低电量

            ret_u8 = is_detect_load();
            if (1 == ret_u8)
            {
                // 如果检测到有负载
                // 什么都不用处理
            }
            else if (0 == ret_u8)
            {
                // 如果确实没有检测到负载 说明已经给主机充满电
                flag_is_fully_charged = 1;

                /* 如果没有打开盖子/插入充电器，绿灯会一直常亮 */

                // 现在改成充满电，熄灭绿灯
                stop_charging_the_host();              // 关闭PWM、关闭绿灯
                flag_is_low_bat = 0;                   // 不给主机充电时，清空该标志位
                cur_dev_status = CUR_STATUS_POWER_OFF; // 直接关机
                continue;
            }
            else if (2 == ret_u8)
            {
                // 如果还在检测有没有负载
            }

            // 检测是否打开了收纳盒
            if (flag_is_open_lid)
            {
                // 如果打开了收纳盒，断开充电：
                cur_dev_status = CUR_STATUS_NONE;

                flag_is_fully_charged = 0;
                flag_is_enable_detect_load = 0; // 清零该标志位，关闭对应的定时器计时功能
                delay_ms(1);
                stop_charging_the_host(); // 关闭PWM、关闭绿灯
                continue;                 // 跳过当前循环
            }

            // 低电量和低电量关机检测：
            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            bat_adc_val = adc_val;
            // if (adc_val < 1485 - AD_OFFSET) // 2.9V
            // if (adc_val < 1587 - AD_OFFSET) // 3.1V
            // if (adc_val < 1587) // 3.1V
            if (adc_val < 1536) // 3.0V
            {
                // 如果检测到电池电电量小于 xx V
                // 断开充电：
                cur_dev_status = CUR_STATUS_POWER_OFF;
                flag_is_fully_charged = 0;
                flag_is_enable_detect_load = 0; // 清零该标志位，关闭对应的定时器计时功能
                delay_ms(1);
                stop_charging_the_host();
                continue;
            }
            // else if (adc_val < 1638 - AD_OFFSET) // 如果充电座的电池电量小于3.2V
            // else if (adc_val < 1638) // 如果充电座的电池电量小于3.2V
            else if (adc_val < 1689) // 如果充电座的电池电量小于3.3V
            {
                //
                LED_GREEN_OFF(); // 关掉绿灯，让红灯慢闪
                delay_ms(1);     // 等待定时器操作红灯
                flag_is_low_bat = 1;
                // flag_is_fully_charged = 0;
            }
            // else if (adc_val > 1638 + AD_OFFSET) // 注意这一条件要与 (充电座的电池电量小于3.2V) 的条件相隔一个ad值死区
            else if (adc_val > 1689 + AD_OFFSET) // 注意这一条件要与 (充电座的电池电量小于3.3V) 的条件相隔一个ad值死区
            {
                flag_is_low_bat = 0;
                delay_ms(1); // 等待定时器关闭红灯
                // 如果不小于3.2V，重新点亮绿灯
                // (防止电池电压从3.2V调到3.2以下，又升回3.2V，指示灯会全部熄灭，不会点亮绿色或红色的指示灯的情况)
                // LED_GREEN_ON(); // 不能在这里点亮，如果检测到有负载，绿灯的闪烁会不正常
            }

            // 检测是否插入了充电器:
            // if (is_in_charging())
            if (flag_is_being_charged)
            {
#if 0  // 被充电时不给主机充电的版本
       // 如果充电座正在被充电，关闭给主机的充电，防止电流过高
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                flag_is_low_bat = 0;                     // 充电座被充电时，清除该标志位
                flag_is_enable_detect_load = 0;          // 清零该标志位，关闭对应的定时器计时功能
                delay_ms(1);
                LED_GREEN_OFF(); // 等待状态和标志位更新完成，再关闭绿灯
                flag_is_fully_charged = 0;
                LED_RED_ON();
#endif // 被充电时不给主机充电的版本

#if 1
                // 如果充电座正在被充电，不关闭给主机的充电
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                flag_is_low_bat = 0;                     // 充电座被充电时，清除该标志位
                flag_is_enable_detect_load = 0;          // 清零该标志位，关闭对应的定时器计时功能
                flag_is_detect_load_when_charged = 1;    // 还在给主机充电，并且检测到插入了充电器，那么就给这个标志位置一
                delay_ms(1);
                continue;
#endif
            }
        } // else if (CUR_STATUS_IN_CHARGING == cur_dev_status) // 当前正在给主机充电
        else if (CUR_STATUS_BE_CHARGING == cur_dev_status) // 当前正在被充电
        {
            // 状态机，是否检测到负载，只在当前语句块使用
            // 默认值只能是0，调用清ram函数后，所有变量的值就是0
            // 0--表示在充电座被充电期间，还没有开始检测是否有负载
            // 1--表示正在检测中
            // 2--检测到没有负载
            // 3--检测到有负载
            static u8 __flag_detect_load_status = 0;

            // 表示盖子的状态
            // 0 -- 初始值
            // 1 -- 盖上了盖子
            // 2 -- 未盖上盖子
            static u8 __lid_status;
            if (0 == __lid_status) // 如果是刚进入充电
            {
                if (P13D) // 如果刚进入充电，盖子就是开启的
                {
                    __lid_status = 2;
                }
                else
                {
                    __lid_status = 1; // 刚进入充电，盖子就是关闭的
                }
            }

            if (0 == __flag_detect_load_status && flag_is_detect_load_when_charged)
            {
                // 如果 __flag_detect_load_status 为默认值且 flag_is_detect_load_when_charged == 1，
                // 说明外部先将flag_is_detect_load_when_charged置一，之后才进入该代码块
                // 那么给 __flag_detect_load_status 赋值3，表示在被充电期间检测到有负载，接下来一直检测负载有没有断开
                __flag_detect_load_status = 3;
            }

            // 如果正在被充电，检测是否充满电
            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            bat_adc_val = adc_val;
            // if (adc_val >= 2124 - AD_OFFSET) // 如果电池电压大于4.15V (实际测试是4.17V，才认为充满电)
            // if (adc_val >= 2150) // 如果电池电压大于4.20V(实际测试，在充电时，电池两端电压到4V就认为充满了)
            // if (adc_val >= 2150 + AD_OFFSET) // 实际测试可能大于4.18V
            // if (adc_val >= 2150 + 20) //
            if (adc_val >= 2048) // 计算出来是4V
            {
                flag_bat_is_fully_charged = 1; // 表示充电座的电池被充满
            }

            // 检测是否断开了充电器:
            // ret_u8 = is_in_charging();
            // if (0 == ret_u8)
            if (0 == flag_is_being_charged)
            {
                // flag_bat_is_fully_charged = 0;
                // cur_dev_status = CUR_STATUS_NONE;

                if (flag_is_detect_load_when_charged) // 如果正在给主机充电
                {
                    cur_dev_status = CUR_STATUS_IN_CHARGING;
                }
                else // 如果没有给主机充电，直接关机
                {
                    cur_dev_status = CUR_STATUS_POWER_OFF;
                }

                // 退出该代码块前，先清除代码块内使用的变量：
                flag_is_detect_load_when_charged = 0;
                __lid_status = 0; // 清零，回到默认值
                __flag_detect_load_status = 0;
                continue;
            }

#if 1
            if (flag_is_open_lid && __lid_status == 1)
            {
                // 如果检测到打开了盖子
                __lid_status = 2; // 表示打开了盖子

                __flag_detect_load_status = 0;
                flag_is_enable_detect_load = 0; // 不使能检测负载的定时器计时
                delay_ms(1);
                flag_is_detect_load_when_charged = 0;
                stop_charging_the_host(); // 关闭PWM
            }
            else if (((1 == __flag_detect_load_status) ||         /* 如果正在检测有没有负载 */
                      (3 == __flag_detect_load_status)) ||        /* 如果检测到负载没有断开，继续检测 */
                     (__lid_status == 2 && 0 == flag_is_open_lid) // 如果之前盖子是开启的，现在检测到盖子盖上
            )
            {
                __lid_status = 1; // 表示盖上盖子

                // 如果盖子是关闭的
                if (0 == __flag_detect_load_status)
                {
                    // 如果还未检测过是否有负载
                    __flag_detect_load_status = 1; // 表示正在检测是否有负载
                    set_timer0_pwm_when_detecting();
                    PWM_ENABLE();
                    LED_GREEN_ON();
                    // delay_ms(4000); // -- 这期间可以检测有没有打开保护盖
                    {
                        u16 i = 0;
                        // for (i = 0; i < 4000; i++)
                        for (i = 0; i < 500; i++)
                        {
                            if (flag_is_open_lid) // 如果打开了保护盖
                            {
                                __flag_detect_load_status = 0;
                                flag_is_detect_load_when_charged = 0;
                                stop_charging_the_host(); // 关闭pwm，关闭绿灯
                                break;
                            }

                            delay_ms(1);
                        }
                    }
                }

                if ((1 == __flag_detect_load_status) || /* 如果正在检测有没有负载 */
                    (3 == __flag_detect_load_status))   /* 如果检测到负载没有断开，继续检测 */
                {
                    ret_u8 = is_detect_load();
                    if (0 == ret_u8)
                    {
                        // 如果断开了负载
                        __flag_detect_load_status = 2;
                        flag_is_detect_load_when_charged = 0;
                        stop_charging_the_host(); // 关闭PWM
                    }
                    else if (1 == ret_u8)
                    {
                        // 如果还未断开负载，或者是从检测负载到认为检测到有负载
                        if (1 == __flag_detect_load_status) // 如果刚从200多K的PWM来到这里，需要切换成300多K的PWM
                        {
                            set_timer0_pwm_when_charging();
                        }

                        __flag_detect_load_status = 3;
                        flag_is_detect_load_when_charged = 1;
                    }
                    else if (2 == ret_u8)
                    {
                        // 如果还在检测是否有负载
                    }
                }
            }
#endif
        }
        else if (CUR_STATUS_POWER_OFF == cur_dev_status)
        {
            // 低电量关机:
            low_power_scan_handle();

            if (cur_dev_status != CUR_STATUS_BE_CHARGING) // 如果不是插入了充电器唤醒的
            {
                cur_dev_status = CUR_STATUS_NONE; // 从低电量关机后又唤醒，回到正常状态
            }
        }
#endif

        __asm;
        clrwdt;
        __endasm;
    } // while (1)
}
/************************************************
;  *    @函数名            : interrupt
;  *    @说明              : 中断函数
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;

    if (KBIF & KBIE)
    {
        KBIF = 0;
    }

    if (T3IF & T3IE)
    {
        // 目前每1ms进入一次中断
        // { // 在检测负载的测试中使用
        //     timer3_cnt++;
        //     if (timer3_cnt >= 4000)
        //     {
        //         timer3_cnt = 0;
        //         flag_tim_set_scan_time_is_arrive = 1;
        //     }
        // }

        { // 检测负载的时间计数
            static volatile u16 detect_load_cnt_ms = 0;
            if (flag_is_enable_detect_load)
            {
                detect_load_cnt_ms++;
                // if (detect_load_cnt_ms >= 4000)
                if (detect_load_cnt_ms >= 2500)
                {
                    detect_load_cnt_ms = 0;
                    flag_tim_set_scan_time_is_arrive = 1;
                }
            }
            else
            {
                detect_load_cnt_ms = 0;
            }
        }

        { // 检测是否打开了保护盖
            static u8 detect_open_lid_cnt;
            static u8 undetect_open_lid_cnt;

            if (P13D) // 如果保护盖是开启的
            {
                undetect_open_lid_cnt = 0;
                detect_open_lid_cnt++;
                if (detect_open_lid_cnt >= 200) // 如果一直检测到有打开保护盖
                {
                    detect_open_lid_cnt = 0;
                    flag_is_open_lid = 1; // 表示打开了保护盖
                }
            }
            else // 如果保护盖是关闭的
            {
                detect_open_lid_cnt = 0;
                undetect_open_lid_cnt++;
                if (undetect_open_lid_cnt >= 200) // 如果一直检测到没有打开保护盖
                {
                    undetect_open_lid_cnt = 0;
                    flag_is_open_lid = 0; // 表示没有
                }
            }
        }

#if 1
        { // 检测充电的时间计数

            static u8 detect_charge_cnt;   // 在没有充电期间，检测到有充电的时间计数
            static u8 undetect_charge_cnt; // 在充电期间，检测到断开充电的时间计数

            if (flag_is_being_charged) // 如果在充电，检测是否断开充电，并进行计数
            {
                if (CHARGE_PIN) // 如果检测到充电
                {
                    undetect_charge_cnt = 0;
                }
                else
                {
                    detect_charge_cnt = 0;
                    undetect_charge_cnt++;
                    if (undetect_charge_cnt >= 200) // 如果连续 xx ms都检测到
                    {
                        undetect_charge_cnt = 0;
                        flag_is_being_charged = 0; // 表示不在充电
                    }
                }
            }
            else // 如果不在充电，检测是否有充电，并进行计数
            {
                if (CHARGE_PIN) // 如果检测到充电
                {
                    undetect_charge_cnt = 0;
                    detect_charge_cnt++;
                    if (detect_charge_cnt >= 200) // 如果连续 xx ms都检测到
                    {
                        detect_charge_cnt = 0;
                        flag_is_being_charged = 1; // 表示正在被充电
                    }
                }
                else // 如果检测不到充电
                {
                    detect_charge_cnt = 0;
                }
            }

        } // 检测充电的时间计数
#endif

#if 1
        { // 负责控制红灯点亮/熄灭的相关代码块
            static u16 blink_cnt = 0;
            static u16 fully_charged_cnt;                   // 控制满电 xx ms后，关闭红灯的变量
            if (CUR_STATUS_IN_CHARGING == cur_dev_status && /* 充电座给主机充电时 */
                flag_is_low_bat)                            /* 充电座电池处于低电量 */
            {
                // 如果在充电座给主机充电时，检测到充电座的电池处于低电量
                // LED_GREEN_OFF(); // 关闭绿灯，避免与红灯冲突

                if (blink_cnt < 65535)
                    blink_cnt++;

                // if (blink_cnt <= 350)
                if (blink_cnt <= 1000)
                {
                    LED_RED_ON();
                }
                // else if (blink_cnt <= 700)
                else if (blink_cnt <= 2000)
                {
                    LED_RED_OFF();
                }
                else
                {
                    blink_cnt = 0;
                }

                fully_charged_cnt = 0;
            }
            // else if (CUR_STATUS_BE_CHARGING == cur_dev_status && /* 充电座被充电时 */
            //          0 == flag_bat_is_fully_charged)             /* 充电座未被充满电 */
            else if (flag_is_being_charged &&        /* 充电座被充电时 */
                     0 == flag_bat_is_fully_charged) /* 充电座未被充满电 */
            {
                // 如果充电座正在被充电，红灯常亮
                // LED_GREEN_OFF(); // 关闭绿灯，避免与红灯冲突
                LED_RED_ON();

                fully_charged_cnt = 0;
            }
            else
            {
                fully_charged_cnt++;

                if (fully_charged_cnt >= 1000) // 满电 xx ms后，再关闭红灯
                {
                    fully_charged_cnt = 0;
                    LED_RED_OFF();
                }

                blink_cnt = 0;
            }
        }

#if 1     // 控制充电时，让绿灯闪烁的代码块
        { // 控制充电时，让绿灯闪烁的代码块

            static u16 led_green_blink_cnt = 0;
            // 如果充电座正在给主机充电，并且充电座的电池不处于低电量
#if 1 // 在被充电时，仍然使能给主机充电的版本
            if (flag_is_detect_load_when_charged ||
                (CUR_STATUS_IN_CHARGING == cur_dev_status &&
                 0 == flag_is_low_bat &&
                 0 == flag_is_fully_charged))
#endif // 在被充电时，仍然使能给主机充电的版本
       // if (CUR_STATUS_IN_CHARGING == cur_dev_status &&
       //     0 == flag_is_low_bat &&
       //     0 == flag_is_fully_charged)
            {
                // if (led_green_blink_cnt < 65535)
                {
                    led_green_blink_cnt++;
                }

                if (led_green_blink_cnt <= 500)
                {
                    LED_GREEN_ON();
                }
                else if (led_green_blink_cnt <= 1000)
                {
                    LED_GREEN_OFF();
                }
                else
                {
                    led_green_blink_cnt = 0;
                }
            }
            else
            {
                led_green_blink_cnt = 0;
            }

        } // 控制充电时，让绿灯闪烁的代码块
#endif // 控制充电时，让绿灯闪烁的代码块

#endif

        T3IF = 0;
    }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
