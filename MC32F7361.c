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
// 250KHz：
void timer0_pwm_config(void)
{
    T0CR = 0x49; // 使能PWM,FTMR,2分频 (约每0.03125us计数一次)
    // T0CNT = 50 - 1;
    // 理论上重装载值应该是128，但是时钟是RC振荡得来的，不准确，需要加上补偿
    T0LOAD = (128 + 10) - 1; //
    // T0DATA = 23 + 10;
    // T0DATA = 59 + 10;
    T0DATA = 49 + 10;


    PWMCR0 = 0x00; // 正向输出
    PWMCR1 = 0x18; // 时钟源FHOSC × 2  普通模式
    // PWM0OPS = 0; // 选择P16端口输出 (可以不写，默认就是0)

    // T0EN = 1;
    T0EN = 0;   // 关闭定时器
    PWM0EC = 0; // 不使能PWM0输出
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
u16 adc_get_max_val(void)
{
    u8 i = 0; // adc采集次数的计数
    volatile u16 g_temp_value = 0;
    volatile u16 g_adcmax = 0;

    // 采集40次
    for (i = 0; i < 40; i++)
    {
        ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
        while (!ADEOC)
            ;                // 等待转换完成
        g_temp_value = ADRH; // 取出转换后的值
        g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);

        if (g_temp_value > g_adcmax)
            g_adcmax = g_temp_value; // 最大
    }

    return g_adcmax;
}

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

// 充电检测
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
        // cnt = 0;
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
        // cnt = 0;
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

// 是否检测到负载 -- 调用一次约72ms
u8 is_detect_load(void)
{
    u16 max_adc_val = 0;
    u8 i = 0;
    adc_sel_channel(ADC_CHANNEL_LOAD);
    for (i = 0; i < 100; i++)
    {
        // adc_val = adc_get_max_val(); // 用这个函数会采集到4095
        adc_val = adc_get_val();
        if (adc_val > max_adc_val)
        {
            max_adc_val = adc_val;
        }
    }

    if (max_adc_val > ADC_VAL_LOAD_THRESHOLD)
    {
        // 有负载
        return 1;
    }
    else
    {
        // 没有负载
        return 0;
    }
}

// 关闭给主机的充电：
void stop_charging_the_host(void)
{
    LED_GREEN_OFF();
    PWM_DISABLE(); // 关闭控制充电的pwm
}

void low_power_scan_handle(void)
{
    if (CUR_STATUS_NONE == cur_dev_status || CUR_STATUS_POWER_OFF == cur_dev_status)
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
        // LVDEN = 0; // 是否关闭LVD都不影响待机电流
        HFEN = 0; // 关闭高速时钟
        LFEN = 0; // 关闭低速时钟
        // 休眠前注意关闭不用的外设
        Nop();
        Nop();
        Stop();
        Nop();
        Nop();
        HFEN = 1; // 开启高速时钟
                  // LVDEN = 1;
#if USE_MY_DEBUG
        P03KE = 0;
#else
        P04KE = 0;
#endif
        P13KE = 0; // 关闭键盘中断
        KBIE = 0;
        KBIF = 0;

        // // 检测电池电压是否小于3.0V或未进行充电
        // if (CHARGE_PIN)
        // {
        //     // 如果检测到充电，什么也不做
        //     // DEBUG_PIN = 1;
        // }
        // else
        // {
        //     // 如果没有检测到充电，检测电池电量是否大于3.0V，才允许开机
        //     // adc_config();
        //     // adc_sel_channel(ADC_CHANNEL_BAT);
        //     // adc_val = adc_get_val();
        //     // if (adc_val < 1485) // 如果检测到电池电压小于2.9V
        //     // {
        //     //     // goto low_power_label;  // 这里会导致无法从正常的电压下启动
        //     // }
        // }

        // DEBUG_PIN = ~DEBUG_PIN; // 测试是否能唤醒单片机

        Sys_Init();
        delay_ms(1);
        GIE = 1;
    }
}

/************************************************
;  *    @函数名            : main
;  *    @说明              : 主程序
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void main(void)
{
    Sys_Init();
    delay_ms(10);

    LED_GREEN_ON();
    delay_ms(1000);
    LED_GREEN_OFF();

#if 0
    // flag_is_low_bat = 1; // 测试时使用 
    LED_GREEN_ON();
    PWM_ENABLE(); 
    T3EN = 0;
#endif

    while (1)
    {
#if 0 // 测试在无线充电时，充电座检测到的ad值
        adc_sel_channel(ADC_CHANNEL_LOAD);
        for (i = 0; i < 100; i++)
        {
            // adc_val = adc_get_max_val(); // 用这个函数会采集到4095
            adc_val = adc_get_val();
            // if (adc_val > tmp_val)
            {
                tmp_val = adc_val;
                send_data_msb(tmp_val);
            }
        }
#endif

#if 0
        // ret_u8 = is_open_lid();
        // if (ret_u8)
        // {
        //     LED_GREEN_ON();
        // }
        // else
        // {
        //     LED_GREEN_OFF();
        // }

        ret_u8 = is_detect_load();
        if (ret_u8)
        {
            LED_GREEN_ON();
        }
        else
        {
            LED_GREEN_OFF();
        }
#endif

#if 1
        if (CUR_STATUS_NONE == cur_dev_status)
        {
            // 如果当前设备没有任何操作，检测是否有
            // 是否盖上盖子，
            // 是否在被充电 ，外部5V输入
            ret_u8 = is_open_lid();
            if (0 == ret_u8)
            {
                // 如果设备可以开机，检测有没有负载，如果没有负载 -> 关机
                // 如果给负载充满电 -> 关机

                // 打开充电
                PWM_ENABLE();
                LED_GREEN_ON(); //
                delay_ms(4000); // 等待电平稳定
                flag_is_detecting_load = 1;
                cnt = 0;
                while (flag_is_detecting_load)
                {
                    ret_u8 = is_detect_load();
                    if (ret_u8)
                    {
                        // 如果检测到有负载:
                        if (cnt < 255)
                            cnt++;
                    }
                    else
                    {
                    }

                    if (1 == is_open_lid())
                    {
                        break;
                    }
                }

                if (cnt >= 50)
                {
                    // 有负载
                    cur_dev_status = CUR_STATUS_IN_CHARGING;
                }
                else
                {
                    // 没有负载
                    stop_charging_the_host();
                }
            }

            ret_u8 = is_in_charging();
            if (ret_u8)
            {
                // 如果充电座正在被充电，关闭给主机的充电
                stop_charging_the_host();

                LED_RED_ON();
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
            }

            // 如果执行到这里还是没有改变状态，进入低功耗
            low_power_scan_handle();
        }
        else if (CUR_STATUS_IN_CHARGING == cur_dev_status)
        {
            // 如果正在给主机充电，检测：
            // 是否插入了充电器
            // 是否打开了盖子
            // 检测是否低电量

            ret_u8 = is_detect_load(); // 调用一次约72-73ms
            if (ret_u8)
            {
                // 有负载
                cnt = 0;
            }
            else
            {
                // 没有负载
                cnt++;
            }

            // DEBUG_PIN = ~DEBUG_PIN; // 目前循环一次约75ms

            if (cnt >= 50)
            {
                cnt = 0;
                // 如果确实没有检测到负载 说明已经给主机充满电
                flag_is_fully_charged = 1;
                PWM_DISABLE(); // 关闭控制充电的pwm
                LED_GREEN_ON();
            }

            if (flag_is_fully_charged)
            {
                LED_GREEN_ON();
            }

            ret_u8 = is_open_lid();
            if (ret_u8)
            {
                // 如果打开了收纳盒，断开充电：
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_NONE;
                flag_is_low_bat = 0; // 不给主机充电时，清空该标志位
                flag_is_fully_charged = 0;
            }

            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            // if (adc_val < 1485 - AD_OFFSET) // 2.9V
            if (adc_val < 1587 - AD_OFFSET) // 3.1V
            {
                // 如果检测到电池电电量小于 xx V
                // 断开充电：
                // DEBUG_PIN = ~DEBUG_PIN;
                cur_dev_status = CUR_STATUS_POWER_OFF;
                flag_is_fully_charged = 0;
                stop_charging_the_host();
            }
            else if (adc_val < 1638 - AD_OFFSET) // 如果充电座的电池电量小于3.2V
            {
                //
                flag_is_low_bat = 1;
                flag_is_fully_charged = 0;
                LED_GREEN_OFF(); // 关掉绿灯，让红灯慢闪
            }
            else
            {
                flag_is_low_bat = 0;
                // 如果不小于3.2V，重新点亮绿灯
                // (防止电池电压从3.2V调到3.2以下，又升回3.2V，指示灯会全部熄灭，不会点亮绿色或红色的指示灯的情况)
                // LED_GREEN_ON();
            }

            if (0 == flag_is_low_bat)
            {
                // 如果不处于低电量，并且不在充电，关闭红灯
                // 防止低电量时，刚好闪烁到红灯点亮的情况，又退出了低电量
                LED_RED_OFF();
            }

            ret_u8 = is_in_charging();
            if (ret_u8)
            {
                // 如果充电座正在被充电，关闭给主机的充电，防止电流过高
                // LED_GREEN_OFF();
                // PWM_DISABLE(); // 关闭控制充电的pwm
                // LED_RED_ON();
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                flag_is_fully_charged = 0;
                LED_RED_ON();
            }
        }
        else if (CUR_STATUS_BE_CHARGING == cur_dev_status)
        {
            // 如果正在被充电，检测是否充满电
            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            if (adc_val >= 2124 - AD_OFFSET) // 如果电池电压大于4.15V (实际测试是4.17V，才认为充满电)
            {
                LED_RED_OFF();

                // 等到拔出了充电器，再执行其他操作
                while (1)
                {
                    ret_u8 = is_in_charging();
                    if (0 == ret_u8)
                    {
                        cur_dev_status = CUR_STATUS_NONE;
                        break;
                    }

                    __asm;
                    clrwdt;
                    __endasm;
                }
            }

            flag_is_low_bat = 0; // 充电时，清除该标志位

            ret_u8 = is_in_charging();
            if (0 == ret_u8)
            {
                LED_RED_OFF();
                cur_dev_status = CUR_STATUS_NONE;
            }
        }
        else if (CUR_STATUS_POWER_OFF == cur_dev_status)
        {
            low_power_scan_handle();
            // 从低电量关机后又唤醒，回到正常状态

            // DEBUG_PIN = ~DEBUG_PIN;
            cur_dev_status = CUR_STATUS_NONE;
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
        static u16 blink_cnt = 0;
        if (CUR_STATUS_IN_CHARGING == cur_dev_status && flag_is_low_bat)
        {
            // 如果在充电座给主机充电时，检测到充电座的电池处于低电量
            if (blink_cnt < 65535)
                blink_cnt++;

            if (blink_cnt <= 400)
            {
                LED_RED_ON();
            }
            else if (blink_cnt <= 800)
            {
                LED_RED_OFF();
            }
            else
            {
                blink_cnt = 0;
            }
        }
        else
        {
            // LED_RED_OFF();
            blink_cnt = 0;
        }

        //
        {
            static u16 led_green_blink_cnt = 0;
            // 如果充电座正在给主机充电，并且充电座的电池不处于低电量
            if (CUR_STATUS_IN_CHARGING == cur_dev_status &&
                0 == flag_is_low_bat &&
                0 == flag_is_fully_charged)
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
        }

        { // 负责在刚检测到霍尔时，控制持续检测负载的时间
            static u16 __detecting_load_cnt = 0;
            if (flag_is_detecting_load)
            {
                // if (__detecting_load_cnt < 65535)
                __detecting_load_cnt++;

                if (__detecting_load_cnt >= 4000)
                {
                    flag_is_detecting_load = 0;
                }
            }
            else
            {
                __detecting_load_cnt = 0;
            }
        }

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
