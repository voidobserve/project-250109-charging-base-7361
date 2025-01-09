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
    T0DATA = 64 + 10;
    PWMCR0 = 0x00; // 正向输出
    PWMCR1 = 0x18; // 时钟源FHOSC × 2  普通模式
    // PWM0OPS = 0; // 选择P16端口输出 (可以不写，默认就是0)

    // T0EN = 1;
    T0EN = 0;   // 关闭定时器
    PWM0EC = 0; // 不使能PWM0输出
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
    case ADC_CHANNEL_BAT: // 选择 1/4 VDD 通道
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
        break;

    case ADC_CHANNEL_LOAD: // P15 AN9，检测充电座 放电
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 0;
        ADCHS0 = 1;
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

// 获取adc单次转换后的值
u16 adc_get_val_once(void)
{
    u16 g_temp_value = 0;
    ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
    while (!ADEOC)
        ;                // 等待转换完成
    g_temp_value = ADRH; // 取出转换后的值
    g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
    return g_temp_value;
}

// 对扫描到的按键事件进行处理
void key_event_handle(void)
{
    // 处理完成后，清除按键事件
    // key_event = KEY_EVENT_NONE;
}

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
    P13PU = 0; // 上拉
    P13OE = 0; // 输入模式
    P13KE = 1; // 使能键盘中断

    GIE = 1;
}

// 充电检测
void charge_scan(void)
{
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
            flag_is_in_charging = 1;
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
            flag_is_in_charging = 0;
        }
    }
}

void power_on_scan(void)
{
    // 开机时，检测是否打开了盖子：
    if (flag_is_device_open)
    {
        if (P13D)
        {
            cnt = 0;
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
                // 如果检测到是关机：
                flag_is_device_open = 0;
            }
        }
    }
    else // 关机时，检测是否关上了盖子：
    {
        if (0 == P13D)
        {
            cnt = 0;
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
                // 如果检测到是开机：
                flag_is_device_open = 1;
            }
        }
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

    while (1)
    {
        power_on_scan();
        if (flag_is_device_open)
        {
            // 如果设备可以开机，检测有没有负载，如果没有负载 -> 关机
            // 如果给负载充满电 -> 关机
            
        }
        else
        {
            // 如果设备不能开机，进入低功耗，由键盘中断、外部5V输入来唤醒

        }
       

        __asm;
        clrwdt;
        __endasm;
    }
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

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
