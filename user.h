/******************************************************************************
;  *       @型号                   : MC32F7361
;  *       @创建日期               : 2021.12.21
;  *       @公司/作者              : SINOMCU-FAE
;  *       @晟矽微技术支持         : 2048615934
;  *       @晟矽微官网             : http://www.sinomcu.com/
;  *       @版权                   : 2021 SINOMCU公司版权所有.
;  *---------------------- 建议 ---------------------------------
;  *                   变量定义时使用全局变量
******************************************************************************/
#ifndef USER
#define USER
#include "mc32-common.h"
#include "MC32F7361.h"

/*****************************************************************
;       Function : Define variables
;*****************************************************************/

#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long int
#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int

#define DEF_SET_BIT0 0x01
#define DEF_SET_BIT1 0x02
#define DEF_SET_BIT2 0x04
#define DEF_SET_BIT3 0x08
#define DEF_SET_BIT4 0x10
#define DEF_SET_BIT5 0x20
#define DEF_SET_BIT6 0x40
#define DEF_SET_BIT7 0x80

#define DEF_CLR_BIT0 0xFE
#define DEF_CLR_BIT1 0xFD
#define DEF_CLR_BIT2 0xFB
#define DEF_CLR_BIT3 0xF7
#define DEF_CLR_BIT4 0xEF
#define DEF_CLR_BIT5 0xDF
#define DEF_CLR_BIT6 0xBF
#define DEF_CLR_BIT7 0x7F

#define FAIL 1
#define PASS 0

#define USE_MY_DEBUG 0
#define AD_OFFSET 41 // 检测到的ad值与实际的电压值有偏差，要减去这个值
// ===================================================
// 充电座的LED                                      //
// ===================================================
#define LED_RED_PIN (P17D)
#define LED_GREEN_PIN (P00D)
#define LED_RED_ON()         \
    {                        \
        do                   \
        {                    \
            LED_RED_PIN = 1; \
        } while (0);         \
    }
#define LED_RED_OFF()        \
    {                        \
        do                   \
        {                    \
            LED_RED_PIN = 0; \
        } while (0);         \
    }
#define LED_GREEN_ON()         \
    {                          \
        do                     \
        {                      \
            LED_GREEN_PIN = 1; \
        } while (0);           \
    }
#define LED_GREEN_OFF()        \
    {                          \
        do                     \
        {                      \
            LED_GREEN_PIN = 0; \
        } while (0);           \
    }

#define PWM_ENABLE()    \
    {                   \
        do              \
        {               \
            PWM0EC = 1; \
            T0EN = 1;   \
        } while (0);    \
    }
#define PWM_DISABLE()   \
    {                   \
        do              \
        {               \
            PWM0EC = 0; \
            T0EN = 0;   \
        } while (0);    \
    }

#if USE_MY_DEBUG
#define CHARGE_PIN (P03D) // 充电检测引脚
#else
#define CHARGE_PIN (P04D) // 充电检测引脚
#endif

// ===================================================
// 设备当前状态的相关配置                            //
// ===================================================
enum
{
    CUR_STATUS_NONE = 0,
    CUR_STATUS_BE_CHARGING, // 被充电
    CUR_STATUS_IN_CHARGING, // 给主机充电
    // CUR_STATUS_IS_FULLY_CHARGED, // 给主机充满电
    CUR_STATUS_POWER_OFF, // 低电量关机
};
volatile u8 cur_dev_status;

#if 0
// 低通滤波器系数 alpha = 0.3
#define FILTER_ALPHA 3
#define FILTER_SCALE 10
#endif
#if 0
    for (i = 0; i < 100; i++)
    adc_val = adc_get_val();
    // 低通滤波计算: Y(n) = α * X(n) + (1-α) * Y(n-1)
    filter_value = (FILTER_ALPHA * adc_val + (FILTER_SCALE - FILTER_ALPHA) * filter_value) / FILTER_SCALE;
    adc_val = filter_value;

    if (adc_val > ADC_VAL_LOAD_THRESHOLD)
    {
        return 0;
    }
    else
    {
        return 1;
    }
#endif

// ===================================================
// 充电座检测负载的相关配置                            //
// ===================================================
// 检测到有负载的ad值
// #define ADC_VAL_LOAD_THRESHOLD ((2625 + 2315) / 2) // ad值通过测试得出
// #define ADC_VAL_LOAD_THRESHOLD ((2477 + 2129) / 2)// ad值通过测试得出
// #define ADC_VAL_LOAD_THRESHOLD ((1260 + 2463) / 2) // ad值通过测试得出
// #define ADC_VAL_LOAD_THRESHOLD (1400) // ad值通过测试得出
// #define ADC_VAL_LOAD_THRESHOLD (850) // ad值通过测试得出
#define ADC_VAL_LOAD_THRESHOLD (820) // ad值通过测试得出
// #define ADC_VAL_LOAD_THRESHOLD (1870) // ad值通过测试得出


// volatile u32 timer3_cnt;             // 测试时使用
// volatile u8 flag_4s;                 // 测试时使用
u32 detect_load_cnt;                 // 测试时使用
u32 undetect_load_cnt;               // 测试时使用
// 检测条件1--测试可以检测到负载 （充电座不同电压下，都可以检测到负载）
#define DETECT_LOAD_ADC_VAL (3767)   // 检测到负载时，对应的ad阈值
#define UNDETECT_LOAD_ADC_VAL (3986) // 未检测到负载时，对应的ad阈值

// 检测条件2--测试可以检测到负载 （充电座不同电压下，都可以检测到负载）
// #define DETECT_LOAD_ADC_VAL (3882)   // 检测到负载时，对应的ad阈值
// #define UNDETECT_LOAD_ADC_VAL (3983) // 未检测到负载时，对应的ad阈值

// 定义adc的通道
enum
{
    ADC_CHANNEL_NONE,
    ADC_CHANNEL_BAT,  // 电池
    ADC_CHANNEL_LOAD, // 负载
};

//===============Field Protection Variables===============
volatile u8 abuf;
volatile u8 statusbuf;

//===============Global Variable===============
volatile u8 i;   // 循环计数值
volatile u8 cnt; // 循环中使用到的事件计数
volatile u8 ret_u8;
volatile u16 adc_val;
volatile u16 tmp_val;

// volatile u16 max_adc_val; // 检测负载时，用于存放一段时间内检测到的最大的ad值

//============Define  Flag=================
typedef union
{
    unsigned char byte;
    struct
    {
        u8 bit0 : 1;
        u8 bit1 : 1;
        u8 bit2 : 1;
        u8 bit3 : 1;
        u8 bit4 : 1;
        u8 bit5 : 1;
        u8 bit6 : 1;
        u8 bit7 : 1;
    } bits;
} bit_flag;
volatile bit_flag flag1;
#define flag_is_low_bat flag1.bits.bit0        // 标志位，是否在给主机充电时检测到低电量
#define flag_is_fully_charged flag1.bits.bit1  // 标志位，是否给主机充满电


#define flag_is_enable_detect_load flag1.bits.bit3 // 标志位，是否使能检测负载的功能
#define flag_4s flag1.bits.bit4 // 标志位，在检测负载时，是否持续检测了4s

#define flag_is_open_lid flag1.bits.bit5 // 标志位，表示是否打开了收纳盒,0--未打开，1--打开
#define flag_bat_is_fully_charged flag1.bits.bit6 // 标志位，表示充电座的电池是否被充满电

#define flag_is_detect_load_when_charged flag1.bits.bit7 // 标志位，是否在被充电时检测到了负载

// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 4
void delay_ms(u16 xms)
{
    while (xms)
    {
        u16 i = 572;
        while (i--)
        {
            Nop();
        }
        xms--; // 把 --操作放在while()判断条件外面，更节省空间

        __asm;
        clrwdt; // 喂狗
        __endasm;
    }
}

// #if USE_MY_DEBUG
#define DEBUG_PIN P22D
#if 1 // 以下程序约占用81字节空间
// 通过一个引脚输出数据(发送一次约400ms)
// #define DEBUG_PIN P22D
void send_data_msb(u32 send_data)
{
    // 先发送格式头
    // __set_input_pull_up(); // 高电平
    DEBUG_PIN = 1;
    delay_ms(15);
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(7); //

    for (u8 i = 0; i < 32; i++)
    {
        if ((send_data >> (32 - 1 - i)) & 0x01)
        {
            // 如果要发送逻辑1
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(10); //
        }
        else
        {
            // 如果要发送逻辑0
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(5); //
        }
    }

    // 最后，设置为低电平
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(1);
    DEBUG_PIN = 1;
    delay_ms(1);
    DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/
