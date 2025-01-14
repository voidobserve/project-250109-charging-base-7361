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

#define USE_MY_DEBUG 1
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

#define PWM_ENABEL()    \
    {                   \
        do              \
        {               \
            PWM0EC = 1; \
            T0EN = 1;   \
        } while (0);    \
    }
#define PWM_DISABEL()   \
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
    // CUR_STATUS_LOW_BAT, // 低电量
};
volatile u8 cur_dev_status;

// ===================================================
// 充电座检测负载的相关配置                            //
// ===================================================
// 检测到有负载的ad值
// #define ADC_VAL_LOAD_THRESHOLD ((2625 + 2315) / 2) // ad值通过测试得出
#define ADC_VAL_LOAD_THRESHOLD (2470) // ad值通过测试得出
// #define ADC_VAL_LOAD_THRESHOLD (3000) // ad值通过测试得出

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
// #define flag_is_in_charging flag1.bits.bit0      // 标志位，是否在充电
// #define flag_is_device_open flag1.bits.bit1      // 标志位，设备是否开启
// #define flag_is_charging_to_host flag1.bits.bit2 // 标志位，是否检测到了主机
// #define flag_is_enable_detect flag1.bits.bit3 // 标志位，是否使能主机检测
#define flag_is_low_bat flag1.bits.bit4 // 标志位，是否在充电时检测到低电量

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
