/******************************************************************************
;  *       @�ͺ�                   : MC32F7361
;  *       @��������               : 2021.12.21
;  *       @��˾/����              : SINOMCU-FAE
;  *       @����΢����֧��         : 2048615934
;  *       @����΢����             : http://www.sinomcu.com/
;  *       @��Ȩ                   : 2021 SINOMCU��˾��Ȩ����.
;  *---------------------- ���� ---------------------------------
;  *                   ��������ʱʹ��ȫ�ֱ���
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
// �������LED                                      //
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
#define CHARGE_PIN (P03D) // ���������
#else
#define CHARGE_PIN (P04D) // ���������
#endif

// ����adc��ͨ��
enum
{
    ADC_CHANNEL_NONE,
    ADC_CHANNEL_BAT,  // ���
    ADC_CHANNEL_LOAD, // ����
};

//===============Field Protection Variables===============
volatile u8 abuf;
volatile u8 statusbuf;

//===============Global Variable===============
volatile u8 i; // ѭ������ֵ
volatile u8 cnt; // ѭ����ʹ�õ����¼�����
volatile u16 adc_val;

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
#define flag_is_in_charging flag1.bits.bit0 // ��־λ���Ƿ��ڳ��
#define flag_is_device_open flag1.bits.bit1 // ��־λ���豸�Ƿ���
#define flag_is_detect_host flag1.bits.bit2 // ��־λ���Ƿ��⵽������

#define flag_is_enable_detect flag1.bits.bit3 // ��־λ���Ƿ�ʹ���������
// #define flag_is_detect_


// ���뼶��ʱ (����1%���ڣ�1ms��10ms��100ms��ʱ������С��1%)
// ǰ��������FCPU = FHOSC / 4
void delay_ms(u16 xms)
{
    while (xms)
    {
        u16 i = 572;
        while (i--)
        {
            Nop();
        }
        xms--; // �� --��������while()�ж��������棬����ʡ�ռ�

        __asm;
        clrwdt; // ι��
        __endasm;
    }
}

// #if USE_MY_DEBUG
#define DEBUG_PIN P22D
#if 1 // ���³���Լռ��81�ֽڿռ�
// ͨ��һ�������������(����һ��Լ400ms)
// #define DEBUG_PIN P22D
void send_data_msb(u32 send_data)
{
    // �ȷ��͸�ʽͷ
    // __set_input_pull_up(); // �ߵ�ƽ
    DEBUG_PIN = 1;
    delay_ms(15);
    // __set_output_open_drain(); // �͵�ƽ
    DEBUG_PIN = 0;
    delay_ms(7); //

    for (u8 i = 0; i < 32; i++)
    {
        if ((send_data >> (32 - 1 - i)) & 0x01)
        {
            // ���Ҫ�����߼�1
            // __set_input_pull_up();  	   // �ߵ�ƽ
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // �͵�ƽ
            DEBUG_PIN = 0;
            delay_ms(10); //
        }
        else
        {
            // ���Ҫ�����߼�0
            // __set_input_pull_up();  	   // �ߵ�ƽ
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // �͵�ƽ
            DEBUG_PIN = 0;
            delay_ms(5); //
        }
    }

    // �������Ϊ�͵�ƽ
    // __set_output_open_drain(); // �͵�ƽ
    DEBUG_PIN = 0;
    delay_ms(1);
    DEBUG_PIN = 1;
    delay_ms(1);
    DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/
