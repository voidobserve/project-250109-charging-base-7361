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

#define USE_MY_DEBUG 0
#define AD_OFFSET 41 // ��⵽��adֵ��ʵ�ʵĵ�ѹֵ��ƫ�Ҫ��ȥ���ֵ
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
#define CHARGE_PIN (P03D) // ���������
#else
#define CHARGE_PIN (P04D) // ���������
#endif

// ===================================================
// �豸��ǰ״̬���������                            //
// ===================================================
enum
{
    CUR_STATUS_NONE = 0,
    CUR_STATUS_BE_CHARGING, // �����
    CUR_STATUS_IN_CHARGING, // ���������
    // CUR_STATUS_IS_FULLY_CHARGED, // ������������
    CUR_STATUS_POWER_OFF, // �͵����ػ�
};
volatile u8 cur_dev_status;

#if 0
// ��ͨ�˲���ϵ�� alpha = 0.3
#define FILTER_ALPHA 3
#define FILTER_SCALE 10
#endif
#if 0
    for (i = 0; i < 100; i++)
    adc_val = adc_get_val();
    // ��ͨ�˲�����: Y(n) = �� * X(n) + (1-��) * Y(n-1)
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
// �������⸺�ص��������                            //
// ===================================================
// ��⵽�и��ص�adֵ
// #define ADC_VAL_LOAD_THRESHOLD ((2625 + 2315) / 2) // adֵͨ�����Եó�
// #define ADC_VAL_LOAD_THRESHOLD ((2477 + 2129) / 2)// adֵͨ�����Եó�
// #define ADC_VAL_LOAD_THRESHOLD ((1260 + 2463) / 2) // adֵͨ�����Եó�
// #define ADC_VAL_LOAD_THRESHOLD (1400) // adֵͨ�����Եó�
// #define ADC_VAL_LOAD_THRESHOLD (850) // adֵͨ�����Եó�
#define ADC_VAL_LOAD_THRESHOLD (820) // adֵͨ�����Եó�
// #define ADC_VAL_LOAD_THRESHOLD (1870) // adֵͨ�����Եó�


// volatile u32 timer3_cnt;             // ����ʱʹ��
// volatile u8 flag_4s;                 // ����ʱʹ��
u32 detect_load_cnt;                 // ����ʱʹ��
u32 undetect_load_cnt;               // ����ʱʹ��
// �������1--���Կ��Լ�⵽���� ���������ͬ��ѹ�£������Լ�⵽���أ�
#define DETECT_LOAD_ADC_VAL (3767)   // ��⵽����ʱ����Ӧ��ad��ֵ
#define UNDETECT_LOAD_ADC_VAL (3986) // δ��⵽����ʱ����Ӧ��ad��ֵ

// �������2--���Կ��Լ�⵽���� ���������ͬ��ѹ�£������Լ�⵽���أ�
// #define DETECT_LOAD_ADC_VAL (3882)   // ��⵽����ʱ����Ӧ��ad��ֵ
// #define UNDETECT_LOAD_ADC_VAL (3983) // δ��⵽����ʱ����Ӧ��ad��ֵ

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
volatile u8 i;   // ѭ������ֵ
volatile u8 cnt; // ѭ����ʹ�õ����¼�����
volatile u8 ret_u8;
volatile u16 adc_val;
volatile u16 tmp_val;

// volatile u16 max_adc_val; // ��⸺��ʱ�����ڴ��һ��ʱ���ڼ�⵽������adֵ

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
#define flag_is_low_bat flag1.bits.bit0        // ��־λ���Ƿ��ڸ��������ʱ��⵽�͵���
#define flag_is_fully_charged flag1.bits.bit1  // ��־λ���Ƿ������������


#define flag_is_enable_detect_load flag1.bits.bit3 // ��־λ���Ƿ�ʹ�ܼ�⸺�صĹ���
#define flag_4s flag1.bits.bit4 // ��־λ���ڼ�⸺��ʱ���Ƿ���������4s

#define flag_is_open_lid flag1.bits.bit5 // ��־λ����ʾ�Ƿ�������ɺ�,0--δ�򿪣�1--��
#define flag_bat_is_fully_charged flag1.bits.bit6 // ��־λ����ʾ������ĵ���Ƿ񱻳�����

#define flag_is_detect_load_when_charged flag1.bits.bit7 // ��־λ���Ƿ��ڱ����ʱ��⵽�˸���

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
