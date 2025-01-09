/******************************************************************************
;  *       @�ͺ�                 : MC32F7361
;  *       @��������             : 2021.12.21
;  *       @��˾/����            : SINOMCU-FAE
;  *       @����΢����֧��       : 2048615934
;  *       @����΢����           : http://www.sinomcu.com/
;  *       @��Ȩ                 : 2021 SINOMCU��˾��Ȩ����.
;  *----------------------ժҪ����---------------------------------
;  *
******************************************************************************/

#include "user.h"

/************************************************
;  *    @������          : CLR_RAM
;  *    @˵��            : ��RAM
;  *    @�������        :
;  *    @���ز���        :
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
;  *    @������            : IO_Init
;  *    @˵��              : IO��ʼ��
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io������λ
    OEP0 = 0x3F;   // io�ڷ��� 1:out  0:in
    PUP0 = 0x00;   // io����������   1:enable  0:disable
    PDP0 = 0x00;   // io����������   1:enable  0:disable
    P0ADCR = 0x00; // io����ѡ��  1:ģ������  0:ͨ��io

    IOP1 = 0x00;   // io������λ
    OEP1 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP1 = 0x00;   // io����������   1:enable  0:disable
    PDP1 = 0x00;   // io����������   1:enable  0:disable
    P1ADCR = 0x00; // io����ѡ��  1:ģ������  0:ͨ��io

    IOP2 = 0x00; // io������λ
    OEP2 = 0x0F; // io�ڷ��� 1:out  0:in
    PUP2 = 0x00; // io����������   1:enable  0:disable
    PDP2 = 0x00; // io����������   1:enable  0:disablea

    PMOD = 0x00;  // P00��P01��P13 io�˿�ֵ�ӼĴ��������������
    DRVCR = 0x80; // ��ͨ����
}

/************************************************
;  *    @������            : ADC_Init
;  *    @˵��              : ADC��ʼ��
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void adc_config(void)
{
#if USE_MY_DEBUG
    // ��������ŵ���������ţ�
    P11OE = 0; // ����ģʽ
    P11DC = 1; // ģ��ģʽ
#else
    // ��������ŵ���������ţ�
    P15OE = 0; // ����ģʽ
    P15DC = 1; // ģ��ģʽ
#endif

    ADCR0 = 0x0B; // 12λ���ȣ�ʹ��adc
    ADCR1 = 0x80; // adcת��ʱ��ѡ�� FHIRC/32��ʹ�ÃȲ�2.0V�ο���ѹ
    ADCR2 = 0x0F; // ����ʱ�䣬ֻ�̶ܹ���15 �� ADCLK
}

// ���Ƴ���pwm
// 250KHz��
void timer0_pwm_config(void)
{
    T0CR = 0x49; // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    // T0CNT = 50 - 1;
    // ��������װ��ֵӦ����128������ʱ����RC�񵴵����ģ���׼ȷ����Ҫ���ϲ���
    T0LOAD = (128 + 10) - 1; //
    T0DATA = 64 + 10;
    PWMCR0 = 0x00; // �������
    PWMCR1 = 0x18; // ʱ��ԴFHOSC �� 2  ��ͨģʽ
    // PWM0OPS = 0; // ѡ��P16�˿���� (���Բ�д��Ĭ�Ͼ���0)

    // T0EN = 1;
    T0EN = 0;   // �رն�ʱ��
    PWM0EC = 0; // ��ʹ��PWM0���
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
    case ADC_CHANNEL_BAT: // ѡ�� 1/4 VDD ͨ��
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
        break;

    case ADC_CHANNEL_LOAD: // P15 AN9��������� �ŵ�
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 0;
        ADCHS0 = 1;
        break;

    default:
        break;
    }

    delay_ms(1); // �ȴ�adc�ȶ�
}

// ��ȡadc����ת�����ֵ
u16 adc_get_val(void)
{
    u8 i = 0; // adc�ɼ������ļ���
    u16 g_temp_value = 0;
    u32 g_tmpbuff = 0;
    u16 g_adcmax = 0;
    u16 g_adcmin = 0xFFFF;

    // �ɼ�20�Σ�ȥ��ǰ���β�������ȥ��һ�����ֵ��һ����Сֵ����ȡƽ��ֵ
    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
        while (!ADEOC)
            ;                // �ȴ�ת�����
        g_temp_value = ADRH; // ȡ��ת�����ֵ
        g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
        if (i < 2)
            continue; // ����ǰ���β�����
        if (g_temp_value > g_adcmax)
            g_adcmax = g_temp_value; // ���
        if (g_temp_value < g_adcmin)
            g_adcmin = g_temp_value; // ��С
        g_tmpbuff += g_temp_value;
    }
    g_tmpbuff -= g_adcmax;           // ȥ��һ�����
    g_tmpbuff -= g_adcmin;           // ȥ��һ����С
    g_temp_value = (g_tmpbuff >> 4); // ����16��ȡƽ��ֵ

    return g_temp_value;
}

// ��ȡadc����ת�����ֵ
u16 adc_get_val_once(void)
{
    u16 g_temp_value = 0;
    ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
    while (!ADEOC)
        ;                // �ȴ�ת�����
    g_temp_value = ADRH; // ȡ��ת�����ֵ
    g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
    return g_temp_value;
}

// ��ɨ�赽�İ����¼����д���
void key_event_handle(void)
{
    // ������ɺ���������¼�
    // key_event = KEY_EVENT_NONE;
}

/************************************************
;  *    @������            : Sys_Init
;  *    @˵��              : ϵͳ��ʼ��
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    timer0_pwm_config();
    adc_config();

#if USE_MY_DEBUG
    // ��� �������ţ�
    // P03PD = 1; // ʹ���������� (���ⲿ���������Ҳ��ܴ��������裬���ⲻ��)
    P03OE = 0; // ����ģʽ
#else
    // ��� �������ţ�
    // P04PD = 1; // ʹ���������� (���ⲿ���������Ҳ��ܴ��������裬���ⲻ��)
    P04OE = 0; // ����ģʽ
#endif

    // ������Ԫ����������:
    P13PU = 0; // ����
    P13OE = 0; // ����ģʽ
    P13KE = 1; // ʹ�ܼ����ж�

    GIE = 1;
}

// �����
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
            // �����⵽ȷʵ�ڳ��
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
            // �����⵽���ڳ��
            flag_is_in_charging = 0;
        }
    }
}

void power_on_scan(void)
{
    // ����ʱ������Ƿ���˸��ӣ�
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
                // �����⵽�ǹػ���
                flag_is_device_open = 0;
            }
        }
    }
    else // �ػ�ʱ������Ƿ�����˸��ӣ�
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
                // �����⵽�ǿ�����
                flag_is_device_open = 1;
            }
        }
    }
}

/************************************************
;  *    @������            : main
;  *    @˵��              : ������
;  *    @�������          :
;  *    @���ز���          :
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
            // ����豸���Կ����������û�и��أ����û�и��� -> �ػ�
            // ��������س����� -> �ػ�
            
        }
        else
        {
            // ����豸���ܿ���������͹��ģ��ɼ����жϡ��ⲿ5V����������

        }
       

        __asm;
        clrwdt;
        __endasm;
    }
}
/************************************************
;  *    @������            : interrupt
;  *    @˵��              : �жϺ���
;  *    @�������          :
;  *    @���ز���          :
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
