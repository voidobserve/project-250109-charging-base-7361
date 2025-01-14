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
    T0DATA = 23 + 10;
    PWMCR0 = 0x00; // �������
    PWMCR1 = 0x18; // ʱ��ԴFHOSC �� 2  ��ͨģʽ
    // PWM0OPS = 0; // ѡ��P16�˿���� (���Բ�д��Ĭ�Ͼ���0)

    // T0EN = 1;
    T0EN = 0;   // �رն�ʱ��
    PWM0EC = 0; // ��ʹ��PWM0���
}

// ��ʱ��3
void timer3_config(void)
{
    T3LOAD = 135 - 1; // FCPU 64��Ƶ��������1ms����һ���жϣ��ü��������ֵ���������������һЩ������
    T3CR = 0x86;      // ʹ�ܶ�ʱ����ʱ��Դѡ��FCPU��64��Ƶ
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
    case ADC_CHANNEL_BAT: // ѡ�� 1/4 VDD ͨ��
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
        break;

    case ADC_CHANNEL_LOAD:
#if USE_MY_DEBUG
        // P11 AN6
        ADCHS3 = 0;
        ADCHS2 = 1;
        ADCHS1 = 1;
        ADCHS0 = 0;
#else
        // P15 AN9��������� �ŵ�
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 0;
        ADCHS0 = 1;
#endif
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

// ��ȡadc����ת��������ֵ
u16 adc_get_max_val(void)
{
    u8 i = 0; // adc�ɼ������ļ���
    u16 g_temp_value = 0;
    u16 g_adcmax = 0;

    // �ɼ�40��
    for (i = 0; i < 40; i++)
    {
        ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
        while (!ADEOC)
            ;                // �ȴ�ת�����
        g_temp_value = ADRH; // ȡ��ת�����ֵ
        g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);

        if (g_temp_value > g_adcmax)
            g_adcmax = g_temp_value; // ���
    }

    return g_adcmax;
}

// // ��ȡadc����ת�����ֵ
// u16 adc_get_val_once(void)
// {
//     u16 g_temp_value = 0;
//     ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
//     while (!ADEOC)
//         ;                // �ȴ�ת�����
//     g_temp_value = ADRH; // ȡ��ת�����ֵ
//     g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
//     return g_temp_value;
// }

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
    timer3_config();
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
    // P13PU = 0; // ����  --  ���Բ�����һ�䣬ͬ���ܼ�⵽
    P13OE = 0; // ����ģʽ
    // P13KE = 1; // ʹ�ܼ����ж�

    GIE = 1;
}

// �����
u8 is_in_charging(void)
{
    u8 ret = 0;

    // if (is_open_lid())
    // {
    //     return 0;
    // }

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
            // �����⵽���ڳ��
            ret = 0;
        }
    }

    return ret;
}

u8 is_open_lid(void)
{
    u8 ret = 0;

    if (P13D) // ������ɺ��ǿ�����
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
            ret = 1;
        }
    }
    else // ������ɺ��ǹرյ�
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
            ret = 0;
        }
    }

    return ret;
}

// �Ƿ��⵽����
u8 is_detect_load(void)
{
    // u8 ret = 0;
    u16 max_adc_val = 0;
    adc_sel_channel(ADC_CHANNEL_LOAD);
    for (i = 0; i < 100; i++)
    {
        adc_val = adc_get_max_val();
        if (adc_val > max_adc_val)
        {
            max_adc_val = adc_val;
        }
    }

    if (max_adc_val > ADC_VAL_LOAD_THRESHOLD)
    {
        // û�и���
        return 0;
    }
    else
    {
        // �и���
        return 1;
    }
}

// �رո������ĳ�磺
void stop_charging_the_host(void)
{
    LED_GREEN_OFF();
    PWM_DISABEL(); // �رտ��Ƴ���pwm
}

void low_power_scan_handle(void)
{
    if (CUR_STATUS_NONE == cur_dev_status)
    {
    low_power_label:
        // ����͹���
        GIE = 0; // ���������ж�
        LED_GREEN_OFF();
        LED_RED_OFF();
        PWM_DISABEL();
        // LED��������Ϊ����:
        P00OE = 0;
        P17OE = 0;
        // PWM��������͵�ƽ:
        P16D = 0;
        // �رն�ʱ�� 3
        T3EN = 0;
        T3IE = 0;
// �ر�adc�����ad����������Ϊ����:
#if USE_MY_DEBUG
        // ��������ŵ���������ţ�
        P11OE = 0; // ����ģʽ
        P11DC = 0; // ����ģʽ
#else
        // ��������ŵ���������ţ�
        P15OE = 0; // ����ģʽ
        P15DC = 0; // ����ģʽ
#endif
        ADEN = 0;
        // ���������źͼ���������������Ϊ�����жϴ���
        P13KE = 1;
// �������������Ϊ�����жϴ���
#if USE_MY_DEBUG
        P03KE = 1;
#else
        P04KE = 1;
#endif
        KBIF = 0;
        KBIE = 1;
        HFEN = 0; // �رո���ʱ��
        LFEN = 1;
        // ����ǰע��رղ��õ�����
        Nop();
        Nop();
        Stop();
        Nop();
        Nop();
        HFEN = 1; // ��������ʱ��
#if USE_MY_DEBUG
        P03KE = 0;
#else
        P04KE = 0;
#endif
        P13KE = 0; // �رռ����ж�
        KBIE = 0;
        KBIF = 0;

        // ����ص�ѹ�Ƿ�С��3.0V��δ���г��
        if (CHARGE_PIN)
        {
            // �����⵽��磬ʲôҲ����
        }
        else
        {
            // ���û�м�⵽��磬����ص����Ƿ����3.0V����������
            // adc_config();
            // adc_sel_channel(ADC_CHANNEL_BAT);
            // adc_val = adc_get_val();
            // if (adc_val < 1485) // �����⵽��ص�ѹС��2.9V
            // {
            //     // goto low_power_label;  // ����ᵼ���޷��������ĵ�ѹ������
            // }
        }

        Sys_Init();
        delay_ms(1);
        GIE = 1;
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

    // flag_is_low_bat = 1; // ����ʱʹ��
    PWM_ENABEL(); // ����ʱʹ��

    while (1)
    {
#if 1 // ���������߳��ʱ���������⵽��adֵ
        adc_sel_channel(ADC_CHANNEL_LOAD);
        for (i = 0; i < 100; i++)
        {
            adc_val = adc_get_max_val();
            if (adc_val > tmp_val)
            {
                tmp_val = adc_val;
                send_data_msb(tmp_val);
            }
        }

#endif

#if 0
        if (CUR_STATUS_NONE == cur_dev_status)
        {
            // �����ǰ�豸û���κβ���������Ƿ���
            // �Ƿ���ϸ��ӣ�
            // �Ƿ��ڱ���� ���ⲿ5V����
            ret_u8 = is_open_lid();
            if (0 == ret_u8)
            {
                // ����豸���Կ����������û�и��أ����û�и��� -> �ػ�
                // ��������س����� -> �ػ�

                // �򿪳��
                PWM_ENABEL();
                // LED_GREEN_ON(); // ���ܷ�������ᵼ�µ�һֱ��
                delay_ms(1); // �ȴ���ƽ�ȶ�
                ret_u8 = is_detect_load();
                if (ret_u8)
                {
                    // �и���
                    LED_GREEN_ON();
                    cur_dev_status = CUR_STATUS_IN_CHARGING;
                }
                else
                {
                    // û�и���
                    // LED_GREEN_OFF();
                    // PWM_DISABEL(); // �رտ��Ƴ���pwm
                    stop_charging_the_host();
                }
            }

            ret_u8 = is_in_charging();
            if (ret_u8)
            {
                // �����������ڱ���磬�رո������ĳ��
                // LED_GREEN_OFF();
                // PWM_DISABEL(); // �رտ��Ƴ���pwm
                stop_charging_the_host();

                LED_RED_ON();
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
            }

            // ���ִ�е����ﻹ��û�иı�״̬������͹���
            low_power_scan_handle();
        }
        else if (CUR_STATUS_IN_CHARGING == cur_dev_status)
        {
            // ������ڸ�������磬��⣺
            // �Ƿ�����˳����
            // �Ƿ���˸���
            // ����Ƿ�͵���

            ret_u8 = is_detect_load();
            if (ret_u8)
            {
                // �и���
                // LED_GREEN_ON(); // ��������������̵�
            }
            else
            {
                // û�и���
                // LED_GREEN_OFF();
                // PWM_DISABEL(); // �رտ��Ƴ���pwm
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_NONE;
            }

            ret_u8 = is_open_lid();
            if (ret_u8)
            {
                // ����������ɺУ��Ͽ���磺
                // LED_GREEN_OFF();
                // PWM_DISABEL(); // �رտ��Ƴ���pwm
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_NONE;
            }

            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            if (adc_val < 1485)
            {
                // �����⵽��ص����С��2.9V
                // �Ͽ���磺
                // LED_GREEN_OFF();
                // PWM_DISABEL(); // �رտ��Ƴ���pwm
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_NONE;
            }
            else if (adc_val < 1638) // ���������ĵ�ص���С��3.2V
            {
                //
                flag_is_low_bat = 1;
                LED_GREEN_OFF();
            }
            // else
            // {
            //     flag_is_low_bat = 0;
            // }

            ret_u8 = is_in_charging();
            if (ret_u8)
            {
                // �����������ڱ���磬�رո������ĳ�磬��ֹ��������
                LED_GREEN_OFF();
                // PWM_DISABEL(); // �رտ��Ƴ���pwm
                // LED_RED_ON();
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
            }
        }
        else if (CUR_STATUS_BE_CHARGING == cur_dev_status)
        {
            // ������ڱ���磬����Ƿ������
            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            if (adc_val >= 2125) // �����ص�ѹ����4.15V
            {
                LED_RED_OFF();

                // �ȵ��γ��˳��������ִ����������
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

            ret_u8 = is_in_charging();
            if (0 == ret_u8)
            {
                LED_RED_OFF();
                cur_dev_status = CUR_STATUS_NONE;
            }
        }
#endif
    } // while (1)

    __asm;
    clrwdt;
    __endasm;
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

    if (T3IF & T3IE)
    {
        // Ŀǰÿ1ms����һ���ж�
        static u16 blink_cnt = 0;
        if (CUR_STATUS_IN_CHARGING == cur_dev_status && flag_is_low_bat)
        {
            // ����ڳ�������������ʱ����⵽������ĵ�ش��ڵ͵���
            if (blink_cnt < 65535)
                blink_cnt++;

            if (blink_cnt <= 500)
            {
                LED_RED_ON();
            }
            else if (blink_cnt <= 1000)
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
            LED_RED_OFF();
            blink_cnt = 0;
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
