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

    // // ����ʱ��ʹ�� P05 AN4 ���� 1/4VDD ͨ��������adֵ����
    // P05OE = 0;
    // P05DC = 1; // ģ��ģʽ
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

void timer0_pwm_config(void)
{
    // T0CR = 0x49; // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
// T0CNT = 50 - 1;
#if 0
    // 250KHz��
    T0CR = 0x49; // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    // ��������װ��ֵӦ����128������ʱ����RC�񵴵����ģ���׼ȷ����Ҫ���ϲ���
    T0LOAD = (128 + 10) - 1; //
    // T0DATA = 23 + 10;
    T0DATA = 59 + 10;
    // T0DATA = 45 + 10;
#endif

#if 1
    T0CR = 0x49;  // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    T0LOAD = 118; // 271KHz
    // T0LOAD = 135; // 235.2KHz
    // T0LOAD = 143; // 222����ʱ��224KHz
    // T0LOAD = 144;  // 220-222KHz
    // T0LOAD = 146;  // 218KHz
    // T0LOAD = 154; // 206.8KHz
    // T0LOAD = 158; // 201.6KHz
    // T0LOAD = 94; // ��358 - 363 KHz
    T0DATA = 41;

    // T0CR = (0x01 << 6) | (0x01 << 3); // ʹ��PWM,FTMR,����Ƶ (Լÿ0.000000015625us����һ�Σ�ʵ�ʵĿ϶���������������Ҫ���ϲ���)
    // // T0LOAD = 188; // 363 ~ 369 KHz
    // T0LOAD = 190; // ƽ��ֵԼΪ360.7KHz
    // T0DATA = 28;
#endif

#if 0
    // 214-216KHz �� 10%
    T0CR = 0x49;  // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    T0LOAD = 148; //
    T0DATA = 15;
#endif

#if 0
    // 186KHz
    T0CR = 0x49; // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    T0LOAD = 184; //
    T0DATA = 74;
#endif

#if 0
    // 135KHz
    T0CR = 0x49; // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    T0LOAD = 255 - 1; //
    T0DATA = 127;
#endif

#if 0
    // 125KHz
    T0CR = 0x4A;  // ʹ��PWM,FTMR,4��Ƶ (Լÿ0.06250us����һ��)
    T0LOAD = 137; //
    T0DATA = 14;
#endif

    PWMCR0 = 0x00; // �������
    PWMCR1 = 0x18; // ʱ��ԴFHOSC �� 2  ��ͨģʽ
    // PWM0OPS = 0; // ѡ��P16�˿���� (���Բ�д��Ĭ�Ͼ���0)

    // T0EN = 1;
    T0EN = 0;   // �رն�ʱ��
    PWM0EC = 0; // ��ʹ��PWM0���
}

void set_timer0_pwm_when_detecting(void)
{
    T0EN = 0;     // �رն�ʱ��
    PWM0EC = 0;   // ��ʹ��PWM0���
    T0CR = 0x49;  // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    T0LOAD = 114; // Լ282KHz
    // T0DATA = 17;
    T0DATA = 35;
    T0EN = 1;   // ʹ�ܶ�ʱ��
    PWM0EC = 1; // ʹ��PWM0���
    delay_ms(1);
}

void set_timer0_pwm_when_charging(void)
{
    T0EN = 0;   // �رն�ʱ��
    PWM0EC = 0; // ��ʹ��PWM0���

    // T0CR = (0x01 << 6) | (0x01 << 3); // ʹ��PWM,FTMR,����Ƶ (Լÿ0.000000015625us����һ�Σ�ʵ�ʵĿ϶���������������Ҫ���ϲ���)
    // // T0LOAD = 190;                     // ƽ��ֵԼΪ360.7KHz(ʵ��ֻ��333K~334K)
    // T0LOAD = 177; //
    // T0DATA = 41;

    T0CR = 0x49; // ʹ��PWM,FTMR,2��Ƶ (Լÿ0.03125us����һ��)
    // T0LOAD = 95; // ʵ�ʲ����� 333K
    // T0DATA = 19;
    // T0LOAD = 96; //
    // T0DATA = 19;
    T0LOAD = 97; // ʵ�ʲ����� 326K
    T0DATA = 19;

    // T0LOAD = 103; // Լ310KHz
    // T0DATA = 26;
    // T0LOAD = 111; // Լ288KHz
    // T0DATA = 26;

    T0EN = 1;   // ʹ�ܶ�ʱ��
    PWM0EC = 1; // ʹ��PWM0���
    delay_ms(1);
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
    case ADC_CHANNEL_BAT:
#if USE_MY_DEBUG
        // // ����ʱ���� P05 AN4 ���� 1/4VDD ͨ��
        // ADCHS3 = 0;
        // ADCHS2 = 1;
        // ADCHS1 = 0;
        // ADCHS0 = 0;
        // ѡ�� 1/4 VDD ͨ��
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
#else
        // ѡ�� 1/4 VDD ͨ��
        ADCHS3 = 1;
        ADCHS2 = 0;
        ADCHS1 = 1;
        ADCHS0 = 0;
#endif
        ADCR1 = 0x80; // adcת��ʱ��ѡ�� FHIRC/32��ʹ�ÃȲ�2.0V�ο���ѹ
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
        ADCR1 = 0x83; // adcת��ʱ��ѡ�� FHIRC/32��ʹ��VDD��Ϊ�ο���ѹ
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
// u16 adc_get_max_val(void)
// {
//     u8 i = 0; // adc�ɼ������ļ���
//     volatile u16 g_temp_value = 0;
//     volatile u16 g_adcmax = 0;

//     // �ɼ�40��
//     for (i = 0; i < 40; i++)
//     {
//         ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
//         while (!ADEOC)
//             ;                // �ȴ�ת�����
//         g_temp_value = ADRH; // ȡ��ת�����ֵ
//         g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);

//         if (g_temp_value > g_adcmax)
//             g_adcmax = g_temp_value; // ���
//     }

//     return g_adcmax;
// }

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
    P13PU = 0; // ����  --  ����ɾ����һ�䣬���ܻᵼ�½���˯���ֻ���
    P13OE = 0; // ����ģʽ
    // P13KE = 1; // ʹ�ܼ����ж�

    GIE = 1;
}

#if 0  // �����
/**
 * @brief �����
 *
 * @return * u8
 *              0 -- δ�ڳ��
 *              1 -- ���ڳ��
 *              2 -- ���ڼ���Ƿ��г�磨��δ��Ӹ��
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

    last_status = ret;

    return ret;
}
#endif // �����

#if 0  // �Ƿ�򿪱�����
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

    // ���û�з����仯
    if (ret == last_status)
    {
        return ret;
    }

    if (P13D) // ������ɺ��ǿ�����
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
    else // ������ɺ��ǹرյ�
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
#endif // �Ƿ�򿪱�����

// �Ƿ��⵽����
// ����ֵ 0--û�и���  1--�и��� 2--���ڼ����
u8 is_detect_load(void)
{
    u8 ret = 2; // ��ʾ���ڼ����
    volatile u32 tmp_u32 = bat_adc_val;
    tmp_u32 = tmp_u32 * 2000 * 4 / 4096;
    volatile u32 cmp_val = (tmp_u32 - CHARGING_CURRENT_ADC_VAL) * 4096 / tmp_u32;

    if (0 == flag_is_enable_detect_load)
    {
        // ����Ǹս��븺�ؼ��
        // ��ʼ�������ͱ�־λ
        detect_load_cnt = 0;
        undetect_load_cnt = 0;
        flag_tim_set_scan_time_is_arrive = 0;
        flag_is_enable_detect_load = 1; // ʹ�ܸñ�־λ�����ö�ʱ����ʼ��ʱ
    }

    adc_sel_channel(ADC_CHANNEL_LOAD);
    adc_val = adc_get_val();

    if (
#if 0
        (flag_is_being_charged && 114 == T0LOAD && adc_val < DETECT_LOAD_ADC_VAL_WHEN_BE_CHARGING) || /* ��������ڱ���磬��ʹ��282KHz��PWM��⸺�� */
        // (flag_is_being_charged && 114 != T0LOAD && adc_val < 3725) ||                                 /* ��������ڱ���磬��ʹ��326KHz��PWM��⸺���Ƿ�Ͽ� */
        (flag_is_being_charged && 114 != T0LOAD && adc_val < cmp_val) ||                                 /* ��������ڱ���磬��ʹ��326KHz��PWM��⸺���Ƿ�Ͽ� */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val < 3882) /* �����δ����磬��ʹ�� 326KHz ��PWM��⸺���Ƿ�Ͽ� */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val < 3700) /* �����δ����磬��ʹ�� 326KHz ��PWM��⸺���Ƿ�Ͽ� */
        (0 == flag_is_being_charged && 114 == T0LOAD && adc_val < 3649) || /* �����δ����磬��ʹ�� 282KHz ��PWM��⸺���Ƿ�Ͽ� */
        (0 == flag_is_being_charged && 114 != T0LOAD && adc_val < cmp_val) /* �����δ����磬��ʹ�� 326KHz ��PWM��⸺���Ƿ�Ͽ� */
#endif

        adc_val < cmp_val)
    {
        detect_load_cnt++;
    }
    else if (

#if 0
        (flag_is_being_charged && 114 == T0LOAD && adc_val > UNDETECT_LOAD_ADC_VAL_WHEN_BE_CHARGING) || /* ��������ڱ���磬��ʹ��282KHz��PWM��⸺�� */
        // (flag_is_being_charged && 114 != T0LOAD && adc_val > 3910) ||                                   /* ��������ڱ���磬��ʹ��326KHz��PWM��⸺���Ƿ�Ͽ� */
        (flag_is_being_charged && 114 != T0LOAD && adc_val > cmp_val) ||                                   /* ��������ڱ���磬��ʹ��326KHz��PWM��⸺���Ƿ�Ͽ� */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val > 3983) /* �����δ����磬��ʹ�� 326KHz ��PWM��⸺���Ƿ�Ͽ� */
        // (cur_dev_status == CUR_STATUS_IN_CHARGING && adc_val > 3780) /* �����δ����磬��ʹ�� 326KHz ��PWM��⸺���Ƿ�Ͽ� */
        (0 == flag_is_being_charged && 114 == T0LOAD && adc_val > 3872) || /* �����δ����磬��ʹ�� 282KHz ��PWM��⸺���Ƿ�Ͽ� */
        (0 == flag_is_being_charged && 114 != T0LOAD && adc_val > cmp_val) /* �����δ����磬��ʹ�� 326KHz ��PWM��⸺���Ƿ�Ͽ� */
#endif

        adc_val > cmp_val)
    {
        undetect_load_cnt++;
    }

    if (flag_tim_set_scan_time_is_arrive)
    {
        flag_tim_set_scan_time_is_arrive = 0;
        flag_is_enable_detect_load = 0; // ����ñ�־λ���رն�Ӧ�Ķ�ʱ����ʱ����
        delay_ms(1);

        if (detect_load_cnt > undetect_load_cnt)
        {
            // ������뵽���˵�������и���
            if ((detect_load_cnt - undetect_load_cnt) > (undetect_load_cnt / 2))
            {
                // �����⵽���صļ��� - δ��⵽���صļ��� ���� δ��⵽���ؼ�����һ��
                ret = 1;
            }
        }
        else
        {
            // ������뵽���˵�����ܶϿ�/û�и���
            // if ((undetect_load_cnt - detect_load_cnt) > (detect_load_cnt / 2))
            // {
            //     // ���δ��⵽���صļ��� - ��⵽���صļ��� ���� ��⵽���ؼ�����һ��
            //     ret = 0;
            // }

            // ֱ����Ϊû�и��أ�
            ret = 0;
        }

        detect_load_cnt = 0;
        undetect_load_cnt = 0;

        // last_status = 0;
        // pulse_cnt = 0;
    }

    return ret;
}

// �رո������ĳ�磺
void stop_charging_the_host(void)
{
    LED_GREEN_OFF();
    PWM_DISABLE(); // �رտ��Ƴ���pwm
}

void low_power_scan_handle(void)
{
    if (CUR_STATUS_NONE == cur_dev_status ||    /* ���κ�״̬ */
        CUR_STATUS_POWER_OFF == cur_dev_status) /* �͵����ػ� */
    {
    low_power_label:
        // ����͹���
        GIE = 0; // ���������ж�
        LED_GREEN_OFF();
        LED_RED_OFF();
        PWM_DISABLE();
        // LED��������Ϊ����:
        // P00OE = 0;
        // P17OE = 0;
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

        // adcͨ��ѡ��һ������ 1/4 VDD��
        ADCHS3 = 0;
        ADCHS2 = 0;
        ADCHS1 = 0;
        ADCHS0 = 0;
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
        LVDEN = 0; // �Ƿ�ر�LVD����Ӱ���������
        HFEN = 0;  // �رո���ʱ��
        LFEN = 0;  // �رյ���ʱ��
        // ����ǰע��رղ��õ�����
        Nop();
        Nop();
        Stop();
        Nop();
        Nop();
        HFEN = 1; // ��������ʱ��
        LVDEN = 1;
#if USE_MY_DEBUG
        P03KE = 0;
#else
        P04KE = 0;
#endif
        P13KE = 0; // �رռ����ж�
        KBIE = 0;
        KBIF = 0;

        // �ж��Ƿ��ǻ������߳�绽�ѣ�������ǣ��ص��͹���
        if (CHARGE_PIN || 0 == P13D)
        {
            // �������������Ǹߵ�ƽ ���� ���������ǵ͵�ƽ(�ر������ɺ�)
            // �˳��͹���
        }
        else
        {
            // �ص��͹���
            goto low_power_label;
        }

        // DEBUG_PIN = ~DEBUG_PIN; // �����Ƿ��ܻ��ѵ�Ƭ��
        GIE = 1;
        Sys_Init();
        delay_ms(1);

        if (CHARGE_PIN)
        {
            flag_is_being_charged = 1;
            cur_dev_status = CUR_STATUS_BE_CHARGING; // ֱ�ӽ��뱻����״̬������⸺��
        }

        if (0 == P13D)
        {
            flag_is_open_lid = 0; // ��ʾ�����˱�����
        }
        else
        {
            flag_is_open_lid = 1; // ��ʾ���˱�����
        }
    }
}

/************************************************
;  *    @������            : main
;  *    @˵��              : ������
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
// volatile u32 timer3_cnt; // ����ʱʹ��(���Գ�����ܷ��⵽����)
void main(void)
{
    Sys_Init();
    delay_ms(10);

    LED_GREEN_ON();
    delay_ms(1000);
    LED_GREEN_OFF();

#if 0  // ����ʱʹ��
    // flag_is_low_bat = 1; // ����ʱʹ��
    // LED_GREEN_ON();
    PWM_ENABLE();
    T3EN = 0;

    // delay_ms(4000); // �ȴ���⸺�صĵ����ȶ�
    // T3EN = 1;       // ����ʱʹ��
#endif // ����ʱʹ��

    // set_timer0_pwm_when_charging(); // // ����ʱʹ��

    flag_is_open_lid = 1;                  // ��ʾ�����Ǵ�
    flag_is_being_charged = 0;             // ��ʾû���ڳ��
    cur_dev_status = CUR_STATUS_POWER_OFF; // �ϵ�֮���ػ�

    while (1)
    {
#if 1

        if (CUR_STATUS_NONE == cur_dev_status)
        {
            if (0 == flag_is_open_lid) // ���û�д򿪸���
            {
                // ����豸���Կ����������û�и��أ����û�� ���� -> �ػ�
                // ��������س����� -> �ػ�
                set_timer0_pwm_when_detecting();
                // �򿪳��
                PWM_ENABLE();
                LED_GREEN_ON(); //
                // delay_ms(4000); // �ȴ���ƽ�ȶ� -- ���ڼ���Լ����û�д򿪱�����
                {
                    u16 i = 0;
                    // for (i = 0; i < 4000; i++)
                    for (i = 0; i < 500; i++) // �ȴ�500ms
                    {
                        // if (is_open_lid()) // ������˱�����
                        if (flag_is_open_lid) // ������˱�����
                        {
                            stop_charging_the_host(); // �ر�pwm���ر��̵�
                            break;
                        }

                        delay_ms(1);
                    }
                }

                adc_sel_channel(ADC_CHANNEL_LOAD);
                flag_is_enable_detect_load = 0; // ����ñ�־λ��׼�������⸺��

                while (1) // ������� xx s���ж���û�и���
                {

#if 0 // �ڸô������ʹ�ö����ļ�⸺�صķ�ʽ��δʹ�ö�ʱ����������ʱ
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
                                // ������뵽���˵�������и���
                                if ((detect_load_cnt - undetect_load_cnt) > (undetect_load_cnt / 2))
                                {
                                    // �����⵽���صļ��� - δ��⵽���صļ��� ���� δ��⵽���ؼ�����һ��
                                    // �����⵽�и���:
                                    // detect_load_cnt = 0;
                                    // undetect_load_cnt = 0;
                                    cur_dev_status = CUR_STATUS_IN_CHARGING;
                                    set_timer0_pwm_when_charging();
                                    // break;
                                }
                            }
                            else
                            {
                                // ������뵽���˵�����ܶϿ�/û�и���
                                // if ((undetect_load_cnt - detect_load_cnt) > (detect_load_cnt / 2))
                                {
                                    // ���δ��⵽���صļ��� - ��⵽���صļ��� ���� ��⵽���ؼ�����һ��
                                    // �����⵽ û�и���
                                    // detect_load_cnt = 0;
                                    // undetect_load_cnt = 0;
                                    stop_charging_the_host(); // �ر�pwm���ر��̵�
                                    // break;
                                }
                            }

                            if (flag_is_being_charged) // �����⵽�˳����
                            {
                                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                            }

                            detect_load_cnt = 0;
                            undetect_load_cnt = 0;
                            break;
                        }

#endif // �ڸô������ʹ�ö����ļ�⸺�صķ�ʽ��δʹ�ö�ʱ����������ʱ

                    ret_u8 = is_detect_load();
                    if (0 == ret_u8) // ���δ��⵽����
                    {
                        stop_charging_the_host(); // �ر�pwm���ر��̵�
                        break;                    // �˳���⸺�ص�ѭ��
                    }
                    else if (1 == ret_u8) // �����⵽����
                    {
                        cur_dev_status = CUR_STATUS_IN_CHARGING;
                        set_timer0_pwm_when_charging();
                        break; // �˳���⸺�ص�ѭ��
                    }

                    if (flag_is_open_lid)
                    {
                        // ����������ɺУ��˳�
                        detect_load_cnt = 0;
                        undetect_load_cnt = 0;
                        // __detect_ms_cnt = 0;
                        // flag_is_enable_detect_load = 0; // ����ñ�־λ���رն�Ӧ�Ķ�ʱ����ʱ����
                        // delay_ms(1);
                        stop_charging_the_host(); // �ر�pwm���ر��̵�
                        break;                    // �˳���⸺�ص�ѭ��
                    }

                    // ������ֱ�ӽ��뱻����״̬�����޷��ر��̵ƣ���״̬����û����Ӧ�Ĵ���
                    // else if (flag_is_being_charged) // ����ڼ���Ƿ��и����ڼ� �����г��������
                    // {
                    //     cur_dev_status = CUR_STATUS_BE_CHARGING; //
                    //     break;                                   // �˳���⸺�ص�ѭ��
                    // }
                } // while (1) // ������� xx s���ж���û�и���
            }

            if (flag_is_being_charged) // �����⵽�˳����
            {
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                if (1 == LED_GREEN_PIN)                  // ����̵ƻ��ڵ�����˵�����и���
                {
                    flag_is_detect_load_when_charged = 1; // ��ʾ�ڳ��ʱ��⵽�˸���
                }
            }

            // loop_cnt++;
            // if ((cur_dev_status != CUR_STATUS_NONE) || /* ���״̬�б仯������ (CUR_STATUS_NONE == cur_dev_status) �Ĵ���� */
            //     loop_cnt >= 2000)                      /* ��������� xx s��Ҳ������ǰ�Ĵ���� */
            // {
            //     loop_cnt = 0;
            //     break;
            // }

            // delay_ms(1);
            // } // while (1)

            //             ret_u8 = is_in_charging(); // �ж��Ƿ����ⲿ��5V����
            //             if (ret_u8)
            //             {
            // #if 0  // �����ʱ�����������İ汾
            //        // �����������ڱ���磬�رո������ĳ��:
            //                 stop_charging_the_host();
            // #endif // �����ʱ�����������İ汾

            //                 cur_dev_status = CUR_STATUS_BE_CHARGING; //
            //                 delay_ms(1);
            //                 continue; //
            //             }

            // ���ִ�е����ﻹ��û�иı�״̬������͹���
            // �����ڲ����ж��ǲ��Ǵ���NONE״̬���Ž���͹��ģ�
            low_power_scan_handle();
        } // if (CUR_STATUS_NONE == cur_dev_status)
        else if (CUR_STATUS_IN_CHARGING == cur_dev_status)
        {
            // ������ڸ�������磬��⣺
            // �����Ƿ������
            // �Ƿ�����˳����
            // �Ƿ���˸���
            // ����Ƿ�͵���

            ret_u8 = is_detect_load();
            if (1 == ret_u8)
            {
                // �����⵽�и���
                // ʲô�����ô���
            }
            else if (0 == ret_u8)
            {
                // ���ȷʵû�м�⵽���� ˵���Ѿ�������������
                flag_is_fully_charged = 1;

                /* ���û�д򿪸���/�����������̵ƻ�һֱ���� */

                // ���ڸĳɳ����磬Ϩ���̵�
                stop_charging_the_host();              // �ر�PWM���ر��̵�
                flag_is_low_bat = 0;                   // �����������ʱ����ոñ�־λ
                cur_dev_status = CUR_STATUS_POWER_OFF; // ֱ�ӹػ�
                continue;
            }
            else if (2 == ret_u8)
            {
                // ������ڼ����û�и���
            }

            // ����Ƿ�������ɺ�
            if (flag_is_open_lid)
            {
                // ����������ɺУ��Ͽ���磺
                cur_dev_status = CUR_STATUS_NONE;

                flag_is_fully_charged = 0;
                flag_is_enable_detect_load = 0; // ����ñ�־λ���رն�Ӧ�Ķ�ʱ����ʱ����
                delay_ms(1);
                stop_charging_the_host(); // �ر�PWM���ر��̵�
                continue;                 // ������ǰѭ��
            }

            // �͵����͵͵����ػ���⣺
            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            bat_adc_val = adc_val;
            // if (adc_val < 1485 - AD_OFFSET) // 2.9V
            // if (adc_val < 1587 - AD_OFFSET) // 3.1V
            // if (adc_val < 1587) // 3.1V
            if (adc_val < 1536) // 3.0V
            {
                // �����⵽��ص����С�� xx V
                // �Ͽ���磺
                cur_dev_status = CUR_STATUS_POWER_OFF;
                flag_is_fully_charged = 0;
                flag_is_enable_detect_load = 0; // ����ñ�־λ���رն�Ӧ�Ķ�ʱ����ʱ����
                delay_ms(1);
                stop_charging_the_host();
                continue;
            }
            // else if (adc_val < 1638 - AD_OFFSET) // ���������ĵ�ص���С��3.2V
            // else if (adc_val < 1638) // ���������ĵ�ص���С��3.2V
            else if (adc_val < 1689) // ���������ĵ�ص���С��3.3V
            {
                //
                LED_GREEN_OFF(); // �ص��̵ƣ��ú������
                delay_ms(1);     // �ȴ���ʱ���������
                flag_is_low_bat = 1;
                // flag_is_fully_charged = 0;
            }
            // else if (adc_val > 1638 + AD_OFFSET) // ע����һ����Ҫ�� (������ĵ�ص���С��3.2V) ���������һ��adֵ����
            else if (adc_val > 1689 + AD_OFFSET) // ע����һ����Ҫ�� (������ĵ�ص���С��3.3V) ���������һ��adֵ����
            {
                flag_is_low_bat = 0;
                delay_ms(1); // �ȴ���ʱ���رպ��
                // �����С��3.2V�����µ����̵�
                // (��ֹ��ص�ѹ��3.2V����3.2���£�������3.2V��ָʾ�ƻ�ȫ��Ϩ�𣬲��������ɫ���ɫ��ָʾ�Ƶ����)
                // LED_GREEN_ON(); // ��������������������⵽�и��أ��̵Ƶ���˸�᲻����
            }

            // ����Ƿ�����˳����:
            // if (is_in_charging())
            if (flag_is_being_charged)
            {
#if 0  // �����ʱ�����������İ汾
       // �����������ڱ���磬�رո������ĳ�磬��ֹ��������
                stop_charging_the_host();
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                flag_is_low_bat = 0;                     // ����������ʱ������ñ�־λ
                flag_is_enable_detect_load = 0;          // ����ñ�־λ���رն�Ӧ�Ķ�ʱ����ʱ����
                delay_ms(1);
                LED_GREEN_OFF(); // �ȴ�״̬�ͱ�־λ������ɣ��ٹر��̵�
                flag_is_fully_charged = 0;
                LED_RED_ON();
#endif // �����ʱ�����������İ汾

#if 1
                // �����������ڱ���磬���رո������ĳ��
                cur_dev_status = CUR_STATUS_BE_CHARGING; //
                flag_is_low_bat = 0;                     // ����������ʱ������ñ�־λ
                flag_is_enable_detect_load = 0;          // ����ñ�־λ���رն�Ӧ�Ķ�ʱ����ʱ����
                flag_is_detect_load_when_charged = 1;    // ���ڸ�������磬���Ҽ�⵽�����˳��������ô�͸������־λ��һ
                delay_ms(1);
                continue;
#endif
            }
        } // else if (CUR_STATUS_IN_CHARGING == cur_dev_status) // ��ǰ���ڸ��������
        else if (CUR_STATUS_BE_CHARGING == cur_dev_status) // ��ǰ���ڱ����
        {
            // ״̬�����Ƿ��⵽���أ�ֻ�ڵ�ǰ����ʹ��
            // Ĭ��ֵֻ����0��������ram���������б�����ֵ����0
            // 0--��ʾ�ڳ����������ڼ䣬��û�п�ʼ����Ƿ��и���
            // 1--��ʾ���ڼ����
            // 2--��⵽û�и���
            // 3--��⵽�и���
            static u8 __flag_detect_load_status = 0;

            // ��ʾ���ӵ�״̬
            // 0 -- ��ʼֵ
            // 1 -- �����˸���
            // 2 -- δ���ϸ���
            static u8 __lid_status;
            if (0 == __lid_status) // ����Ǹս�����
            {
                if (P13D) // ����ս����磬���Ӿ��ǿ�����
                {
                    __lid_status = 2;
                }
                else
                {
                    __lid_status = 1; // �ս����磬���Ӿ��ǹرյ�
                }
            }

            if (0 == __flag_detect_load_status && flag_is_detect_load_when_charged)
            {
                // ��� __flag_detect_load_status ΪĬ��ֵ�� flag_is_detect_load_when_charged == 1��
                // ˵���ⲿ�Ƚ�flag_is_detect_load_when_charged��һ��֮��Ž���ô����
                // ��ô�� __flag_detect_load_status ��ֵ3����ʾ�ڱ�����ڼ��⵽�и��أ�������һֱ��⸺����û�жϿ�
                __flag_detect_load_status = 3;
            }

            // ������ڱ���磬����Ƿ������
            adc_sel_channel(ADC_CHANNEL_BAT);
            adc_val = adc_get_val();
            bat_adc_val = adc_val;
            // if (adc_val >= 2124 - AD_OFFSET) // �����ص�ѹ����4.15V (ʵ�ʲ�����4.17V������Ϊ������)
            // if (adc_val >= 2150) // �����ص�ѹ����4.20V(ʵ�ʲ��ԣ��ڳ��ʱ��������˵�ѹ��4V����Ϊ������)
            // if (adc_val >= 2150 + AD_OFFSET) // ʵ�ʲ��Կ��ܴ���4.18V
            // if (adc_val >= 2150 + 20) //
            if (adc_val >= 2048) // ���������4V
            {
                flag_bat_is_fully_charged = 1; // ��ʾ������ĵ�ر�����
            }

            // ����Ƿ�Ͽ��˳����:
            // ret_u8 = is_in_charging();
            // if (0 == ret_u8)
            if (0 == flag_is_being_charged)
            {
                // flag_bat_is_fully_charged = 0;
                // cur_dev_status = CUR_STATUS_NONE;

                if (flag_is_detect_load_when_charged) // ������ڸ��������
                {
                    cur_dev_status = CUR_STATUS_IN_CHARGING;
                }
                else // ���û�и�������磬ֱ�ӹػ�
                {
                    cur_dev_status = CUR_STATUS_POWER_OFF;
                }

                // �˳��ô����ǰ��������������ʹ�õı�����
                flag_is_detect_load_when_charged = 0;
                __lid_status = 0; // ���㣬�ص�Ĭ��ֵ
                __flag_detect_load_status = 0;
                continue;
            }

#if 1
            if (flag_is_open_lid && __lid_status == 1)
            {
                // �����⵽���˸���
                __lid_status = 2; // ��ʾ���˸���

                __flag_detect_load_status = 0;
                flag_is_enable_detect_load = 0; // ��ʹ�ܼ�⸺�صĶ�ʱ����ʱ
                delay_ms(1);
                flag_is_detect_load_when_charged = 0;
                stop_charging_the_host(); // �ر�PWM
            }
            else if (((1 == __flag_detect_load_status) ||         /* ������ڼ����û�и��� */
                      (3 == __flag_detect_load_status)) ||        /* �����⵽����û�жϿ���������� */
                     (__lid_status == 2 && 0 == flag_is_open_lid) // ���֮ǰ�����ǿ����ģ����ڼ�⵽���Ӹ���
            )
            {
                __lid_status = 1; // ��ʾ���ϸ���

                // ��������ǹرյ�
                if (0 == __flag_detect_load_status)
                {
                    // �����δ�����Ƿ��и���
                    __flag_detect_load_status = 1; // ��ʾ���ڼ���Ƿ��и���
                    set_timer0_pwm_when_detecting();
                    PWM_ENABLE();
                    LED_GREEN_ON();
                    // delay_ms(4000); // -- ���ڼ���Լ����û�д򿪱�����
                    {
                        u16 i = 0;
                        // for (i = 0; i < 4000; i++)
                        for (i = 0; i < 500; i++)
                        {
                            if (flag_is_open_lid) // ������˱�����
                            {
                                __flag_detect_load_status = 0;
                                flag_is_detect_load_when_charged = 0;
                                stop_charging_the_host(); // �ر�pwm���ر��̵�
                                break;
                            }

                            delay_ms(1);
                        }
                    }
                }

                if ((1 == __flag_detect_load_status) || /* ������ڼ����û�и��� */
                    (3 == __flag_detect_load_status))   /* �����⵽����û�жϿ���������� */
                {
                    ret_u8 = is_detect_load();
                    if (0 == ret_u8)
                    {
                        // ����Ͽ��˸���
                        __flag_detect_load_status = 2;
                        flag_is_detect_load_when_charged = 0;
                        stop_charging_the_host(); // �ر�PWM
                    }
                    else if (1 == ret_u8)
                    {
                        // �����δ�Ͽ����أ������ǴӼ�⸺�ص���Ϊ��⵽�и���
                        if (1 == __flag_detect_load_status) // ����մ�200��K��PWM���������Ҫ�л���300��K��PWM
                        {
                            set_timer0_pwm_when_charging();
                        }

                        __flag_detect_load_status = 3;
                        flag_is_detect_load_when_charged = 1;
                    }
                    else if (2 == ret_u8)
                    {
                        // ������ڼ���Ƿ��и���
                    }
                }
            }
#endif
        }
        else if (CUR_STATUS_POWER_OFF == cur_dev_status)
        {
            // �͵����ػ�:
            low_power_scan_handle();

            if (cur_dev_status != CUR_STATUS_BE_CHARGING) // ������ǲ����˳�������ѵ�
            {
                cur_dev_status = CUR_STATUS_NONE; // �ӵ͵����ػ����ֻ��ѣ��ص�����״̬
            }
        }
#endif

        __asm;
        clrwdt;
        __endasm;
    } // while (1)
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
        // { // �ڼ�⸺�صĲ�����ʹ��
        //     timer3_cnt++;
        //     if (timer3_cnt >= 4000)
        //     {
        //         timer3_cnt = 0;
        //         flag_tim_set_scan_time_is_arrive = 1;
        //     }
        // }

        { // ��⸺�ص�ʱ�����
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

        { // ����Ƿ���˱�����
            static u8 detect_open_lid_cnt;
            static u8 undetect_open_lid_cnt;

            if (P13D) // ����������ǿ�����
            {
                undetect_open_lid_cnt = 0;
                detect_open_lid_cnt++;
                if (detect_open_lid_cnt >= 200) // ���һֱ��⵽�д򿪱�����
                {
                    detect_open_lid_cnt = 0;
                    flag_is_open_lid = 1; // ��ʾ���˱�����
                }
            }
            else // ����������ǹرյ�
            {
                detect_open_lid_cnt = 0;
                undetect_open_lid_cnt++;
                if (undetect_open_lid_cnt >= 200) // ���һֱ��⵽û�д򿪱�����
                {
                    undetect_open_lid_cnt = 0;
                    flag_is_open_lid = 0; // ��ʾû��
                }
            }
        }

#if 1
        { // ������ʱ�����

            static u8 detect_charge_cnt;   // ��û�г���ڼ䣬��⵽�г���ʱ�����
            static u8 undetect_charge_cnt; // �ڳ���ڼ䣬��⵽�Ͽ�����ʱ�����

            if (flag_is_being_charged) // ����ڳ�磬����Ƿ�Ͽ���磬�����м���
            {
                if (CHARGE_PIN) // �����⵽���
                {
                    undetect_charge_cnt = 0;
                }
                else
                {
                    detect_charge_cnt = 0;
                    undetect_charge_cnt++;
                    if (undetect_charge_cnt >= 200) // ������� xx ms����⵽
                    {
                        undetect_charge_cnt = 0;
                        flag_is_being_charged = 0; // ��ʾ���ڳ��
                    }
                }
            }
            else // ������ڳ�磬����Ƿ��г�磬�����м���
            {
                if (CHARGE_PIN) // �����⵽���
                {
                    undetect_charge_cnt = 0;
                    detect_charge_cnt++;
                    if (detect_charge_cnt >= 200) // ������� xx ms����⵽
                    {
                        detect_charge_cnt = 0;
                        flag_is_being_charged = 1; // ��ʾ���ڱ����
                    }
                }
                else // �����ⲻ�����
                {
                    detect_charge_cnt = 0;
                }
            }

        } // ������ʱ�����
#endif

#if 1
        { // ������ƺ�Ƶ���/Ϩ�����ش����
            static u16 blink_cnt = 0;
            static u16 fully_charged_cnt;                   // �������� xx ms�󣬹رպ�Ƶı���
            if (CUR_STATUS_IN_CHARGING == cur_dev_status && /* ��������������ʱ */
                flag_is_low_bat)                            /* �������ش��ڵ͵��� */
            {
                // ����ڳ�������������ʱ����⵽������ĵ�ش��ڵ͵���
                // LED_GREEN_OFF(); // �ر��̵ƣ��������Ƴ�ͻ

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
            // else if (CUR_STATUS_BE_CHARGING == cur_dev_status && /* ����������ʱ */
            //          0 == flag_bat_is_fully_charged)             /* �����δ�������� */
            else if (flag_is_being_charged &&        /* ����������ʱ */
                     0 == flag_bat_is_fully_charged) /* �����δ�������� */
            {
                // �����������ڱ���磬��Ƴ���
                // LED_GREEN_OFF(); // �ر��̵ƣ��������Ƴ�ͻ
                LED_RED_ON();

                fully_charged_cnt = 0;
            }
            else
            {
                fully_charged_cnt++;

                if (fully_charged_cnt >= 1000) // ���� xx ms���ٹرպ��
                {
                    fully_charged_cnt = 0;
                    LED_RED_OFF();
                }

                blink_cnt = 0;
            }
        }

#if 1     // ���Ƴ��ʱ�����̵���˸�Ĵ����
        { // ���Ƴ��ʱ�����̵���˸�Ĵ����

            static u16 led_green_blink_cnt = 0;
            // �����������ڸ�������磬���ҳ�����ĵ�ز����ڵ͵���
#if 1 // �ڱ����ʱ����Ȼʹ�ܸ��������İ汾
            if (flag_is_detect_load_when_charged ||
                (CUR_STATUS_IN_CHARGING == cur_dev_status &&
                 0 == flag_is_low_bat &&
                 0 == flag_is_fully_charged))
#endif // �ڱ����ʱ����Ȼʹ�ܸ��������İ汾
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

        } // ���Ƴ��ʱ�����̵���˸�Ĵ����
#endif // ���Ƴ��ʱ�����̵���˸�Ĵ����

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
