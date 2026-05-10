/*******************************************************

            IN THE NAME OF ALLAH

********************************************************

Project : CoolerController_Version2
Version :
Date    : 12/6/2022
Author  :
Company :
Comments:


Chip type               : ATmega64A
Program type            : Application
AVR Core Clock frequency: 16.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024
*******************************************************/
// INCLUDE SECTION:
#include <mega64a.h>
#include <delay.h>
#include <i2c.h>
#include <ds1307.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// DEFINES & MACROS SECTION:
#define BV(bit) (1 << (bit))
#define sbi(reg, bit) reg |= (BV(bit))
#define cbi(reg, bit) reg &= ~(BV(bit))

#define PumpSet sbi(PORTE, 3)
#define PumpReset cbi(PORTE, 3)
#define HighSet sbi(PORTE, 5)
#define HighReset cbi(PORTE, 5)
#define LowSet sbi(PORTE, 4)
#define LowReset cbi(PORTE, 4)
#define BuzSet cbi(PORTE, 6) // 6
#define BuzReset sbi(PORTE, 6)// 6

#define K1 !(PING & (1 << 2))
#define K2 !(PINC & (1 << 7))
#define K3 !(PINC & (1 << 6))
#define K4 !(PINC & (1 << 5))
#define K5 !(PINC & (1 << 4))
#define K6 !(PINC & (1 << 3))
#define K7 !(PINC & (1 << 2))

#define LED1S sbi(PORTE, 2)
#define LED1R cbi(PORTE, 2)
#define LED2S sbi(PORTA, 0)
#define LED2R cbi(PORTA, 0)
#define LED3S sbi(PORTA, 1)
#define LED3R cbi(PORTA, 1)
#define LED4S sbi(PORTA, 2)
#define LED4R cbi(PORTA, 2)
#define LED5S sbi(PORTA, 3)
#define LED5R cbi(PORTA, 3)
#define LED6S sbi(PORTA, 4)
#define LED6R cbi(PORTA, 4)
#define LED7S sbi(PORTA, 5)
#define LED7R cbi(PORTA, 5)

#define KeyBouncingTime 120
#define BuzerTime 50
#define MotorSwtich 100

// GLOBAL VARIABLES SECTION:
eeprom unsigned char set_point = 25;
char slider = 0,on_time = 0, off_time = 0, on_off_temp_time = 0, day_numer = 0, t_on_number = 1, time_hour = 0, time_minute = 0, str[10];
unsigned int timespan[4] = {0, 0, 0, 0};
bit transaction = 1, timer_loop_select = 0,on_off_select = 0, t_off_number = 1, auto_flicker = 0, flick_count = 0, timespan0init = 0, timespan1init = 0, timespan2init = 0, timespan3init = 0;

typedef struct
{
    unsigned char ton1_hour;
    unsigned char ton1_minute;
    unsigned char ton2_hour;
    unsigned char ton2_minute;
    unsigned char ton3_hour;
    unsigned char ton3_minute;
    unsigned char toff1_hour;
    unsigned char toff1_minute;
    unsigned char toff2_hour;
    unsigned char toff2_minute;
    unsigned char toff3_hour;
    unsigned char toff3_minute;
} daytime;

eeprom daytime day[7];


typedef struct
{
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    unsigned char week_day;
    unsigned char day;
    unsigned char month;
    unsigned char year;
} rtc_t;

rtc_t time =  {0, 0, 0, 0, 0, 0, 0};

enum State
{
    A, //
    B, //
    C, //
    D, //
    E, //  init pomp for some seconds
    F, //
    G,
    H,
    I,
    J0,
    JJ0,
    J1,
    K0,
    K11,
    L0,
    L1,
    N1,
    M1
};

enum State state = A, laststate = A;

// Buffering Transmision:
#define DATA_REGISTER_EMPTY (1 << UDRE0)
#define RX_COMPLETE (1 << RXC0)
#define FRAMING_ERROR (1 << FE0)
#define PARITY_ERROR (1 << UPE0)
#define DATA_OVERRUN (1 << DOR0)

// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 32
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0 <= 256
unsigned char rx_wr_index0 = 0, rx_rd_index0 = 0;
#else
unsigned int rx_wr_index0 = 0, rx_rd_index0 = 0;
#endif

#if RX_BUFFER_SIZE0 < 256
unsigned char rx_counter0 = 0;
#else
unsigned int rx_counter0 = 0;
#endif

// This flag is set on USART0 Receiver buffer overflow
bit rx_buffer_overflow0;

// USART0 Receiver interrupt service routine
interrupt[USART0_RXC] void usart0_rx_isr(void)
{
    char status, data;
    status = UCSR0A;
    data = UDR0;
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
    {
        rx_buffer0[rx_wr_index0++] = data;
#if RX_BUFFER_SIZE0 == 256
        // special case for receiver buffer size=256
        if (++rx_counter0 == 0)
            rx_buffer_overflow0 = 1;
#else
        if (rx_wr_index0 == RX_BUFFER_SIZE0)
            rx_wr_index0 = 0;
        if (++rx_counter0 == RX_BUFFER_SIZE0)
        {
            rx_counter0 = 0;
            rx_buffer_overflow0 = 1;
        }
#endif
    }
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used +
char getchar(void)
{
    char data;
    while (rx_counter0 == 0)
        ;
    data = rx_buffer0[rx_rd_index0++];
#if RX_BUFFER_SIZE0 != 256
    if (rx_rd_index0 == RX_BUFFER_SIZE0)
        rx_rd_index0 = 0;
#endif
#asm("cli")
    --rx_counter0;
#asm("sei")
    return data;
}
#pragma used -
#endif

// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 32
char tx_buffer0[TX_BUFFER_SIZE0];

#if TX_BUFFER_SIZE0 <= 256
unsigned char tx_wr_index0 = 0, tx_rd_index0 = 0;
#else
unsigned int tx_wr_index0 = 0, tx_rd_index0 = 0;
#endif

#if TX_BUFFER_SIZE0 < 256
unsigned char tx_counter0 = 0;
#else
unsigned int tx_counter0 = 0;
#endif

// USART0 Transmitter interrupt service routine
interrupt[USART0_TXC] void usart0_tx_isr(void)
{
    if (tx_counter0)
    {
        --tx_counter0;
        UDR0 = tx_buffer0[tx_rd_index0++];
#if TX_BUFFER_SIZE0 != 256
        if (tx_rd_index0 == TX_BUFFER_SIZE0)
            tx_rd_index0 = 0;
#endif
    }
}

#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART0 Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used +
void putchar(char c)
{
    while (tx_counter0 == TX_BUFFER_SIZE0)
        ;
#asm("cli")
    if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY) == 0))
    {
        tx_buffer0[tx_wr_index0++] = c;
#if TX_BUFFER_SIZE0 != 256
        if (tx_wr_index0 == TX_BUFFER_SIZE0)
            tx_wr_index0 = 0;
#endif
        ++tx_counter0;
    }
    else
        UDR0 = c;
#asm("sei")
}
#pragma used -
#endif



// Functions Section:
void Send(char *str);

// TIMERS ISR SECTION:
/// Timer 0 overflow interrupt service routine
interrupt[TIM0_OVF] void timer0_ovf_isr(void)
{
    // Place your code here
}

// Timer 0 output compare interrupt service routine
interrupt[TIM0_COMP] void timer0_comp_isr(void)
{
    // Place your code here
}

// Timer1 overflow interrupt service routine
interrupt[TIM1_OVF] void timer1_ovf_isr(void)
{
    // Place your code here
    if (timespan0init)
    {
        if (timespan[0] >= 20)
        {
            timespan0init = 0;
            timespan[0] = 0;
        }
        else
        {
            timespan[0]++;
        }
    }

    if (timespan1init)
    {
        if (timespan[1] > 7)
        {
            timespan1init = 0;
            timespan[1] = 0;

            BuzSet;
            delay_ms(2 * BuzerTime);
            BuzReset;

            transaction = 1;
            if (state == D || state == H || state == I || state == J0 || state == JJ0 || state == J1 || state == K11 || state == L1)
            {
                state = D;
            }
            else
            {
                state = A;
            }
        }
        else
        {
            timespan[1]++;
        }
    }

    if (timespan2init)
    {
        timespan[2]++;
    }

    if(timespan3init)
    {
        timespan[3]++;
        if(timespan[3] >= 2)
        {
            if(on_off_select)
            {
                on_time--;
            }
            else
            {
                off_time--;
            }
            timespan[3] = 0;
        }
    }

    if(auto_flicker)
    {
        if(flick_count++)
        {
            LED2R;
            flick_count = 0;
        }
        else
        {
            LED2S;
        }
    }
        
}

// Timer1 input capture interrupt service routine
interrupt[TIM1_CAPT] void timer1_capt_isr(void)
{
    // Place your code here
}

// Timer1 output compare A interrupt service routine
interrupt[TIM1_COMPA] void timer1_compa_isr(void)
{
    // Place your code here
}

// Timer1 output compare B interrupt service routine
interrupt[TIM1_COMPB] void timer1_compb_isr(void)
{
    // Place your code here
}

// Timer1 output compare C interrupt service routine
interrupt[TIM1_COMPC] void timer1_compc_isr(void)
{
    // Place your code here
}

// Timer2 overflow interrupt service routine
interrupt[TIM2_OVF] void timer2_ovf_isr(void)
{
    // Place your code here
}

// Timer2 output compare interrupt service routine
interrupt[TIM2_COMP] void timer2_comp_isr(void)
{
    // Place your code here
}

// Timer2 input capture interrupt service routine
// interrupt [TIM2_CAPT] void timer2_capt_isr(void)
//{
// Place your code here

//}

// Timer3 overflow interrupt service routine
interrupt[TIM3_OVF] void timer3_ovf_isr(void)
{
    // Place your code here
    Send(str);
}

// Timer3 input capture interrupt service routine
interrupt[TIM3_CAPT] void timer3_capt_isr(void)
{
    // Place your code here
}

// Timer3 output compare A interrupt service routine
interrupt[TIM3_COMPA] void timer3_compa_isr(void)
{
    // Place your code here
}

// Timer3 output compare B interrupt service routine
interrupt[TIM3_COMPB] void timer3_compb_isr(void)
{
    // Place your code here
}

// Timer3 output compare C interrupt service routine
interrupt[TIM3_COMPC] void timer3_compc_isr(void)
{
    // Place your code here
}

// A/D CONVERSION SECTION:

unsigned int adc_data;
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0 << REFS1) | (1 << REFS0) | (0 << ADLAR))

// ADC interrupt service routine
interrupt[ADC_INT] void adc_isr(void)
{
    // Read the AD conversion result
    adc_data = ADCW;
}

// Read the AD conversion result
// with noise canceling
unsigned int read_temp() // read_adc(unsigned char adc_input)
{
// ADMUX=adc_input | ADC_VREF_TYPE;
//  Delay needed for the stabilization of the ADC input voltage
// delay_us(10);
#asm
    in r30, mcucr cbr r30, __sm_mask sbr r30, __se_bit | __sm_adc_noise_red out mcucr, r30 sleep cbr r30, __se_bit out mcucr, r30
#endasm
        return adc_data;
}

// Entry Point:
void main(void)
{
    // Declare your local variables here
    unsigned char temp = 0;
    unsigned char setp = set_point;
    char error_sp;
    bit low_mode, high_mode, on_select = 0, highIsSet = 0, hm_select = 0, enable = 0, openloop = 0;

    // Input/Output Ports initialization
    // Port A initialization
    // Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
    DDRA = (0 << DDA7) | (0 << DDA6) | (1 << DDA5) | (1 << DDA4) | (1 << DDA3) | (1 << DDA2) | (1 << DDA1) | (1 << DDA0);
    // State: Bit7=T Bit6=T Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
    PORTA = (0 << PORTA7) | (0 << PORTA6) | (0 << PORTA5) | (0 << PORTA4) | (0 << PORTA3) | (0 << PORTA2) | (0 << PORTA1) | (0 << PORTA0);

    // Port B initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRB = (0 << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) | (0 << DDB3) | (0 << DDB2) | (0 << DDB1) | (0 << DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB = (0 << PORTB7) | (0 << PORTB6) | (0 << PORTB5) | (0 << PORTB4) | (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

    // Port C initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRC = (0 << DDC7) | (0 << DDC6) | (0 << DDC5) | (0 << DDC4) | (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
    // State: Bit7=P Bit6=P Bit5=P Bit4=P Bit3=P Bit2=P Bit1=T Bit0=T
    PORTC = (1 << PORTC7) | (1 << PORTC6) | (1 << PORTC5) | (1 << PORTC4) | (1 << PORTC3) | (1 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

    // Port D initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD = (0 << DDD7) | (0 << DDD6) | (0 << DDD5) | (0 << DDD4) | (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTD = (0 << PORTD7) | (0 << PORTD6) | (0 << PORTD5) | (0 << PORTD4) | (0 << PORTD3) | (0 << PORTD2) | (0 << PORTD1) | (0 << PORTD0);

    // Port E initialization
    // Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In
    DDRE = (0 << DDE7) | (1 << DDE6) | (1 << DDE5) | (1 << DDE4) | (1 << DDE3) | (1 << DDE2) | (0 << DDE1) | (0 << DDE0);
    // State: Bit7=T Bit6=1 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T
    PORTE = (0 << PORTE7) | (1 << PORTE6) | (0 << PORTE5) | (0 << PORTE4) | (0 << PORTE3) | (0 << PORTE2) | (0 << PORTE1) | (0 << PORTE0);

    // Port F initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRF = (0 << DDF7) | (0 << DDF6) | (0 << DDF5) | (0 << DDF4) | (0 << DDF3) | (0 << DDF2) | (0 << DDF1) | (0 << DDF0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTF = (0 << PORTF7) | (0 << PORTF6) | (0 << PORTF5) | (0 << PORTF4) | (0 << PORTF3) | (0 << PORTF2) | (0 << PORTF1) | (0 << PORTF0);

    // Port G initialization
    // Function: Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRG = (0 << DDG4) | (0 << DDG3) | (0 << DDG2) | (0 << DDG1) | (0 << DDG0);
    // State: Bit4=T Bit3=T Bit2=P Bit1=T Bit0=T
    PORTG = (0 << PORTG4) | (0 << PORTG3) | (1 << PORTG2) | (0 << PORTG1) | (0 << PORTG0);

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    // Mode: Normal top=0xFF
    // OC0 output: Disconnected
    ASSR = 0 << AS0;
    TCCR0 = (0 << WGM00) | (0 << COM01) | (0 << COM00) | (0 << WGM01) | (0 << CS02) | (0 << CS01) | (0 << CS00);
    TCNT0 = 0x00;
    OCR0 = 0x7D;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 62.500 kHz
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // OC1C output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 1.0486 s
    // Timer1 Overflow Interrupt: On
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << COM1C1) | (0 << COM1C0) | (0 << WGM11) | (0 << WGM10);
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10);
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;
    OCR1CH = 0x00;
    OCR1CL = 0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer2 Stopped
    // Mode: Normal top=0xFF
    // OC2 output: Disconnected
    TCCR2 = (0 << WGM20) | (0 << COM21) | (0 << COM20) | (0 << WGM21) | (0 << CS22) | (0 << CS21) | (0 << CS20);
    TCNT2 = 0x00;
    OCR2 = 0x00;

    // Timer/Counter 3 initialization
    // Clock source: System Clock
    // Clock value: 250.000 kHz
    // Mode: Normal top=0xFFFF
    // OC3A output: Disconnected
    // OC3B output: Disconnected
    // OC3C output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0.26214 s
    // Timer3 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR3A = (0 << COM3A1) | (0 << COM3A0) | (0 << COM3B1) | (0 << COM3B0) | (0 << COM3C1) | (0 << COM3C0) | (0 << WGM31) | (0 << WGM30);
    TCCR3B = (0 << ICNC3) | (0 << ICES3) | (0 << WGM33) | (0 << WGM32) | (0 << CS32) | (1 << CS31) | (1 << CS30);
    TCNT3H = 0x00;
    TCNT3L = 0x00;
    ICR3H = 0x00;
    ICR3L = 0x00;
    OCR3AH = 0x00;
    OCR3AL = 0x00;
    OCR3BH = 0x00;
    OCR3BL = 0x00;
    OCR3CH = 0x00;
    OCR3CL = 0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK = (0 << OCIE2) | (0 << TOIE2) | (0 << TICIE1) | (0 << OCIE1A) | (0 << OCIE1B) | (1 << TOIE1) | (0 << OCIE0) | (0 << TOIE0);
    ETIMSK = (0 << TICIE3) | (0 << OCIE3A) | (0 << OCIE3B) | (1 << TOIE3) | (0 << OCIE3C) | (0 << OCIE1C);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: Off
    // INT3: Off
    // INT4: Off
    // INT5: Off
    // INT6: Off
    // INT7: Off
    EICRA = (0 << ISC31) | (0 << ISC30) | (0 << ISC21) | (0 << ISC20) | (0 << ISC11) | (0 << ISC10) | (0 << ISC01) | (0 << ISC00);
    EICRB = (0 << ISC71) | (0 << ISC70) | (0 << ISC61) | (0 << ISC60) | (0 << ISC51) | (0 << ISC50) | (0 << ISC41) | (0 << ISC40);
    EIMSK = (0 << INT7) | (0 << INT6) | (0 << INT5) | (0 << INT4) | (0 << INT3) | (0 << INT2) | (0 << INT1) | (0 << INT0);

    // USART0 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART0 Receiver: Off
    // USART0 Transmitter: On
    // USART0 Mode: Asynchronous
    // USART0 Baud Rate: 9600 (Double Speed Mode)
    UCSR0A = (0 << RXC0) | (0 << TXC0) | (0 << UDRE0) | (0 << FE0) | (0 << DOR0) | (0 << UPE0) | (1 << U2X0) | (0 << MPCM0);
    UCSR0B = (0 << RXCIE0) | (1 << TXCIE0) | (0 << UDRIE0) | (0 << RXEN0) | (1 << TXEN0) | (0 << UCSZ02) | (0 << RXB80) | (0 << TXB80);
    UCSR0C = (0 << UMSEL0) | (0 << UPM01) | (0 << UPM00) | (0 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0);
    UBRR0H = 0x00;
    UBRR0L = 0xCF;

    // USART1 initialization
    // USART1 disabled
    UCSR1B = (0 << RXCIE1) | (0 << TXCIE1) | (0 << UDRIE1) | (0 << RXEN1) | (0 << TXEN1) | (0 << UCSZ12) | (0 << RXB81) | (0 << TXB81);

    // Analog Comparator initialization
    // Analog Comparator: On
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    // Analog Comparator Input Capture by Timer/Counter 1: Off
    ACSR = (0 << ACD) | (0 << ACBG) | (0 << ACO) | (0 << ACI) | (0 << ACIE) | (0 << ACIC) | (0 << ACIS1) | (0 << ACIS0);

    // ADC initialization
    // ADC Clock frequency: 1000.000 kHz
    // ADC Voltage Reference: Int., cap. on AREF
    // Only the 8 most significant bits of
    // the AD conversion result are used
    ADMUX = ADC_VREF_TYPE;
    ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADFR) | (0 << ADIF) | (1 << ADIE) | (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
    SFIOR = (0 << ACME);

    // SPI initialization
    // SPI disabled
    SPCR = (0 << SPIE) | (0 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (0 << SPR0);

    // TWI initialization
    // TWI disabled
    TWCR = (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | (0 << TWEN) | (0 << TWIE);

    // Bit-Banged I2C Bus initialization
    // I2C Port: PORTA
    // I2C SDA bit: 7
    // I2C SCL bit: 6
    // Bit Rate: 100 kHz
    // Note: I2C settings are specified in the
    // Project|Configure|C Compiler|Libraries|I2C menu.
    i2c_init();

// DS1307 Real Time Clock initialization
// Square wave output on pin SQW/OUT: Off
// SQW/OUT pin state: 0
// rtc_init(0,0,0);
// rtc_set_time(15,7,0);
 //rtc_set_date(1,27,9,22);

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/2048k
#pragma optsize -
    WDTCR = (1 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
    WDTCR = (0 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize +
#endif

// Global enable interrupts
#asm("sei")

    while (1)
    {
        #asm("WDR")
        switch (state)
        {
        case A:
            if (transaction)
            {            
                transaction = 0;
                laststate = A;

                LED1S;
                LED2R;
                LED3R;
                LED4R;
                LED5R;
                LED6R;
                LED7R;

                while (K1 || K4);
                delay_ms(KeyBouncingTime);
            }

            
            temp = (unsigned char)(read_temp() * 0.488);
            rtc_get_time(&time.hour, &time.min, &time.sec);
            sprintf(str, "%c%c%c%c%c%c%c%c", (temp / 10), (temp % 10), (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x01, 0x0A);
            

            if (K1)
            {            
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                transaction = 1;
                state = B;
            }

            if (K4)
            {            
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                transaction = 1;
                state = D;
            }

            if (K7)
            {            
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                transaction = 1;
                state = E;
            }

            if (state != laststate)
            {
                 transaction = 1;
            }
            break;

        case B:
            if (transaction)
            {            
                transaction = 0;
                laststate = B;
            
                timespan1init = 1;
                timespan[1] = 0;

                LED7S; //Clock LED On

                rtc_get_time(&time.hour, &time.min, &time.sec);
                rtc_get_date(&time.week_day, &time.day, &time.month, &time.year);
                sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x01, 0x0A);

                while (K1);
                delay_ms(KeyBouncingTime);
            }

            
            if (K2)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                LED7R; //Clock LED Off
                LED6S; // Date LED on
        
                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x00, 0x0A);
                
                while (K2);
                delay_ms(KeyBouncingTime);
            }


            if (K3)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                LED6R; //Date LED off
                LED7S; //Clock LED on

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x01, 0x0A);

                while (K3);
                delay_ms(KeyBouncingTime);
            }


            if (K1)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                transaction = 1;
                state = C;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;
            }

            if (state != laststate)
            {
                transaction = 1;
            }
            break;

        case C:
            if (transaction)
            {            
                transaction = 0;
                laststate = C;

                timespan[1] = 0;
                slider = 0;

                rtc_get_time(&time.hour, &time.min, &time.sec);
                rtc_get_date(&time.week_day, &time.day, &time.month, &time.year);

                if (PORTA & (1 << 5))                                                                 
                    sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x60, 0x0A);
                else
                    sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, 14, 11, 35, time.week_day, 0xc0, 0x0A);    

                while (K1);
                delay_ms(KeyBouncingTime);
            }

            if (PORTA & (1 << 5))
            {                   
                if (K1)
                {            
                    delay_ms(KeyBouncingTime);
                    timespan[1] = 0;

                    BuzSet;
                    delay_ms(BuzerTime);
                    BuzReset;

                    slider++;
                    if (slider == 1)
                    { 
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x18, 0x0A); //3 4
                    }
                    else
                    {                    
                        slider = 0;

                        timespan1init = 0;
                        timespan[1] = 0;

                        rtc_set_time(time.hour, time.min, 0);

                        transaction = 1;
                        state = A;
                        break;
                    }
                    while (K1);
                    delay_ms(KeyBouncingTime);
                }

                if (K3)
                {            
                    delay_ms(KeyBouncingTime);
                    timespan[1] = 0;

                    BuzSet;
                    delay_ms(BuzerTime);
                    BuzReset;

                    if(slider == 0)
                    {                    
                        if(time.min >= 0 & time.min < 59)
                            time.min++;  
                        else if(time.min == 59)
                            time.min = 0;                                                                         
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x60, 0x0A);
                    }
                    else if (slider == 1)
                    {                    
                        if(time.hour >= 0 & time.hour < 24)
                            time.hour++;  
                        else if(time.hour == 24)
                            time.hour = 0;                                                                         
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x18, 0x0A);                        
                    }

                    while (K3);
                    delay_ms(KeyBouncingTime);             
                }

                if (K2)
                {                
                    delay_ms(KeyBouncingTime);
                    timespan[1] = 0;

                    BuzSet;
                    delay_ms(BuzerTime);
                    BuzReset;

                    if(slider == 0)
                    {                    
                        if(time.min > 0 & time.min <= 59)
                            time.min--; 
                        else if(time.min == 0)
                            time.min = 59;                                                                      
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x60, 0x0A);
                    }
                    else if(slider == 1)
                    {                    
                        if (time.hour > 0 & time.hour <= 24)
                            time.hour--; 
                        else if(time.hour == 0)
                            time.hour = 24;                                                                    
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 30, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x18, 0x0A);
                    }

                    while(K2);
                    delay_ms(KeyBouncingTime);
                } 
            }
            else
            {
                if (K1)
                {
                    delay_ms(KeyBouncingTime);
                    timespan[1] = 0;

                    BuzSet;
                    delay_ms(BuzerTime);
                    BuzReset;

                    slider++;
                    if (slider == 1)
                    {
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x60, 0x0A);
                    } 
                    else if (slider == 2)
                    {
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x18, 0x0A);
                    }
                    else if (slider == 3)
                    {
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.year / 10), (time.year % 10), (time.month / 10), (time.month % 10), 0x18, 0x0A);
                    }
                    else
                    {
                        slider = 0;

                        timespan1init = 0;
                        timespan[1] = 0;

                        rtc_set_date(time.week_day, time.day, time.month, time.year);

                        transaction = 1;
                        state = A;
                        break;
                    }
                    while (K1);
                    delay_ms(KeyBouncingTime);
                }

                if (K3)
                {
                    delay_ms(KeyBouncingTime);
                    timespan[1] = 0;

                    BuzSet;
                    delay_ms(BuzerTime);
                    BuzReset;

                    if (slider == 0)
                    {
                        if (time.week_day >= 1 & time.week_day < 7)
                            time.week_day++; 
                        else if(time.week_day == 7)
                            time.week_day = 1;    
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, 14, 11, 35, time.week_day, 0xc0, 0x0A);
                    }
                    else if (slider == 1)
                    {
                        if (time.day >= 1 & time.day < 31)
                            time.day++; 
                        else if(time.day == 31)
                            time.day = 1;                                                                     
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x60, 0x0A);
                    }
                    else if (slider == 2)
                    {
                        if (time.month >= 1 & time.month < 12)
                            time.month++;
                        else if(time.month == 12)
                            time.month = 1;
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x18, 0x0A);
                    }
                    else if (slider == 3)
                    {
                        if (time.year >= 22 & time.year < 99)
                            time.year++;
                        else if(time.year == 99)
                            time.year = 22;
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.year / 10), (time.year % 10), (time.month / 10), (time.month % 10), 0x18, 0x0A);
                    }            
                    while (K3);
                    delay_ms(KeyBouncingTime);
                }

                if (K2)
                {
                    delay_ms(KeyBouncingTime);
                    timespan[1] = 0;

                    BuzSet;
                    delay_ms(BuzerTime);
                    BuzReset;

                    if (slider == 0)
                    {
                        if (time.week_day > 1 & time.week_day <= 7)
                            time.week_day--; 
                        else if(time.week_day == 1)
                            time.week_day = 7;
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, 14, 11, 35, time.week_day, 0xc0, 0x0A);
                    }
                    else if (slider == 1)
                    {
                        if (time.day > 1 & time.day <= 31)
                            time.day--; 
                        else if(time.day == 1)
                            time.day = 31;                                                                         
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x60, 0x0A);
                    }
                    else if (slider == 2)
                    {
                        if (time.month > 1 & time.month <= 12)
                            time.month--; 
                        else if(time.month == 1)
                            time.month = 12;
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.month / 10), (time.month % 10), (time.day / 10), (time.day % 10), 0x18, 0x0A);
                    }
                    else if (slider == 3)
                    {
                        if (time.year > 22 & time.year <= 99)
                            time.year--;
                        else if(time.year == 22)
                            time.year = 99;
                        sprintf(str, "%c%c%c%c%c%c%c%c", 29, 14, (time.year / 10), (time.year % 10), (time.month / 10), (time.month % 10), 0x18, 0x0A);
                    }
                    while (K2);
                    delay_ms(KeyBouncingTime);
                }
            }
            if (state != laststate)
            {
                 transaction = 1;
            }               
            break;

        case D:
            if (transaction)
            {
                transaction = 0;
                laststate = D;

                auto_flicker = 0;

                timespan[1] = 0;
                timespan[2] = 0;

                slider = 0;
                
                LED1R; //Manual LED Off
                LED2S; // Auto LED on
                LED3R; //SetPoint Off

                while (K4 || K7);                    ;
                delay_ms(KeyBouncingTime);
            }

            temp = (unsigned char)(read_temp() * 0.488);
            rtc_get_time(&time.hour, &time.min, &time.sec);
            if (slider == 0)
                sprintf(str, "%c%c%c%c%c%c%c%c", (temp / 10), (temp % 10), (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x01, 0x0A);
            else
                sprintf(str, "%c%c%c%c%c%c%c%c", (setp / 10), (setp % 10), 29, 15, 30, 26, 0x86, 0x0A);

            
            if (K1)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                while (K1)
                {
                    delay_ms(KeyBouncingTime);
                    timespan1init = 0;
                    timespan2init = 1;
                    if (timespan[2] >= 4)
                    {
                        transaction = 1;
                        state = H;

                        BuzSet;
                        delay_ms(2 * BuzerTime);
                        BuzReset;
                    
                        break;
                    }
                }                
                timespan2init = 0;
                timespan[2] = 0;
                delay_ms(KeyBouncingTime);

                if(state == H)
                {
                    break;
                }

                if (slider++)
                {
                    slider = 0;
                    LED3R; // Set Point LED Off
                    timespan1init = 0;

                    BuzSet;
                    delay_ms(2 * BuzerTime);
                    BuzReset;

                    set_point = setp;
                    transaction = 1;
                    state = G;
                }
                else
                {
                    LED3S; // Set Point LED On
                    timespan1init = 1;
                }
            }

            if (K2 & slider != 0)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                if (setp > 18 & setp <= 28)
                    setp--;

                while (K2);                ;
                delay_ms(KeyBouncingTime);
            }   

            if (K3 & slider != 0)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                if (setp >= 18 & setp < 28)
                    setp++;

                while (K3);                    ;
                delay_ms(KeyBouncingTime);
            }

            if (K4 & slider == 0)
            {
                delay_ms(KeyBouncingTime);

                transaction = 1;
                state = A;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;
            }

            if (state != laststate)
                transaction = 1;
    
            break;  
        case E:
            if (transaction)
            {
                transaction = 0;
                laststate = E;

                timespan0init = 1;

                PumpSet;

                while (K7);                    
                delay_ms(KeyBouncingTime);
            }


            temp = (unsigned char)(read_temp() * 0.488);
            rtc_get_time(&time.hour, &time.min, &time.sec);
            sprintf(str, "%c%c%c%c%c%c%c%c", (temp / 10), (temp % 10), (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x01, 0x0A);


            if (timespan[0] >= 10)
            {
                LowSet;

                transaction = 1;
                state = F;
            }
            if (K7)
            {
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                PumpReset;
                LowReset;
                HighReset;

                transaction = 1;
                timespan0init = 0;
                state = A;
            }
            if (state != laststate)
                transaction = 1;
            break;
        case F:
            if (transaction)
            {
                transaction = 0;
                timespan0init = 0;
                laststate = F;
            }
            
            timespan[0] = 12;
            temp = (unsigned char)(read_temp() * 0.488);
            rtc_get_time(&time.hour, &time.min, &time.sec);
            sprintf(str, "%c%c%c%c%c%c%c%c", (temp / 10), (temp % 10), (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x01, 0x0A);

            if (K5)
            {
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                LowReset;
                delay_ms(MotorSwtich);
                HighSet;

                while (K5);                    ;
                delay_ms(KeyBouncingTime);
            }
            if (K6)
            {
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                LowSet;
                delay_ms(MotorSwtich);
                HighReset;

                while (K6);                    ;
                delay_ms(KeyBouncingTime);
            }
            if (K7)
            {
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                PumpReset;
                LowReset;
                HighReset;

                transaction = 1;
                timespan0init = 1;
                state = A;
            }
            if (state != laststate)
                transaction = 1; 
            break;
        case G:
            if (transaction)
            {
                transaction = 0;
                laststate = G;

                timespan0init = 1;
               // timespan[0] = 12;

                HighReset;
                LowReset;

                temp = (unsigned char)(read_temp() * 0.488);
                error_sp = temp - setp;
                if (error_sp > 0)
                {
                    PumpSet;
                    if (error_sp < 5)
                    {
                        low_mode = 1;
                        high_mode = 0;
                    }

                    else
                    {
                        high_mode = 1;
                        low_mode = 0;
                    }
                }
                else
                {
                    PumpReset;
                }

                while (K1);                    ;
                delay_ms(KeyBouncingTime);
            }

            if (K7)
            {
                delay_ms(KeyBouncingTime);

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                PumpReset;
                LowReset;
                HighReset;

                transaction = 1;
                state = D;
                break;
            }
            
            temp = (unsigned char)(read_temp() * 0.488);
            rtc_get_time(&time.hour, &time.min, &time.sec);
            sprintf(str, "%c%c%c%c%c%c%c%c", (temp / 10), (temp % 10), 29, 26, (setp / 10), (setp % 10), 0x60, 0x0A);
            error_sp = temp - setp;

            if(error_sp < 0)
            {
                timespan[0] = 0;
                PumpReset;
                HighReset;
                LowReset;
                break;
            }

            if (low_mode)
            {
                if (error_sp > 7)
                {
                    low_mode = 0;
                    high_mode = 1;
                }
            }
            else
            {
                if (error_sp < 3)
                {
                    high_mode = 0;
                    low_mode = 1;
                }
            }


            if(timespan[0] > 10 && error_sp > 0)
            {
                timespan[0] = 12;
                if (error_sp < 5 & low_mode)
                {
                    HighReset;
                    delay_ms(MotorSwtich);
                    LowSet;
                }
                else if (error_sp > 5 & high_mode)
                {
                    LowReset;
                    delay_ms(MotorSwtich);
                    HighSet;
                }
            }
          
           
            if (state != laststate)
                transaction = 1;
            break;

        case H:
            if (transaction)
            {
                transaction = 0;
                laststate = H;

                auto_flicker = 1;
                timespan1init = 1;
                timespan[1] = 0;

                sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 22, 25, 25, 26, 0xF8, 0x0A); // of-loop

                while (K1);                    
                delay_ms(KeyBouncingTime);
            }

            if (K2)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                timer_loop_select = 1;

                sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 30, 19, 23, 15, 0xF8, 0x0A); // of-time

                while (K2);                    
                delay_ms(KeyBouncingTime);
            }

            if (K3)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                timer_loop_select = 0;

                sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 22, 25, 25, 26, 0xF8, 0x0A); //of-loop

                while (K3);                    
                delay_ms(KeyBouncingTime);
            }
            if (K1)
            {
                delay_ms(KeyBouncingTime);
                timespan[1] = 0;

                BuzSet;
                delay_ms(BuzerTime);
                BuzReset;

                transaction = 1;
                state = I;
            }

            if (state != laststate)
                transaction = 1;
            break;
            
            case I:
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = I;

                        timespan[1] = 0;

                        if (timer_loop_select == 0)
                        {
                             sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 22, 25, 25, 26, 0x86, 0x0A);
                        }
                        else
                        {
                             sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 30, 19, 23, 15, 0x86, 0x0A);
                        } 

                        while(K1)
                            timespan[1] = 0;
                        delay_ms(KeyBouncingTime);                        
                    }
                    if (K2)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if (timer_loop_select == 0)
                        {
                             sprintf(str, "%c%c%c%c%c%c%c%c", 25, 24, 22, 25, 25, 26, 0x86, 0x0A);
                        }
                        else
                        {
                             sprintf(str, "%c%c%c%c%c%c%c%c", 25, 24, 30, 19, 23, 15, 0x86, 0x0A);
                        } 

                        on_select = 1;
                        while (K2);                    
                        delay_ms(KeyBouncingTime);
                    }

                    if (K3)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if (timer_loop_select == 0)
                        {
                             sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 22, 25, 25, 26, 0x86, 0x0A);
                        }
                        else
                        {
                             sprintf(str, "%c%c%c%c%c%c%c%c", 25, 16, 30, 19, 23, 15, 0x86, 0x0A);
                        } 

                        on_select = 0;
                        while (K3);                    
                        delay_ms(KeyBouncingTime);
                    }
                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        transaction = 1;
                        if(on_select)
                        {
                            if(timer_loop_select == 0)
                            {
                                state = J0;
                            }
                            else
                            {
                                state = J1;
                            }
                        }
                        else
                        {
                            timespan1init = 0;
                            timespan[1] = 0;
                            state = D;
                        }
                    }
                    
                    if (state != laststate)
                        transaction = 1;
                    break;

            case J0: 
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = J0;

                        timespan[1] = 0;

                        on_select = 0;
                        timer_loop_select = 0;

                        on_time = 0;
                        off_time = 0;
                        
                        
                        sprintf(str, "%c%c%c%c%c%c%c%c", on_time / 10, on_time % 10, 25, 24, 30, 23, 0x06, 0x0A);
                        
                        while(K1);
                        delay_ms(KeyBouncingTime);
                    }

                    if (K2)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(on_time > 0)
                            on_time--;
                        sprintf(str, "%c%c%c%c%c%c%c%c", on_time / 10, on_time % 10, 25, 24, 30, 23, 0x06, 0x0A);                            

                        while (K2)
                            timespan[1] = 0;                   
                        delay_ms(KeyBouncingTime);
                    }

                    if (K3)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(on_time < 24)
                            on_time++;
                        sprintf(str, "%c%c%c%c%c%c%c%c", on_time / 10, on_time % 10, 25, 24, 30, 23, 0x06, 0x0A);                            


                        while (K3)
                            timespan[1] = 0;               
                        delay_ms(KeyBouncingTime);
                    }
                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);
                            timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        transaction = 1; 
                        if(on_time != 0)                      
                            state = JJ0;                    
                        else 
                        {
                            timespan1init = 0;
                            timespan[1] = 0;
                            state = D;
                        }   
                    }   
                    
                    if (state != laststate)
                        transaction = 1;
                    break;

            case JJ0: 
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = JJ0;

                        timespan[1] = 0;

                        
                        sprintf(str, "%c%c%c%c%c%c%c%c", off_time / 10, off_time % 10, 25, 16, 30, 23, 0x06, 0x0A);
                        
                        while(K1);
                        delay_ms(KeyBouncingTime);
                    }

                    if (K2)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(off_time > 0)
                            off_time--;
                        sprintf(str, "%c%c%c%c%c%c%c%c", off_time / 10, off_time % 10, 25, 16, 30, 23, 0x06, 0x0A);                            

                        while (K2)
                            timespan[1] = 0;                   
                        delay_ms(KeyBouncingTime);
                    }

                    if (K3)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(off_time < 24)
                            off_time++;
                        sprintf(str, "%c%c%c%c%c%c%c%c", off_time / 10, off_time % 10, 25, 16, 30, 23, 0x06, 0x0A);                            


                        while (K3)
                            timespan[1] = 0;               
                        delay_ms(KeyBouncingTime);
                    }
                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);
                            timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(off_time == 0)                      
                            openloop = 1;
                        else
                            openloop = 0;

                        timespan1init = 0;
                        timespan[1] = 0;   
                        transaction = 1; 
                        state = K0;                           
                          
                    }   
                    
                    if (state != laststate)
                        transaction = 1;
                    break;


            case J1: 
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = J1;

                        timespan[1] = 0;

                        on_select = 0;
                        timer_loop_select = 0;

                        sprintf(str, "%c%c%c%c%c%c%c%c", 14, day_numer, 33, 15, 15, 21, 0x84, 0x0A);                            
                        
                        while(K1);
                        delay_ms(KeyBouncingTime);
                    }  

                    if (K2)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(day_numer > 0)
                            day_numer--;
                        else if (day_numer ==0)
                            day_numer = 6;   
                            sprintf(str, "%c%c%c%c%c%c%c%c", 14, day_numer, 33, 15, 15, 21, 0x84, 0x0A);                            

                        while (K2)
                            timespan[1] = 0;                   
                        delay_ms(KeyBouncingTime);
                    }

                    if (K3)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(day_numer < 6)
                            day_numer++;
                        else if(day_numer == 6)
                            day_numer = 0;
                            sprintf(str, "%c%c%c%c%c%c%c%c", 14, day_numer, 33, 15, 15, 21, 0x84, 0x0A);                            


                        while (K3)
                            timespan[1] = 0;                   
                        delay_ms(KeyBouncingTime);
                    }
                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        transaction = 1; 
                        state = K11;
                    }   
 
                    
                    if (state != laststate)
                        transaction = 1;
                    break; 

            case K0: 
                    if(transaction)
                    {
                        transaction = 0;

                        if(laststate == L0)
                            off_time = on_off_temp_time;
                        on_off_temp_time = on_time;
                        
                        laststate = K0;

                        on_off_select = 1;
                        timespan3init = 1;

                        //auto_flicker = 0;


                        PumpSet;
                        if(highIsSet)
                            HighSet;
                        else
                            LowSet;
                        
                        while(K1);
                        delay_ms(KeyBouncingTime);
                    } 
                                        
                    sprintf(str, "%c%c%c%c%c%c%c%c", on_time / 10, on_time % 10, 25, 24, 30, 23, 0x79, 0x0A);                            
                    
                    if (K7)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        PumpReset;
                        LowReset;
                        HighReset;

                        timespan3init = 0;
                        timespan[3] = 0;
                        on_time = 0;
                        off_time =0;
                        transaction = 1;
                        state = D;
                        break;
                    }

                    if(on_time == 0)
                    {

                        if(openloop)
                        {
                            timespan3init = 0;
                            timespan[3] = 0;
                            PumpReset;
                            LowReset;
                            HighReset;
                            break;
                        }
                        else
                        {
                            BuzSet;
                            delay_ms(2*BuzerTime);
                            BuzReset;
                            transaction = 1;
                            state = L0;
                            break;
                        }
                        
                    }
          
                    if (K5)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        LowReset;
                        delay_ms(MotorSwtich);
                        HighSet;

                        highIsSet = 1;

                        while (K5);                    ;
                        delay_ms(KeyBouncingTime);
                    }

                    if (K6)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        LowSet;
                        delay_ms(MotorSwtich);
                        HighReset;

                        highIsSet = 0;

                        while (K6);                    ;
                        delay_ms(KeyBouncingTime);
                    }

                    
                    if (state != laststate)
                        transaction = 1;
                    break; 

            case K11:
                    if(transaction)
                    {
                        transaction = 0;

                        timespan[1] = 0;

                        if(laststate == L1)  
                        {
                            if(t_off_number)
                            {
                                switch(t_on_number)
                                {
                                    case 1:
                                            sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff1_hour / 10 , day[day_numer].toff1_hour % 10, day[day_numer].toff1_minute / 10, day[day_numer].toff1_minute % 10, 0x7D, 0x0A);                           
                                            break;

                                    case 2:
                                            sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff2_hour / 10 , day[day_numer].toff2_hour % 10, day[day_numer].toff2_minute / 10, day[day_numer].toff2_minute % 10, 0x7D, 0x0A);                           
                                            break;

                                    case 3: 
                                            sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff3_hour / 10 , day[day_numer].toff3_hour % 10, day[day_numer].toff3_minute / 10, day[day_numer].toff3_minute % 10, 0x7D, 0x0A);                           
                                            break;
                                }
                                t_off_number = 0;
                            }
                            else
                            {
                                switch(t_on_number)
                                {
                                    case 1:
                                            sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton1_hour / 10 , day[day_numer].ton1_hour % 10, day[day_numer].ton1_minute / 10, day[day_numer].ton1_minute % 10, 0x7D, 0x0A);                           
                                            break;

                                    case 2:
                                            sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton2_hour / 10 , day[day_numer].ton2_hour % 10, day[day_numer].ton2_minute / 10, day[day_numer].ton2_minute % 10, 0x7D, 0x0A);                           
                                            break;

                                    case 3: 
                                            sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton3_hour / 10 , day[day_numer].ton3_hour % 10, day[day_numer].ton3_minute / 10, day[day_numer].ton3_minute % 10, 0x7D, 0x0A);                           
                                            break;
                                }
                                t_off_number = 1;
                            }
                        } 
                        else
                        {
                            switch(t_on_number)
                            {
                                case 1:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton1_hour / 10 , day[day_numer].ton1_hour % 10, day[day_numer].ton1_minute / 10, day[day_numer].ton1_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 2:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton2_hour / 10 , day[day_numer].ton2_hour % 10, day[day_numer].ton2_minute / 10, day[day_numer].ton2_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 3: 
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton3_hour / 10 , day[day_numer].ton3_hour % 10, day[day_numer].ton3_minute / 10, day[day_numer].ton3_minute % 10, 0x7D, 0x0A);                           
                                        break;
                            }
                        }

                        laststate = K11;

                        while(K1)
                            timespan[1] = 0;
                        delay_ms(KeyBouncingTime);


                    } 

                     if (K3)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        if(t_on_number >= 3 && t_off_number == 0)
                            break;

                        if(t_on_number < 3 && t_off_number == 0)
                            t_on_number++;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;


                        if(t_off_number)
                        {
                            switch(t_on_number)
                            {
                                case 1:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff1_hour / 10 , day[day_numer].toff1_hour % 10, day[day_numer].toff1_minute / 10, day[day_numer].toff1_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 2:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff2_hour / 10 , day[day_numer].toff2_hour % 10, day[day_numer].toff2_minute / 10, day[day_numer].toff2_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 3:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff3_hour / 10 , day[day_numer].toff3_hour % 10, day[day_numer].toff3_minute / 10, day[day_numer].toff3_minute % 10, 0x7D, 0x0A);                           
                                        break;
                            }
                            t_off_number = 0;
                        }
                        else
                        {
                            switch(t_on_number)
                            {
                                case 1:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton1_hour / 10 , day[day_numer].ton1_hour % 10, day[day_numer].ton1_minute / 10, day[day_numer].ton1_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 2:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton2_hour / 10 , day[day_numer].ton2_hour % 10, day[day_numer].ton2_minute / 10, day[day_numer].ton2_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 3:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton3_hour / 10 , day[day_numer].ton3_hour % 10, day[day_numer].ton3_minute / 10, day[day_numer].ton3_minute % 10, 0x7D, 0x0A);                           
                                        break;
                            }
                            t_off_number = 1;
                        }


                        while (K3)
                            timespan[1] = 0;                  
                        delay_ms(KeyBouncingTime);
                    }

                    if (K2)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        if(t_on_number <= 1 && t_off_number == 1)
                            break;
                        
                        if(t_on_number > 1 && t_off_number == 1)
                            t_on_number--;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;


                         if(t_off_number)
                        {
                            switch(t_on_number)
                            {
                                case 1:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff1_hour / 10 , day[day_numer].toff1_hour % 10, day[day_numer].toff1_minute / 10, day[day_numer].toff1_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 2:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff2_hour / 10 , day[day_numer].toff2_hour % 10, day[day_numer].toff2_minute / 10, day[day_numer].toff2_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 3:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, day[day_numer].toff3_hour / 10 , day[day_numer].toff3_hour % 10, day[day_numer].toff3_minute / 10, day[day_numer].toff3_minute % 10, 0x7D, 0x0A);                           
                                        break;
                            }
                            t_off_number = 0;
                        }
                        else
                        {
                            switch(t_on_number)
                            {
                                case 1:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton1_hour / 10 , day[day_numer].ton1_hour % 10, day[day_numer].ton1_minute / 10, day[day_numer].ton1_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 2:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton2_hour / 10 , day[day_numer].ton2_hour % 10, day[day_numer].ton2_minute / 10, day[day_numer].ton2_minute % 10, 0x7D, 0x0A);                           
                                        break;

                                case 3:
                                        sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, day[day_numer].ton3_hour / 10 , day[day_numer].ton3_hour % 10, day[day_numer].ton3_minute / 10, day[day_numer].ton3_minute % 10, 0x7D, 0x0A);                           
                                        break;
                            }
                            t_off_number = 1;
                        }

                        while (K2)
                            timespan[1] = 0;                 
                        delay_ms(KeyBouncingTime);
                    }
                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        transaction = 1; 
                        state = L1;
                    }   

                    if (state != laststate)
                        transaction = 1;
                    break;

            case L0: 
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = L0;
                        on_time = on_off_temp_time;
                        on_off_temp_time = off_time;
                        on_off_select = 0;
                        

                        PumpReset;
                        LowReset;
                        HighReset;
                        
                    } 
                                        
                    sprintf(str, "%c%c%c%c%c%c%c%c", off_time / 10, off_time % 10, 25, 16, 30, 23, 0x79, 0x0A);                            

                    if(off_time == 0)
                    {
                        BuzSet;
                        delay_ms(2*BuzerTime);
                        BuzReset;

                        transaction = 1;
                        state = K0;
                        break;
                    }
                    
                    if (K7)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        PumpReset;
                        LowReset;
                        HighReset;

                        timespan3init = 0;
                        timespan[3] = 0;
                        on_time = 0;
                        off_time = 0;
                        transaction = 1;
                        state = D;
                        break;
                    }
                    
                    if (state != laststate)
                        transaction = 1;
                    break;  

            case L1:
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = L1;

                        timespan[1] = 0;

                        hm_select = 0;

                        switch(t_on_number)
                        {
                            case 1:
                                    if(t_off_number)
                                    {
                                        sprintf(str, "%c%c%c%c%c%c%c%c",24, t_on_number, day[day_numer].ton1_hour / 10, day[day_numer].ton1_hour % 10, day[day_numer].ton1_minute / 10, day[day_numer].ton1_minute % 10, 0x60, 0x0A);                           
                                        time_hour = day[day_numer].ton1_hour;
                                        time_minute = day[day_numer].ton1_minute;
                                    }
                                    else 
                                    {
                                        sprintf(str, "%c%c%c%c%c%c%c%c",16, t_on_number, day[day_numer].toff1_hour / 10, day[day_numer].toff1_hour % 10, day[day_numer].toff1_minute / 10, day[day_numer].toff1_minute % 10, 0x60, 0x0A);                           
                                        time_hour = day[day_numer].toff1_hour;
                                        time_minute = day[day_numer].toff1_minute;
                                    }   
                                    break;

                            case 2:
                                    if(t_off_number)
                                    {
                                        sprintf(str, "%c%c%c%c%c%c%c%c",24, t_on_number, day[day_numer].ton2_hour / 10, day[day_numer].ton2_hour % 10, day[day_numer].ton2_minute / 10, day[day_numer].ton2_minute % 10, 0x60, 0x0A);                           
                                        time_hour = day[day_numer].ton2_hour;
                                        time_minute = day[day_numer].ton2_minute;
                                    }
                                    else 
                                    {
                                        sprintf(str, "%c%c%c%c%c%c%c%c",16, t_on_number, day[day_numer].toff2_hour / 10, day[day_numer].toff2_hour % 10, day[day_numer].toff2_minute / 10, day[day_numer].toff2_minute % 10, 0x60, 0x0A);                           
                                        time_hour = day[day_numer].toff2_hour;
                                        time_minute = day[day_numer].toff2_minute;
                                    }   
                                    break;

                            case 3:
                                    if(t_off_number)
                                    {
                                        sprintf(str, "%c%c%c%c%c%c%c%c",24, t_on_number, day[day_numer].ton3_hour / 10, day[day_numer].ton3_hour % 10, day[day_numer].ton3_minute / 10, day[day_numer].ton3_minute % 10, 0x60, 0x0A);                           
                                        time_hour = day[day_numer].ton3_hour;
                                        time_minute = day[day_numer].ton3_minute;
                                    }
                                    else 
                                    {
                                        sprintf(str, "%c%c%c%c%c%c%c%c",16, t_on_number, day[day_numer].toff3_hour / 10, day[day_numer].toff3_hour % 10, day[day_numer].toff3_minute / 10, day[day_numer].toff3_minute % 10, 0x60, 0x0A);                           
                                        time_hour = day[day_numer].toff3_hour;
                                        time_minute = day[day_numer].toff3_minute;
                                    }   
                                    break;
                        }                        

                        while(K1)
                            timespan[1] = 0;
                        delay_ms(KeyBouncingTime);
                    } 

                     if (K2)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(hm_select)
                        {
                            if(time_hour > 0)
                                time_hour--;
                            else if(time_hour == 0)
                                time_hour = 24;
                            if(t_off_number)
                                sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x18, 0x0A);                            
                            else    
                                sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x18, 0x0A);                            
                        
                        }
                        else
                        {
                            if(time_minute > 0)
                                time_minute--;
                            else if(time_minute == 0)
                                time_minute = 59;
                            if(t_off_number)
                                sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x60, 0x0A);                            
                            else    
                                sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x60, 0x0A);                            
                        }


                        while (K2)
                            timespan[1] = 0;                   
                        delay_ms(KeyBouncingTime);
                    }

                    if (K3)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        if(hm_select)
                        {
                            if(time_hour < 24)
                                time_hour++;
                            else if(time_hour == 24)
                                time_hour = 0;
                            if(t_off_number)
                                sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x18, 0x0A);                            
                            else    
                                sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x18, 0x0A);                            
                        
                        }
                        else
                        {
                            if(time_minute < 59)
                                time_minute++;
                            else if(time_minute == 59)
                                time_minute = 0;
                            if(t_off_number)
                                sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x60, 0x0A);                            
                            else    
                                sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x60, 0x0A);                            
                        }
                        
                        while (K3)
                            timespan[1] = 0;                   
                        delay_ms(KeyBouncingTime);
                    }

                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);
                        timespan[1] = 0;

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;



                        if(hm_select == 0)
                        {
                            hm_select = 1;
                            if(t_off_number)
                                sprintf(str, "%c%c%c%c%c%c%c%c", 24, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x18, 0x0A);                            
                            else    
                                sprintf(str, "%c%c%c%c%c%c%c%c", 16, t_on_number, time_hour / 10, time_hour % 10, time_minute / 10, time_minute % 10, 0x18, 0x0A);                            
                            while(K1)
                                timespan[1] = 0;
                            delay_ms(KeyBouncingTime);
                            break;
                        }
                        else
                        {

                            if(t_off_number)
                            {
                                timespan[1] = 0;
                                switch(t_on_number)
                                {
                                    case 1:
                                            day[day_numer].ton1_hour = time_hour;
                                            day[day_numer].ton1_minute = time_minute;
                                            break;

                                    case 2:
                                            day[day_numer].ton2_hour = time_hour;
                                            day[day_numer].ton2_minute = time_minute;
                                            break;

                                    case 3: 
                                            day[day_numer].ton3_hour = time_hour;
                                            day[day_numer].ton3_minute = time_minute;
                                            break;
                                }
                            }
                            else
                            {
                                timespan[1] = 0;
                                switch(t_on_number)
                                {
                                    case 1:
                                            day[day_numer].toff1_hour = time_hour;
                                            day[day_numer].toff1_minute = time_minute;
                                            break;

                                    case 2:
                                            day[day_numer].toff2_hour = time_hour;
                                            day[day_numer].toff2_minute = time_minute;
                                            break;

                                    case 3: 
                                            day[day_numer].toff3_hour = time_hour;
                                            day[day_numer].toff3_minute = time_minute;
                                            break;
                                }
                            }

                            transaction = 1;
                            if(t_on_number == 3 && t_off_number == 0)
                            {
                                t_on_number = 1;
                                t_off_number = 1;
                                //auto_flicker = 0;
                                timespan1init = 0;
                                timespan[1] = 0;
                                state = M1;
                            }
                            else
                            {
                                if(t_off_number == 0)
                                    t_on_number++;
                                timespan[1] = 0;
                                state = K11;
                            }
                        }
                    }   

                    if (state != laststate)
                        transaction = 1;
                    break; 

            case M1:
                    if(transaction)
                    {
                        transaction = 0;
                        laststate = M1;

                        //LED2S;
                        
                                                   
                        while(K1);
                        delay_ms(KeyBouncingTime);
                    }

                    rtc_get_time(&time.hour, &time.min, &time.sec);
                    rtc_get_date(&time.week_day, &time.day, &time.month, &time.year);
                    sprintf(str, "%c%c%c%c%c%c%c%c", 30, 23, (time.hour / 10), (time.hour % 10), (time.min / 10), (time.min % 10), 0x07, 0x0A);
                    switch(time.week_day)
                    {
                        case 1:
                                if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                } 
                                break;

                        case 2:
                               if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                }
                                break;

                        case 3: 
                               if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                }
                                break;

                        case 4: 
                                if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                }
                                break;

                        case 5: 
                               if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                }
                                break;

                        case 6: 
                                if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                }
                                break;

                        case 7: 
                              if((day[time.week_day - 1].ton1_hour == time.hour && day[time.week_day - 1].ton1_minute == time.min) || (day[time.week_day - 1].ton2_hour == time.hour && day[time.week_day - 1].ton2_minute == time.min) || (day[time.week_day - 1].ton3_hour == time.hour && day[time.week_day - 1].ton3_minute == time.min)) 
                                {
                                    PumpSet;
                                    if(highIsSet == 0)
                                    {
                                        HighReset;
                                        LowSet;
                                    }
                                    else
                                    {
                                        LowReset;
                                        HighSet;
                                    }
                                    enable = 1;
                                }
                        
                                if((day[time.week_day - 1].toff1_hour == time.hour && day[time.week_day - 1].toff1_minute == time.min) || (day[time.week_day - 1].toff2_hour == time.hour && day[time.week_day - 1].toff2_minute == time.min) || (day[time.week_day - 1].toff3_hour == time.hour && day[time.week_day - 1].toff3_minute == time.min)) 
                                {
                                    PumpReset;
                                    LowReset;
                                    HighReset;
                                    enable = 0;
                                }
                                break;
                    }

                     if (K6  && enable)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        HighReset;
                        delay_ms(MotorSwtich);
                        LowSet;

                        highIsSet = 0;
                       
                        while (K6);                    
                        delay_ms(KeyBouncingTime);
                    }

                     if (K5 && enable)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        LowReset;
                        delay_ms(MotorSwtich);
                        HighSet;

                        highIsSet = 1;

                        while (K5);                    
                        delay_ms(KeyBouncingTime);
                    }

                    if (K7)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        PumpReset;
                        LowReset;
                        HighReset;

                        transaction = 1; 
                        state = D;
                    }   

                    if (state != laststate)
                        transaction = 1;
                    break;

            case N1:
                    if(transaction)
                    {
                        laststate = N1;                    

                       
                        while(K1);
                        delay_ms(KeyBouncingTime);
                    }

                     if (K2)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                       
                        while (K2);                    
                        delay_ms(KeyBouncingTime);
                    }

                    if (K3)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;


                        while (K3);                    
                        delay_ms(KeyBouncingTime);
                    }
                    if (K1)
                    {
                        delay_ms(KeyBouncingTime);

                        BuzSet;
                        delay_ms(BuzerTime);
                        BuzReset;

                        

                    
                    }   

                    if (state != laststate)
                        transaction = 1;
                    break;               
              
        }
    }
}
//****************************************************************************************
// Sender
void Send(char *str)
{
    char i;
    for (i = 0; i < 8; i++)
        putchar(*(str)++);
}
