/*******************************************************

        IN THE NAME OF ALLAH

********************************************************

Project :
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

#include <mega64a.h>
#include <delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Global variables Section:

char str[10] = {23, 24, 36, 28, 18, 16, 7};
typedef struct
{
    unsigned char sg1;
    unsigned char sg2;
    unsigned char sg3;
    unsigned char sg4;
    unsigned char sg5;
    unsigned char sg6;
    unsigned char flickCode;

} segment_data;

segment_data segment = {0, 0, 0, 0, 0, 0, 0};

// Buffering Method for receiving data:
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

#define BV(bit) (1 << (bit))
#define sbi(reg, bit) reg |= (BV(bit))
#define cbi(reg, bit) reg &= ~(BV(bit))

// flicker frequency:
#define flc 300

// Segment1 Pin Config
#define A1set sbi(PORTF, 2)
#define A1rst cbi(PORTF, 2)
#define B1set sbi(PORTF, 3)
#define B1rst cbi(PORTF, 3)
#define C1set sbi(PORTA, 1)
#define C1rst cbi(PORTA, 1)
#define D1set sbi(PORTA, 0)
#define D1rst cbi(PORTA, 0)
#define E1set sbi(PORTF, 7)
#define E1rst cbi(PORTF, 7)
#define F1set sbi(PORTF, 0)
#define F1rst cbi(PORTF, 0)
#define G1set sbi(PORTF, 1)
#define G1rst cbi(PORTF, 1)

// Segment2 Pin Confing
#define A2set sbi(PORTF, 5)
#define A2rst cbi(PORTF, 5)
#define B2set sbi(PORTF, 6)
#define B2rst cbi(PORTF, 6)
#define C2set sbi(PORTA, 5)
#define C2rst cbi(PORTA, 5)
#define D2set sbi(PORTA, 3)
#define D2rst cbi(PORTA, 3)
#define E2set sbi(PORTA, 2)
#define E2rst cbi(PORTA, 2)
#define F2set sbi(PORTF, 4)
#define F2rst cbi(PORTF, 4)
#define G2set sbi(PORTA, 4)
#define G2rst cbi(PORTA, 4)

// Segment3 Pin Config
#define A3set sbi(PORTD, 4)
#define A3rst cbi(PORTD, 4)
#define B3set sbi(PORTD, 3)
#define B3rst cbi(PORTD, 3)
#define C3set sbi(PORTB, 7)
#define C3rst cbi(PORTB, 7)
#define D3set sbi(PORTG, 3)
#define D3rst cbi(PORTG, 3)
#define E3set sbi(PORTG, 4)
#define E3rst cbi(PORTG, 4)
#define F3set sbi(PORTD, 6)
#define F3rst cbi(PORTD, 6)
#define G3set sbi(PORTD, 5)
#define G3rst cbi(PORTD, 5)

// Segment4 Pin Config
#define A4set sbi(PORTD, 1)
#define A4rst cbi(PORTD, 1)
#define B4set sbi(PORTD, 0)
#define B4rst cbi(PORTD, 0)
#define C4set sbi(PORTB, 3)
#define C4rst cbi(PORTB, 3)
#define D4set sbi(PORTB, 5)
#define D4rst cbi(PORTB, 5)
#define E4set sbi(PORTB, 6)
#define E4rst cbi(PORTB, 6)
#define F4set sbi(PORTD, 2)
#define F4rst cbi(PORTD, 2)
#define G4set sbi(PORTB, 4)
#define G4rst cbi(PORTB, 4)
#define H4set cbi(PORTB, 2)
#define H4rst sbi(PORTB, 2)

// Segment5 Pin Config
#define A5set sbi(PORTG, 2)
#define A5rst cbi(PORTG, 2)
#define B5set sbi(PORTC, 7)
#define B5rst cbi(PORTC, 7)
#define C5set sbi(PORTC, 1)
#define C5rst cbi(PORTC, 1)
#define D5set sbi(PORTC, 2)
#define D5rst cbi(PORTC, 2)
#define E5set sbi(PORTC, 3)
#define E5rst cbi(PORTC, 3)
#define F5set sbi(PORTA, 6)
#define F5rst cbi(PORTA, 6)
#define G5set sbi(PORTA, 7)
#define G5rst cbi(PORTA, 7)

// Segment6 Pin Config
#define A6set sbi(PORTC, 5)
#define A6rst cbi(PORTC, 5)
#define B6set sbi(PORTC, 4)
#define B6rst cbi(PORTC, 4)
#define C6set sbi(PORTD, 7)
#define C6rst cbi(PORTD, 7)
#define D6set sbi(PORTG, 1)
#define D6rst cbi(PORTG, 1)
#define E6set sbi(PORTC, 0)
#define E6rst cbi(PORTC, 0)
#define F6set sbi(PORTC, 6)
#define F6rst cbi(PORTC, 6)
#define G6set sbi(PORTG, 0)
#define G6rst cbi(PORTG, 0)

//                                                                                   11    12    13   14    15    16     17    18    19   20     21   22     23    24    25   26     27   28     29    30    31   32    33    34    35    36
char segcode[] = {0x02, 0x9e, 0x24, 0x0c, 0x98, 0x48, 0x40, 0x1e, 0x00, 0x08, 0xff, 0x10, 0xc0, 0x62, 0x84, 0x60, 0x70, 0x42, 0x90, 0xf2, 0x8e, 0xa0, 0xe2, 0x56, 0xd4, 0x02, 0x30, 0x18, 0xf4, 0x48, 0xe0, 0x82, 0xc6, 0xaa, 0xd8, 0x88, 0x6c, 0xff};
//                                                                                  'A'   'b'   'C'   'd'   'E    'F'   'G'   'H'   'i'   'J'   'k'   'L'   'M'   'n'   'O'   'p'   'q'   'r'   'S'   't'   'U'   'v'   'w'   'x'   'y'   'z'
// enum Words{
//    A = 10,    B,    C,    D,    E,    F,    G,    H,    I,    J,    K,    L,    M,    N,    O,    P,    Q,    R,    S,    T,    U,    V,    W,    X,    Y,    Z
//};

void SegTest(void);
void Seg1(char num);
void Seg2(char num);
void Seg3(char num);
void Seg4(char num);
void Seg5(char num);
void Seg6(char num);

char getData(char *str);
char getData2(char *str);
void Display(void);
void InitSegments(void);

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

void main(void)
{
    // Declare your local variables here

    // Input/Output Ports initialization
    // Port A initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
    DDRA = (1 << DDA7) | (1 << DDA6) | (1 << DDA5) | (1 << DDA4) | (1 << DDA3) | (1 << DDA2) | (1 << DDA1) | (1 << DDA0);
    // State: Bit7=1 Bit6=1 Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=1 Bit0=1
    PORTA = (1 << PORTA7) | (1 << PORTA6) | (1 << PORTA5) | (1 << PORTA4) | (1 << PORTA3) | (1 << PORTA2) | (1 << PORTA1) | (1 << PORTA0);

    // Port B initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In
    DDRB = (1 << DDB7) | (1 << DDB6) | (1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (0 << DDB1) | (0 << DDB0);
    // State: Bit7=1 Bit6=1 Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=T Bit0=T
    PORTB = (1 << PORTB7) | (1 << PORTB6) | (1 << PORTB5) | (1 << PORTB4) | (1 << PORTB3) | (1 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

    // Port C initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
    DDRC = (1 << DDC7) | (1 << DDC6) | (1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0);
    // State: Bit7=1 Bit6=1 Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=1 Bit0=1
    PORTC = (1 << PORTC7) | (1 << PORTC6) | (1 << PORTC5) | (1 << PORTC4) | (1 << PORTC3) | (1 << PORTC2) | (1 << PORTC1) | (1 << PORTC0);

    // Port D initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
    DDRD = (1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4) | (1 << DDD3) | (1 << DDD2) | (1 << DDD1) | (1 << DDD0);
    // State: Bit7=1 Bit6=1 Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=1 Bit0=1
    PORTD = (1 << PORTD7) | (1 << PORTD6) | (1 << PORTD5) | (1 << PORTD4) | (1 << PORTD3) | (1 << PORTD2) | (1 << PORTD1) | (1 << PORTD0);

    // Port E initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRE = (0 << DDE7) | (0 << DDE6) | (0 << DDE5) | (0 << DDE4) | (0 << DDE3) | (0 << DDE2) | (0 << DDE1) | (0 << DDE0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTE = (0 << PORTE7) | (0 << PORTE6) | (0 << PORTE5) | (0 << PORTE4) | (0 << PORTE3) | (0 << PORTE2) | (0 << PORTE1) | (0 << PORTE0);

    // Port F initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
    DDRF = (1 << DDF7) | (1 << DDF6) | (1 << DDF5) | (1 << DDF4) | (1 << DDF3) | (1 << DDF2) | (1 << DDF1) | (1 << DDF0);
    // State: Bit7=1 Bit6=1 Bit5=1 Bit4=1 Bit3=1 Bit2=1 Bit1=1 Bit0=1
    PORTF = (1 << PORTF7) | (1 << PORTF6) | (1 << PORTF5) | (1 << PORTF4) | (1 << PORTF3) | (1 << PORTF2) | (1 << PORTF1) | (1 << PORTF0);

    // Port G initialization
    // Function: Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
    DDRG = (1 << DDG4) | (1 << DDG3) | (1 << DDG2) | (1 << DDG1) | (1 << DDG0);
    // State: Bit4=1 Bit3=1 Bit2=1 Bit1=1 Bit0=1
    PORTG = (1 << PORTG4) | (1 << PORTG3) | (1 << PORTG2) | (1 << PORTG1) | (1 << PORTG0);

    // USART0 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART0 Receiver: On
    // USART0 Transmitter: Off
    // USART0 Mode: Asynchronous
    // USART0 Baud Rate: 9600 (Double Speed Mode)
    UCSR0A = (0 << RXC0) | (0 << TXC0) | (0 << UDRE0) | (0 << FE0) | (0 << DOR0) | (0 << UPE0) | (1 << U2X0) | (0 << MPCM0);
    UCSR0B = (1 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (1 << RXEN0) | (0 << TXEN0) | (0 << UCSZ02) | (0 << RXB80) | (0 << TXB80);
    UCSR0C = (0 << UMSEL0) | (0 << UPM01) | (0 << UPM00) | (0 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0);
    UBRR0H = 0x00;
    UBRR0L = 0xCF;

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: 15.625 kHz
    // Mode: Normal top=0xFF
    // OC0 output: Disconnected
    // Timer Period: 16.384 ms
    ASSR = 0 << AS0;
    TCCR0 = (0 << WGM00) | (0 << COM01) | (0 << COM00) | (0 << WGM01) | (0 << CS02) | (0 << CS01) | (0 << CS00);
    TCNT0 = 0x00;
    OCR0 = 0x00;

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
    // Input Capture Interrupt: On
    // Compare A Match Interrupt: On
    // Compare B Match Interrupt: On
    // Compare C Match Interrupt: On
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << COM1C1) | (0 << COM1C0) | (0 << WGM11) | (0 << WGM10);
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
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
    // Clock value: 15.625 kHz
    // Mode: Normal top=0xFF
    // OC2 output: Disconnected
    // Timer Period: 16.384 ms
    TCCR2 = (0 << WGM20) | (0 << COM21) | (0 << COM20) | (0 << WGM21) | (0 << CS22) | (0 << CS21) | (0 << CS20);
    TCNT2 = 0x00;
    OCR2 = 0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK = (0 << OCIE2) | (0 << TOIE2) | (0 << TICIE1) | (0 << OCIE1A) | (0 << OCIE1B) | (0 << TOIE1) | (0 << OCIE0) | (0 << TOIE0);

    // Watchdog Timer initialization
    // Watchdog Timer Prescaler: OSC/16k
    //#pragma optsize-
    // WDTCR=(1<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
    // WDTCR=(0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
    //#ifdef _OPTIMIZE_SIZE_
    //#pragma optsize+
    //#endif

    // SegTest();
    InitSegments();

// Global enable interrupts
#asm("sei")

    while (1)
    {
        // Place your code here
        Display();
    }
}

//*****************************************************************************************
// Get Segment Data
char getData(char *str)
{
#if RX_BUFFER_SIZE0 <= 256
    static unsigned char check = 0;
#else
    static unsigned int check = 0;
#endif
    char i = 0;

    if (check != rx_wr_index0)
    {
        check = rx_wr_index0;
        do
        {
            str[i] = getchar();
        } while (str[i++] != 0x0A);

        return 1;
    }

    return 0;
}

char getData2(char *str)
{

    char i = 0;
        do
        {
            str[i] = getchar();
        } while (str[i++] != 0x0A);

        return 1;
}

// Display data on Segments:
void Display(void)
{
    char i, flickCode[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    //if (getData(str))
    if (getData2(str))
    {
        segment.sg1 = str[0];
        segment.sg2 = str[1];
        segment.sg3 = str[2];
        segment.sg4 = str[3];
        segment.sg5 = str[4];
        segment.sg6 = str[5];
        segment.flickCode = str[6];

        for (i = 0; i <= 7; i++)
        {
            flickCode[i] = segment.flickCode & (1 << i);
        }
    }

    Seg1(segcode[segment.sg1]);
    Seg2(segcode[segment.sg2]);
    Seg3(segcode[segment.sg3]);
    Seg4(segcode[segment.sg4]);
    Seg5(segcode[segment.sg5]);
    Seg6(segcode[segment.sg6]);
    if (flickCode[7])
        H4rst;
    else
        H4set;
    delay_ms(flc);

    if (flickCode[0])
    {
        H4rst;
    }
    if (flickCode[1])
    {
        Seg1(0xff);
        // delay_ms(flc);
    }
    if (flickCode[2])
    {
        Seg2(0xff);
        // delay_ms(flc);
    }
    if (flickCode[3])
    {
        Seg3(0xff);
        // delay_ms(flc);
    }
    if (flickCode[4])
    {
        Seg4(0xff);
        // delay_ms(flc);
    }
    if (flickCode[5])
    {
        Seg5(0xff);
        // delay_ms(flc);
    }
    if (flickCode[6])
    {
        Seg6(0xff);
        // delay_ms(flc);
    }
    delay_ms(flc);
}

void InitSegments(void)
{
    char i = 0, j = 0, k = 0, dance[] = {0x7e, 0xfc, 0xee};
    while (++k < 10)
    {
        if (i <= 2 & j == 0)
        {
            Seg1(dance[i]);
            Seg2(dance[i]);
            Seg3(dance[i]);
            Seg4(dance[i]);
            Seg5(dance[i]);
            Seg6(dance[i++]);
            delay_ms(25);
            if (i == 2)
                j = 1;
        }
        if (i >= 0 & j == 1)
        {
            Seg1(dance[i]);
            Seg2(dance[i]);
            Seg3(dance[i]);
            Seg4(dance[i]);
            Seg5(dance[i]);
            Seg6(dance[i--]);
            delay_ms(25);
            if (i == 0)
                j = 0;
        }
    }
}
// Segments Test
void SegTest(void)
{
    char i, dl1 = 100, dl2 = 250;

    for (i = 0; i < 12; i++)
    {
        if (i < 10)
        {
            Seg1(segcode[i]);
            Seg2(segcode[i]);
            Seg3(segcode[i]);
            Seg4(segcode[i]);
            Seg5(segcode[i]);
            Seg6(segcode[i]);
            delay_ms(dl1);
        }
        else
        {
            Seg1(0xff);
            Seg2(0xff);
            Seg3(0xff);
            Seg4(0xff);
            Seg5(0xff);
            Seg6(0xff);
            delay_ms(dl2);
            Seg1(0x02);
            Seg2(0x02);
            Seg3(0x02);
            Seg4(0x02);
            Seg5(0x02);
            Seg6(0x02);
            delay_ms(dl2);
        }
    }
    Seg1(0xff);
    Seg2(0xff);
    Seg3(0xff);
    Seg4(0xff);
    Seg5(0xff);
    Seg6(0xff);
    delay_ms(dl2);
}

// Segment1 Driver
void Seg1(char num)
{
    if (num & 1 << 7)
        A1set;
    else
        A1rst;

    if (num & 1 << 6)
        B1set;
    else
        B1rst;

    if (num & 1 << 5)
        C1set;
    else
        C1rst;

    if (num & 1 << 4)
        D1set;
    else
        D1rst;

    if (num & 1 << 3)
        E1set;
    else
        E1rst;

    if (num & 1 << 2)
        F1set;
    else
        F1rst;

    if (num & 1 << 1)
        G1set;
    else
        G1rst;

    delay_ms(10);
}

// Segment2 Driver
void Seg2(char num)
{
    if (num & 1 << 7)
        A2set;
    else
        A2rst;

    if (num & 1 << 6)
        B2set;
    else
        B2rst;

    if (num & 1 << 5)
        C2set;
    else
        C2rst;

    if (num & 1 << 4)
        D2set;
    else
        D2rst;

    if (num & 1 << 3)
        E2set;
    else
        E2rst;

    if (num & 1 << 2)
        F2set;
    else
        F2rst;

    if (num & 1 << 1)
        G2set;
    else
        G2rst;

    delay_ms(10);
}

// Segment3 Driver
void Seg3(char num)
{
    if (num & 1 << 7)
        A3set;
    else
        A3rst;

    if (num & 1 << 6)
        B3set;
    else
        B3rst;

    if (num & 1 << 5)
        C3set;
    else
        C3rst;

    if (num & 1 << 4)
        D3set;
    else
        D3rst;

    if (num & 1 << 3)
        E3set;
    else
        E3rst;

    if (num & 1 << 2)
        F3set;
    else
        F3rst;

    if (num & 1 << 1)
        G3set;
    else
        G3rst;

    delay_ms(10);
}

// Segment4 Driver
void Seg4(char num)
{
    if (num & 1 << 7)
        A4set;
    else
        A4rst;

    if (num & 1 << 6)
        B4set;
    else
        B4rst;

    if (num & 1 << 5)
        C4set;
    else
        C4rst;

    if (num & 1 << 4)
        D4set;
    else
        D4rst;

    if (num & 1 << 3)
        E4set;
    else
        E4rst;

    if (num & 1 << 2)
        F4set;
    else
        F4rst;

    if (num & 1 << 1)
        G4set;
    else
        G4rst;

    delay_ms(10);
}

// Segment5 Driver
void Seg5(char num)
{
    if (num & 1 << 7)
        A5set;
    else
        A5rst;

    if (num & 1 << 6)
        B5set;
    else
        B5rst;

    if (num & 1 << 5)
        C5set;
    else
        C5rst;

    if (num & 1 << 4)
        D5set;
    else
        D5rst;

    if (num & 1 << 3)
        E5set;
    else
        E5rst;

    if (num & 1 << 2)
        F5set;
    else
        F5rst;

    if (num & 1 << 1)
        G5set;
    else
        G5rst;

    delay_ms(10);
}

// Segment6 Driver
void Seg6(char num)
{
    if (num & 1 << 7)
        A6set;
    else
        A6rst;

    if (num & 1 << 6)
        B6set;
    else
        B6rst;

    if (num & 1 << 5)
        C6set;
    else
        C6rst;

    if (num & 1 << 4)
        D6set;
    else
        D6rst;

    if (num & 1 << 3)
        E6set;
    else
        E6rst;

    if (num & 1 << 2)
        F6set;
    else
        F6rst;

    if (num & 1 << 1)
        G6set;
    else
        G6rst;

    delay_ms(10);
}
