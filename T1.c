#define F_CPU 1000000UL
#include <mega8.h>
#include <io.h>
#include <delay.h>

#define PC0   (1<<0); // вперёд
#define PC1   (1<<1); // назад
#define PC2   (1<<2); // Тампер впереди
#define PC3   (1<<3); // Тампер сзади

short int statusMotor;
short int pwm_dir; // 1 - forward, 0 - back

void initializationDefolt()
{
    // Declare your local variables here

    // Input/Output Ports initialization
    // Port B initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=OUT Bit2=In Bit1=In Bit0=In
    DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (1<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

    // Port C initialization
    // Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (1<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
    // State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

    // Port D initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (1<<DDD0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    TCCR0=(0<<CS02) | (0<<CS01) | (0<<CS00);
    TCNT0=0x00;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: Timer1 Stopped
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer2 Stopped
    // Mode: Normal top=0xFF
    // OC2 output: Disconnected
    ASSR=0<<AS2;
    TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
    TCNT2=0x00;
    OCR2=0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<TOIE0);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    MCUCR=(0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (0<<ISC00);
    GICR|=(1<<INT1) | (0<<INT0);
    MCUCR=(0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (0<<ISC00);
    GIFR=(1<<INTF1) | (0<<INTF0);

    // USART initialization
    // USART disabled
    UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

    // Analog Comparator initialization
    // Analog Comparator: Off
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
    SFIOR=(0<<ACME);

    // ADC initialization
    // ADC disabled
    ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

    // SPI initialization
    // SPI disabled
    SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

    // TWI initialization
    // TWI disabled
    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);
}

void initPWM()
{
    ASSR = 0x00;
    TCCR2 = 0x00;    // таймер отключен
    TCNT2 = 0x00;
    OCR2 = 0x00;
}

int checkPortPC()
{
    int i, j = 0, k = 0;
    //PORTD |= (1<<6);
    for(i = 0; i <= 4; i++)
    {
        if(~PINC & (1<<i))
        {
            j = i;
            //k++;
        }
    }
    //if(k == 1)
    {
        return j;
    }
    //else return 255;
}

void PWM()
{
    if(statusMotor == 0)
    {
        OCR2 = 0x00;
        TCCR2 = 0b01101100; //start timer
        while(OCR2 != 0xFF)
        {
            OCR2++;
            delay_ms(3);
        }
        statusMotor = 1;
    }
    else
    {
        OCR2 = 0xFF;
        TCCR2 = 0b01101100; //start timer
        while(OCR2 != 0x00)
        {
            OCR2--;
            delay_ms(3);
        }
        statusMotor = 0;
    }
    TCCR2 = 0x00; //stop timer
    OCR2 = 0x00;
    if(statusMotor) PORTB |= (1<<3);
    else PORTB &= ~(1<<3);
    return;

    /*if(pwm_dir)
      {
        OCR2++;
      }
    else OCR2--;
    delay_us(500);
    if(OCR2 == 0xFF)
       pwm_dir = 0;
    else if(OCR2 == 0x00)
       pwm_dir = 1;  */
}

void goUpDown(char dir) //1 - up, 0 - down
{
   if (dir) PORTB |= (1<<4);
   else PORTB &= ~(1<<4);
   PWM();
}

void main()
{
    pwm_dir = 1; // 1 - forward, 0 - back
    #asm("cli")
    initializationDefolt();
    initPWM();
    #asm("sei")
    while (1)
    {

    }
}

interrupt [EXT_INT1] void ext_int1_isr(void)
{
    PORTD = (1<<5);
    switch(checkPortPC())
    {
        case 0:
            PORTD |= (1<<7);
            goUpDown(1);
            PORTD &= ~(1<<7);
            break;
        case 1:
        {
            PORTD |= (1<<6);
            goUpDown(0);
            PORTD &= ~(1<<6);
        }
        case 2: {break;}
        case 3: {break;}
        case 4: {break;}
        default: {}
    }

    PORTD &= ~(1<<5);
}
