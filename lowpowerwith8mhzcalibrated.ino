//somewhere i have seen code credit to S. James Remington 3/2013
// low power BCD RTC using bare bones ATMega328p, no regulator, 3-5V
// update 3/1/2019
// This code creates an RTC formed by Timer2 and a 32768 xtal+2x30 pF caps on OSC pins
// The internal RC oscillator is calibrated by comparison with the 32768 standard,
// Low power operation borrowed heavily from https://www.gammon.com.au/power

// SET FUSES: 8 MHz internal RC clock, LP xtal (32768 Hz) on OSC pins
// S. James Remington 3/2013

//reminder: 8 MHz internal RC
//#define F_CPU 8000000UL

#include <avr/sleep.h>
#include <util/delay.h>

// Global variables for RTC
volatile unsigned char RTC_buf[] = {0, 0, 0, 0, 0, 0}; //initialize BCD digits coding for hh:mm:ss
volatile unsigned int dayno = 0; //days since startup

#define BUF_LEN 20
char buf[BUF_LEN];  //print message buffer

void setup() {
  Serial.begin(9600);
  Serial.println("Simple RTC");

  pinMode(2, OUTPUT); //LED on PD2
  for (int i = 1; i < 4; i++) {
    digitalWrite(2, 1);
    delay(100);
    digitalWrite(2, 0);
    delay(900);
  }
  // calibrate 8 MHz oscillator using the 32 kHz xtal
  OSCCAL_calibrate();
  Serial.print("OSCCAL (cal) ");
  Serial.println(OSCCAL);
  Serial.flush(); //finish printing before turning off USART

  //PRR Power Reduction Register (set PRADC after ADCSRA=0)
  //Bit 7 - PRTWI: Power Reduction TWI
  //Bit 6 - PRTIM2: Power Reduction Timer/Counter2
  //Bit 5 - PRTIM0: Power Reduction Timer/Counter0
  //Bit 3 - PRTIM1: Power Reduction Timer/Counter1
  //Bit 2 - PRSPI: Power Reduction Serial Peripheral Interface
  //Bit 1 - PRUSART0: Power Reduction USART0
  //Bit 0 - PRADC: Power Reduction ADC

  ADCSRA = 0; //disable ADC
  // turn off  unneeded modules. 
  PRR |= (1 << PRTWI) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC) | (1 << PRUSART0) ;

  // reprogram timer 2 for this application

  timer2_init();
}

void loop() {

  char t = 0;
  unsigned int batt, h, m, s;

  while (1) {

    // wake up on Timer2 overflow (1/sec)
    // output day, time and cpu voltage on serial monitor every ... (10 seconds now)

    if (t != RTC_buf[4]) { //have 10 seconds passed?

      t = RTC_buf[4];
      s = 10 * RTC_buf[4] + RTC_buf[5]; //format time
      m = 10 * RTC_buf[2] + RTC_buf[3];
      h = 10 * RTC_buf[0] + RTC_buf[1];

      PRR &= ~((1 << PRADC) | (1 << PRUSART0)); //turn on ADC and USART
      ADCSRA = (1 << ADEN); //set up ADC properly
      ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // ADC Prescaler=128

      delay(1); //let ADC and USART stabilize

      batt = readVcc();  //read battery voltage

      //    print day, time and battery voltage

      snprintf(buf, BUF_LEN, "%u,%02u:%02u:%02u,%u", dayno, h, m, s, batt); //max ~16 characters
      Serial.println(buf);
      Serial.flush(); //finish printing before going to sleep

      // back to sleep with modules off

      ADCSRA = 0; //ADC off
      PRR |= (1 << PRADC) | (1 << PRUSART0);  //turn off ADC and USART

    } //end if (t)

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    cli(); //time critical steps follow
    MCUCR = (1 << BODS) | (1 << BODSE); // turn on brown-out enable select
    MCUCR = (1 << BODS);         //Brown out off. This must be done within 4 clock cycles of above
    sei();
    sleep_cpu();

  } //end while(1)
}

//******************************************************************
//  Timer2 Interrupt Service
//  32 kKz / 256 = 1 Hz with Timer2 prescaler 128
//  provides global tick timer and BCD binary Real Time Clock
//  no check for illegal values of RTC_buffer upon startup!

ISR (TIMER2_OVF_vect) {

  // RTC function

  RTC_buf[5]++; // increment second

  if (RTC_buf[5] > 9)
  {
    RTC_buf[5] = 0; // increment ten seconds
    RTC_buf[4]++;
    if ( RTC_buf[4] > 5)
    {
      RTC_buf[4] = 0;
      RTC_buf[3]++; // increment minutes
      if (RTC_buf[3] > 9)
      {
        RTC_buf[3] = 0;
        RTC_buf[2]++; // increment ten minutes

        if (RTC_buf[2] > 5)
        {
          RTC_buf[2] = 0;
          RTC_buf[1]++; // increment hours
          char b = RTC_buf[0]; // tens of hours, handle rollover at 19 or 23
          if ( ((b < 2) && (RTC_buf[1] > 9)) || ((b == 2) && (RTC_buf[1] > 3)) )
          {
            RTC_buf[1] = 0;
            RTC_buf[0]++; // increment ten hours and day number, if midnight rollover
            if (RTC_buf[0] > 2) {
              RTC_buf[0] = 0;
              dayno++;  //count days since startup
            }
          }
        }
      }
    }
  }
}


/*
  // initialize Timer2 as asynchronous 32768 Hz timing source
*/

void timer2_init(void) {

  TCCR2B = 0;  //stop Timer 2
  TIMSK2 = 0; // disable Timer 2 interrupts
  ASSR = (1 << AS2); // select asynchronous operation of Timer2
  TCNT2 = 0; // clear Timer 2 counter
  TCCR2A = 0; //normal count up mode, no port output
  TCCR2B = (1 << CS22) | (1 << CS20); // select prescaler 128 => 1 sec between each overflow

  while (ASSR & ((1 << TCN2UB) | (1 << TCR2BUB))); // wait for TCN2UB and TCR2BUB to be cleared

  TIFR2 = (1 << TOV2); // clear interrupt-flag
  TIMSK2 = (1 << TOIE2); // enable Timer2 overflow interrupt
}

// Read 1.1V reference against AVcc
// EVERY PROCESSOR MUST BE CALIBRATED INDIVIDUALLY!

unsigned int readVcc(void) {

  unsigned int result;

  // set the reference to Vcc and the measurement to the internal 1.1V reference

  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
  delay(2); // Wait for Vref to settle

  ADCSRA |= (1 << ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // wait until done
  result = ADC;

  // two is better than one

  ADCSRA |= (1 << ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // wait until done
  result = ADC;

  // calibrated for Miniduino board
  result = 1195700UL / (unsigned long)result; //1126400 = 1.1*1024*1000
  return result; // Vcc in millivolts
}

//
// Calibrate the internal OSCCAL byte, using the external 32768 Hz crystal as reference.
//

void OSCCAL_calibrate(void)  //This version specific to ATmegaXX8 (tested with ATmega328)

{
  unsigned char calibrate = 0; //FALSE;
  unsigned int temp;

  TIMSK1 = 0; //disable Timer1,2 interrupts
  TIMSK2 = 0;

  ASSR = (1 << AS2);      //select asynchronous operation of timer2 (32,768kHz)
  OCR2A = 200;            // set timer2 compare value
  TCCR1A = 0;
  TCCR1B = (1 << CS11);   // start timer1 with prescaler 8
  TCCR2A = 0;
  TCCR2B = (1 << CS20);   // start timer2 with no prescaling (ATmega169 use TCCR2A!)

  while (ASSR & ((1 << TCN2UB) | (1 << TCR2BUB))); //wait for TCN2UB and TCR2BUB to be cleared

  delay(5000); //5 seconds to allow xtal osc to stabilize

  while (!calibrate)
  {
    cli(); // disable global interrupts

    TIFR1 = 0xFF;   // clear TIFR1 flags
    TIFR2 = 0xFF;   // clear TIFR2 flags

    TCNT1 = 0;      // clear timer1 counter
    TCNT2 = 0;      // clear timer2 counter

    while ( !(TIFR2 & (1 << OCF2A)) ); // wait for timer2 compareflag

    TCCR1B = 0; // stop timer1

    sei(); // enable global interrupts

    if ( (TIFR1 & (1 << TOV1)) ) temp = 0xFFFF; //overflow, load max
    else   temp = TCNT1;

    if (temp > 6150)  //expect about (1e6/32768)*201 = 6134 ticks
    {
      OSCCAL--;   //RC oscillator runs too fast, decrease OSCCAL
    }
    else if (temp < 6120)
    {
      OSCCAL++;   //RC oscillator runs too slow, increase OSCCAL
    }
    else calibrate = 1; //done

    TCCR1B = (1 << CS11); // (re)start timer1

  } //end while(!calibrate)
} //return
