/*
 * RX_v2_6.c
 * version 2.6
 * based on v.2.4
 * Created: Sep.4, 2018 1:11:11 PM
 * Modified: March 5, 2019  2:22:22 PM
 * Author: dmitry
 */

#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <string.h>

#define BAUD 2400UL
#define UBRR ((F_CPU) / ((BAUD) * (16UL)) - 1)
#define PWM_top 2500
#define D1B(x) (uint16_t)(((x) * (uint32_t)(PWM_top)) >> 8)
#define DELAY_ms(x)                                                            \
  TIMSK1 &= ~(1 << ICIE1);                                                     \
  delay_ms = 0;                                                                \
  TIMSK1 |= (1 << ICIE1);                                                      \
  while (delay_ms <= x)

#define BAT_min 108
// Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133
// Vbatt warn~=14V (ADC=133)

// Averaging filter
// If you use an ultrasonic sensor (US), this value MUST be set to 1 as per the
// sensor's manufacturer application notes. If you wish to filter the values
// from US sensor, you should use median or mode filter. If you use an IR
// sensor, you can set it to a reasonable value, something between 4 and 10
// should work well.
#define ADC_sample_max 1

// Distance threshold for IR sensor. Change it for the US one.
// Note that IR and US sensors have different distance-voltage curves.
// You can use the "ADC_DEBUG" option (see below) to get the corresponding ADC
// reading.
#define DIST_TH 76
// 1.5/5*255

// Threshold for the distance readings variation
#define DELTA 5

//***************************************************************************************************************
// un-comment the line below to enable printing of ADC readings through the
// serial port #define ADC_DEBUG
// COM port settings: 2400, 8, ODD, 1, None
// Note 1: for this connection, the flow control must be set to "None".
// Note 2: Use any terminal software to display the data (e.g. Hyperterminal)
// Note 3: Arduino IDE must be closed before launching the terminal,
// and the terminal must be closed before launching Arduino IDE. I.e. only one
// software should access the serial port. Note 4: The ADC values are printed in
// the following order: ADC5, ADC4, ADC3. Note 5: Since unconnected ADC inputs
// are floating, the displayed values of unconnected channels are quasi-random.
//***************************************************************************************************************

static volatile uint8_t msg_char = 1, rx_buff, buttons, V_batt = 255,
                        ADC_sample, ctrl[5], ctrl_count, checksum;
static volatile uint16_t time, ADC_acc, delay_ms;
static volatile uint8_t wdt_soft;
static volatile struct {
  uint8_t mode : 1;
  uint8_t stop : 1;
  uint8_t ADC_ready : 1;
} flags;

static volatile struct {
  uint8_t ADC3;
  uint8_t ADC4;
  uint8_t ADC5;
} ADC_data;

#ifdef ADC_DEBUG
#define PPS 50
// defines the frequency of printing of ADC readings.
// Current value (50) corresponds to one set per second.
// If you set it to 10, it will print 5 sets of readings per second.
// Do not set it too low because you might overload the TX ISR.
static volatile char *msg, data_str[] = "   ;   ;   .\r\n";
static volatile uint8_t TX_delay = PPS;
#endif

// Values for 16MHz
const uint16_t Servo_angle[256] = {
    //  +/-90 degrees
    85,  85,  85,  85,  85,  85,  85,  85,  85,
    85,  85,  85,  85,  85,  85,  85, // 0...15
    108, 108, 108, 108, 108, 108, 108, 108, 108,
    108, 108, 108, 108, 108, 108, 108, // 16...31 0.9ms pulse(108)
    119, 119, 119, 119, 119, 119, 119, 119, 119,
    119, 119, 119, 119, 119, 119, 119, // 32...47
    131, 131, 131, 131, 131, 131, 131, 131, 131,
    131, 131, 131, 131, 131, 131, 131, // 48...63
    142, 142, 142, 142, 142, 142, 142, 142, 142,
    142, 142, 142, 142, 142, 142, 142, // 64...79
    154, 154, 154, 154, 154, 154, 154, 154, 154,
    154, 154, 154, 154, 154, 154, 154, // 80...95
    165, 165, 165, 165, 165, 165, 165, 165, 165,
    165, 165, 165, 165, 165, 165, 165, // 96...111
    177, 177, 177, 177, 177, 177, 177, 177, 177,
    177, 177, 177, 188, 188, 188, 188, // center - 1.5ms pulse (188) 112...127
    188, 188, 188, 188, 198, 198, 198, 198, 198,
    198, 198, 198, 198, 198, 198, 198, // 128...143
    208, 208, 208, 208, 208, 208, 208, 208, 208,
    208, 208, 208, 208, 208, 208, 208, // 143...159
    218, 218, 218, 218, 218, 218, 218, 218, 218,
    218, 218, 218, 218, 218, 218, 218, // 160...175
    228, 228, 228, 228, 228, 228, 228, 228, 228,
    228, 228, 228, 228, 228, 228, 228, // 175...191
    238, 238, 238, 238, 238, 238, 238, 238, 238,
    238, 238, 238, 238, 238, 238, 238, // 192...207
    248, 248, 248, 248, 248, 248, 248, 248, 248,
    248, 248, 248, 248, 248, 248, 248, // 208...223
    270, 270, 270, 270, 270, 270, 270, 270, 270,
    270, 270, 270, 270, 270, 270, 270, // 224...239
    290, 290, 290, 290, 290, 290, 290, 290, 290,
    290, 290, 290, 290, 290, 290, 290 // 240...255
};

// Change this number to the ID that was given to you.
#define Team_NO 0xF1

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++ Here is the placeholder for your control algorithm
//++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void control_script() {
  /*
  ----------------------------------------------------------------------------------
  Sample code for commands
  Lift fan is connected to the "throttle" channel (PWM2).
  Servo (PWM0) controls the "propulsion" fan, which is connected to Up/Down
  channel (PWM1).
  */

  /*
  // If you do not use the remote controll commands, comment the lines below
    for (int16_t i=500; i>0; i--){ // 1s loop
      DELAY_ms(20);
      if (ADC_data.ADC3>DIST_TH) {
        flags.stop=1;
        return;
      }
    }
  */

  /*
  // Go straight for 2s checking if the wall is at ~40cm range
    OCR1A=Servo_angle[127]; //servo at the middle
    OCR0A=255; //full lift
    OCR1B=D1B(256); //full speed
    for (uint8_t i=50; i>0; i--){
      if (ADC_data.ADC3>DIST_TH) break;
      DELAY_ms (40);
    }

  */

  /*
  To check digital IO pins:

  if (PIND&PD2) { //if PD2 is high

  }

  if (!(PIND&PD2)) { //if PD2 is low

  }

  // Note that in current release PD4 is used as output to show ADC timing vs
  polling period
  // You can connect an oscilloscope to it to see it
  // If you wish to use it as input, you have to redefine it in the init section
  and remove XOR in the ADR ISR.
  */

  // Align perpendicular to the wall for 20s.
  // Note:
  // 1) The wall must be already in the range.
  // 2) The example is NOT practical - it is here just to show you how to
  // operate the controls.
  uint8_t dist_old = ADC_data.ADC3;
  OCR0A = 127;      // hover, but not too high
  OCR1B = D1B(127); // turning at lower speed speed
  uint8_t a = 0;
  OCR1A =
      Servo_angle[a]; // servo to one side - change it to more appropriate angle
  for (int16_t i = 500; i > 0; i--) {
    if (ADC_data.ADC3 < dist_old - DELTA) {
      dist_old = ADC_data.ADC3;
      a = ~a;
      OCR1A = Servo_angle[a]; // move servo to the other side
      DELAY_ms(600);
      i -= 15;
      continue;
    }
    dist_old = ADC_data.ADC3;
    DELAY_ms(40);
  }

  flags.stop = 1; // this must be the very last command in your script. It will
                  // ground the hovercraft.
  return;
} // end of control_script()

//====================================================================================|
//====================================================================================|
//===DO NOT CHANGE ANYTHING BELOW THESE LINES (unless you know what your are
// doing)===|
//====================================================================================|
//====================================================================================|

ISR(__vector_default) {}

// >>>>>>>>>>>>>>>>>>>>>>>>> ADC <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ISR(ADC_vect) {
  PORTD ^= (1 << PD4); // to check ISR timing
  if (ADC_sample == 0) {
    ADC_acc = 0;
    ADC_sample++;
    return;
  }
  if (ADC_sample <= ADC_sample_max) {
    ADC_acc += ADCH;
    ADC_sample++;
    return;
  }
  if (ADC_sample > ADC_sample_max) {
    ADC_sample = 0;
    switch (ADMUX & 7) {
    case 3: {
      ADMUX = (ADMUX & 0xF0) | 0x04;
      ADC_data.ADC3 = (uint8_t)(ADC_acc / ADC_sample_max);
      return;
    }

    case 4: {
      ADMUX = (ADMUX & 0xF0) | 0x05;
      ADC_data.ADC4 = (uint8_t)(ADC_acc / ADC_sample_max);
      return;
    }

    case 5: {
      ADMUX = (ADMUX & 0xF0) | 0x06;
      ADC_data.ADC5 = (uint8_t)(ADC_acc / ADC_sample_max);
      //    flags.ADC_ready=1;
      return;
    }

    case 6: {
      ADMUX = (ADMUX & 0xF0) | 0x03;
      ADCSRA &= ~((1 << ADEN) | (1 << ADIE));
      V_batt = (uint8_t)(ADC_acc / ADC_sample_max);
      if (V_batt < BAT_min) {
        V_batt = 0;
        return;
      }
      if ((V_batt < BAT_warn) && (TCCR0A & ((1 << COM0B1) | (1 << COM0B0)))) {
        TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
        return;
      }
      if (!(TCCR0A & ((1 << COM0B1) | (1 << COM0B0)))) {
        if (time > 50) {
          PORTD ^= (1 << PD5);
          time = 0;
          return;
        }
      }
      uint16_t PWM_temp;
      PWM_temp = (((uint16_t)V_batt - (uint16_t)BAT_warn) * 255) /
                 (255 - (uint16_t)BAT_warn);
      OCR0B = (uint8_t)PWM_temp;
      return;
    }
    default: {
      ADMUX = (ADMUX & 0xF0) | 0x03;
    }
    }
  }
}

// RX ISR
ISR(USART_RX_vect) {
  if (!((UCSR0A & (1 << FE0)) | (UCSR0A & (1 << DOR0)) |
        (UCSR0A & (1 << UPE0)))) {
    rx_buff = UDR0;
    switch (msg_char) {
    case 1:
      if (rx_buff == 0x55) {
        checksum = rx_buff;
        msg_char++;
        return;
      }
    case 2:
      if (rx_buff == Team_NO) {
        msg_char++;
        PORTB &= ~(1 << PB4);
        checksum = checksum + rx_buff;
        ctrl_count = 0;
      } else
        msg_char = 1;
      return;
    case 3:
    case 4:
    case 5:
    case 6:
      ctrl[ctrl_count] = rx_buff;
      ctrl_count++;
      msg_char++;
      checksum = checksum + rx_buff;
      return;
    case 7:
      msg_char = 1;
      ctrl_count = 0;
      if (rx_buff != checksum) { // bad check-sum?

        PORTB |= (1 << PB4);
        return;
      }
      wdt_soft = 50; // feed the dog
                     //         __asm__ __volatile__ ("wdr");

      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // >>>>>>> UN-comment the line below to disable remote control commands
      // processing once the "autonomous" mode was turned ON.
      //        if (flags.mode) return;

      if (ctrl[ctrl_count] & 0b00100010)
        flags.mode = 1;
      if (ctrl[ctrl_count] & 0b00010001)
        PORTD |= (1 << PD7);
      if (!(ctrl[ctrl_count] & 0b00010001))
        PORTD &= ~(1 << PD7);
      ctrl_count++;
      if (ctrl[ctrl_count - 1] & 0b01000100)
        OCR0A = ctrl[ctrl_count];
      else
        OCR0A = 0;
      ctrl_count++;
      if (ctrl[ctrl_count - 2] & 0b10001000)
        OCR1B = D1B(ctrl[ctrl_count] + 1);
      else
        OCR1B = 0;
      ctrl_count++;
      OCR1A = Servo_angle[ctrl[ctrl_count]];
      msg_char = 1;
      ctrl_count = 0;
      PORTB |= (1 << PB4);
      // comment the line below if the IR sensor is NOT connected
      //        if ((dist_raw>DIST_TH)&&flags.mode) flags.stop=1;
      return;
    default:
      msg_char = 1;
      ctrl_count = 0;
    }
  } else
    rx_buff = UDR0; // scrap the reading
}
// == Serial port receive end
// -----------------------------------------

ISR(TIMER1_CAPT_vect) { // system tick: 50Hz, 20ms
  if ((!(--wdt_soft)) && (!flags.mode)) {
    PORTB |= (1 << PB4);
    OCR1A = Servo_angle[127];
    OCR0A = 0;
    OCR1B = 0;
    PORTB &= ~(1 << PB0);
    PORTD &= ~(1 << PD7);
    ctrl[0] = 0;
    ctrl[1] = 127;
    ctrl[2] = 0;
    ctrl[3] = 0;
    wdt_soft = 50;
  }
  time++;
  delay_ms += 20;
  if (!(ADCSRA & (1 << ADEN)))
    ADCSRA |= ((1 << ADEN) | (1 << ADIE) | (1 << ADSC));

#ifdef ADC_DEBUG
  if (!TX_delay--) {
    flags.ADC_ready = 1;
    TX_delay = PPS;
  }
#endif
}

#ifdef ADC_DEBUG
ISR(USART_TX_vect) {
  if (*msg) {
    UDR0 = *msg;
    msg++;
  }
}
#endif

int main(void) {
  cli();
  // ------- Ports init -------
  DDRB |= ((1 << PB4) | (1 << PB2) | (1 << PB1) | (1 << PB0));
  PORTB &= ~((1 << PB2) | (1 << PB1) | (1 << PB0));
  PORTB |= ((1 << PB3) | (1 << PB4) | (1 << PB5));

  DDRC = (1 << PC2) | (1 << PC1);
  PORTC &= ~((1 << PC2) | (1 << PC1));
  PORTC |= (1 << PC0);

  DDRD |= (1 << PD7) | (1 << PD6) | (1 << PD5) | (1 << PD4) | (1 << PD1);
  PORTD &= ~((1 << PD7) | (1 << PD6));
  PORTD |= ((1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5));

  // --------UART init---------------------
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN0);
#ifdef ADC_DEBUG
  UCSR0B |= (1 << TXCIE0) | (1 << TXEN0);
#endif
  UCSR0C = (1 << UPM01) | (1 << UPM00) | (3 << UCSZ00);
  UBRR0H = (uint8_t)((UBRR) >> 8);
  UBRR0L = (uint8_t)UBRR;
  // -------- end UART init --------------

  // -------- Timer/PWM init
  // ======= PWM2 and D4 control (8-bit timer0) ===================
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << COM0B0);
  TCCR0A |= (1 << WGM00);
  OCR0A = 0;
  OCR0B = 255;
  TCCR0B |= ((1 << CS01) | (1 << CS00));

  // ======= PWM0 and PWM1 control (16-bit timer1) ===================
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << WGM13);
  ICR1 = PWM_top; // 50Hz PWM
  OCR1A = Servo_angle[127];
  OCR1B = 0;
  TCCR1B |= ((1 << CS11) | (1 << CS10));
  TIMSK1 |= (1 << ICIE1);

  // ADC init
  ADMUX = ((1 << ADLAR) | (1 << REFS0) | (1 << MUX2) | (1 << MUX1));
  ADCSRA = ((1 << ADEN) | (1 << ADIE));
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
  ADCSRA |= (1 << ADATE);
  ADCSRA |= (1 << ADSC);

  // ------ Watch dog init ---------------
  // Oct.29,2018: WDT has been disabled due to incompatibility with arduino
  // bootloader
  /*
    __asm__ __volatile__ ("wdr");
    MCUSR &= ~(1<<WDRF);
    WDTCSR|=(1<<WDCE) | (1<<WDE);
    WDTCSR =(1<<WDE) |(1<<WDP2)|(1<<WDP1); //1 sec delay for WDT
  */
  flags.stop = 0;
  flags.mode = 0;
  sei();
  char tmp_str[3];
  uint8_t *pData;

  while (V_batt && !flags.stop) // main loop
  {
    if (flags.mode)
      control_script();

#ifdef ADC_DEBUG
    if (flags.ADC_ready) {
      flags.ADC_ready = 0;
      pData = &ADC_data.ADC3;
      strncpy(&data_str[0], "   ;   ;   ", 11);
      for (int8_t i = 2; i >= 0; i--, pData++) {
        itoa(*pData, &tmp_str[0], 10);
        strncpy(&data_str[i << 2], tmp_str, strlen(tmp_str));
      }
      msg = data_str;
      UDR0 = *msg;
      msg++;
    }
#endif
  }

  // Shutting the system down
  cli();
  // Oct.29, 2018 - WDR is disabled
  /*
    __asm__ __volatile__ ("wdr");
    MCUSR &= ~(1<<WDRF);
    WDTCSR|=(1<<WDCE) | (1<<WDE);
    WDTCSR=0; // Watch-dog OFF
   */
  TCCR0A = 0;
  TCCR1A = 0;
  PORTB &= ~((1 << PB2) | (1 << PB1) | (1 << PB0));
  PORTD &= ~((1 << PD7) | (1 << PD6));
  ADCSRA &= ~((1 << ADEN) | (1 << ADIE));
  UCSR0B &= ~((1 << RXCIE0) | (1 << RXEN0));
  PORTB |= (1 << PB4);
  PORTD |= (1 << PD5);
  sei();
  time = 0;
  for (uint8_t aa = 0; aa < 6;) {
    if (time > 15) {
      PORTD ^= (1 << PD5);
      time = 0;
      aa++;
    }
  }
  cli();
  sleep_enable();
  sleep_cpu();
}
