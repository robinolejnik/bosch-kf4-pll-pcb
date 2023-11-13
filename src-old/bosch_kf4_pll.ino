#include "uart.h"
#include "util/delay.h"

// ##### Hardware #####
#define PTT_SENSE_DDR   DDRD
#define PTT_SENSE_PORT  PORTD
#define PTT_SENSE_PIN   PIND
#define PTT_SENSE_BIT   PD2

#define PTT_OUT_DDR     DDRC
#define PTT_OUT_PORT    PORTC
#define PTT_OUT_BIT     PC5

// ##### Config #####
#define UART_BAUD_RATE 38400

//#define F_MOD 16800000  // Simplex
//#define F_MOD 21400000  // (Semi-)Duplex SU
#define F_MOD 26000000  // (Semi-)Duplex SO

#define REPEATERTIMEOUT 30

#define F_UPDATE 50000
#define F_UPDATE_RPT 10

/*
prom      Platine AVR Arduino function
04 D0     -       PC1 A1
05 D1     -       PC0 A0
06 D2     -       PB5 13
07 D3     -       PB4 12
08 D4     -       PB0 8       ub/ob
09 D5     4       PD7 7
10 D6     3       PD6 6       über K22 nach St III /4 Leistungsumschaltung
11 D7     2       PD5 5       K19

17 A7     -       PD3 3       ptt sense
-


00  PD0 21  RXD
01  PD1 20  TXD
02  PD2 19  SQL
03  PD3 17  ptt sense
04  PD4 nc
05  PD5 11  D7
06  PD6 10  D6
07  PD7 09  D5
08  PB0 08  D4
09  PB1 -   PWM A
10  PB2 -   PWM B
11  PB3 nc
12  PB4 07  D3
13  PB5 06  D2
A0  PC0 05  D1
A1  PC1 04  D0
A2  PC2 03  A0
A3  PC3 02  A1
A4  PC4 01  A
A5  PC5 18  PTT

ddr
B 0011 0110
C 0010 0011
D 1110 00xx
*/

uint8_t SINE_TABLE[]= {
  128, 131, 134, 137, 140, 143, 146, 149, 152, 156, 159, 162, 165, 168, 171, 174,
  176, 179, 182, 185, 188, 191, 193, 196, 199, 201, 204, 206, 209, 211, 213, 216,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 236, 237, 239, 240, 242, 243, 245,
  246, 247, 248, 249, 250, 251, 252, 252, 253, 254, 254, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 254, 254, 253, 252, 252, 251, 250, 249, 248, 247,
  246, 245, 243, 242, 240, 239, 237, 236, 234, 232, 230, 228, 226, 224, 222, 220,
  218, 216, 213, 211, 209, 206, 204, 201, 199, 196, 193, 191, 188, 185, 182, 179,
  176, 174, 171, 168, 165, 162, 159, 156, 152, 149, 146, 143, 140, 137, 134, 131,
  127, 124, 121, 118, 115, 112, 109, 106, 103, 99, 96, 93, 90, 87, 84, 81,
  79, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39,
  37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10,
  9, 8, 7, 6, 5, 4, 3, 3, 2, 1, 1, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8,
  9, 10, 12, 13, 15, 16, 18, 19, 21, 23, 25, 27, 29, 31, 33, 35,
  37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76,
  79, 81, 84, 87, 90, 93, 96, 99, 103, 106, 109, 112, 115, 118, 121, 124
};
uint16_t phaseRegA, phaseIncA;
uint16_t phaseRegB, phaseIncB;

uint8_t data[8];

volatile uint32_t repeaterTimer;
uint8_t repeaterStatus;

void setrxfreq(const char* s) {
  uint32_t n = 0;
  while(*s!=0) {
    n *= 10;
    n += *s++ - 48;
  }
  n -= 21400000;
  if(n % 12500==0) {
    data[0] |= 0xA0;
    data[1] |= 0xA0;
    data[2] |= 0xA0;
    data[3] |= 0xA0;
    n /= 12500;
  }
  else if(n % 10000==0) {
    data[0] |= 0xB0;
    data[1] |= 0xB0;
    data[2] |= 0xB0;
    data[3] |= 0xB0;
    n /= 10000;
  }
  if(n>=25600) {
    data[2] |= 0x08;
    n-=25600;
  }
  if(n>=12800) {
    data[2] |= 0x04;
    n-=12800;
  }
  if(n>=6400) {
    data[2] |= 0x02;
    n-=6400;
  }
  if(n>=3200) {
    data[2] |= 0x01;
    n-=3200;
  }
  if(n>=1600) {
    data[3] |= 0x08;
    n-=1600;
  }
  if(n>=800) {
    data[3] |= 0x04;
    n-=800;
  }
  if(n>=400) {
    data[3] |= 0x02;
    n-=400;
  }
  if(n>=200) {
    data[3] |= 0x01;
    n-=200;
  }
  if(n>=100) {
    data[1] |= 0x08;
    n-=100;
  }
  if(n>=64) {
    data[1] |= 0x04;
    n-=64;
  }
  if(n>=32) {
    data[1] |= 0x02;
    n-=32;
  }
  if(n>=16) {
    data[1] |= 0x01;
    n-=16;
  }
  if(n>=8) {
    data[0] |= 0x08;
    n-=8;
  }
  if(n>=4) {
    data[0] |= 0x04;
    n-=4;
  }
  if(n>=2) {
    data[0] |= 0x02;
    n-=2;
  }
  if(n>=1) {
    data[0] |= 0x01;
    n-=1;
  }
}

void settxfreq(const char* s) {
  uint32_t n = 0;
  while(*s!=0) {
    n *= 10;
    n += *s++ - 48;
  }
  n -= F_MOD;          // 172780000 155980000
  if(n % 12500==0) {
    data[4] |= 0xA0;
    data[5] |= 0xA0;
    data[6] |= 0xA0;
    data[7] |= 0xA0;
    n /= 12500;
  }
  else if(n % 10000==0) {
    data[4] |= 0xB0;
    data[5] |= 0xB0;
    data[6] |= 0xB0;
    data[7] |= 0xB0;
    n /= 10000;
  }
  if(n>=25600) {
    data[6] |= 0x08;
    n-=25600;
  }
  if(n>=12800) {
    data[6] |= 0x04;
    n-=12800;
  }
  if(n>=6400) {
    data[6] |= 0x02;
    n-=6400;
  }
  if(n>=3200) {
    data[6] |= 0x01;
    n-=3200;
  }
  if(n>=1600) {
    data[7] |= 0x08;
    n-=1600;
  }
  if(n>=800) {
    data[7] |= 0x04;
    n-=800;
  }
  if(n>=400) {
    data[7] |= 0x02;
    n-=400;
  }
  if(n>=200) {
    data[7] |= 0x01;
    n-=200;
  }
  if(n>=100) {
    data[5] |= 0x08;
    n-=100;
  }
  if(n>=64) {
    data[5] |= 0x04;
    n-=64;
  }
  if(n>=32) {
    data[5] |= 0x02;
    n-=32;
  }
  if(n>=16) {
    data[5] |= 0x01;
    n-=16;
  }
  if(n>=8) {
    data[4] |= 0x08;
    n-=8;
  }
  if(n>=4) {
    data[4] |= 0x04;
    n-=4;
  }
  if(n>=2) {
    data[4] |= 0x02;
    n-=2;
  }
  if(n>=1) {
    data[4] |= 0x01;
    n-=1;
  }
  
}

int main(void) {
  uint8_t repeaterStatus = 0;

  DDRB |= (1<<PB5) | (1<<PB4) | (1<<PB2) | (1<<PB1);
  DDRC |= (1<<PC5) | (1<<PC1) | (1<<PC0);
  DDRD |= (1<<PD7) | (1<<PD6) | (1<<PD5);
  PTT_SENSE_PORT |= (1<<PTT_SENSE_BIT);

  TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
  TCCR1B = (1<<CS10);
  OCR1A = 0;
  OCR1B = 0;
/*
  TCCR2A = (1<<WGM21);
  TCCR2B = (1<<CS21);
  OCR2A = (F_CPU / 8 / F_UPDATE - 1);
  TIMSK2 = (1<<OCIE2A);
*/

  TCCR0A = (1<<WGM01);
  TCCR0B = (1<<CS02) | (1<<CS00);
  OCR0A = 255;//(F_CPU / 1024 / 256 - 1);
  TIMSK0 = (1<<OCIE0A);
  
  // enable pin change interrupt for address input
  //PCMSK1 = (1<<PCINT9) | (1<PCINT8);
  //PCICR = (1<<PCIE1);

  phaseIncA = 2000 * 65536 / F_UPDATE;
  phaseIncB = 500 * 65536 / F_UPDATE;

  uart_init();
  sei();
  uart_putc('S');

  setrxfreq("145737500");
  settxfreq("145137500");
  
  while(1) {
    if(!(PTT_SENSE_PIN & (1<<PTT_SENSE_BIT))) {    // senden
      PTT_OUT_PORT |= (1<<PTT_OUT_BIT);
      repeaterStatus = 1;
    }
    if(repeaterStatus==1 && (PTT_SENSE_PIN & (1<<PTT_SENSE_BIT))) {  // nachlauf
      repeaterStatus = 2;
      repeaterTimer = 0;
      uart_putc(2);
    }
    if(repeaterStatus==2 && repeaterTimer>=90) {
      PTT_OUT_PORT &= ~(1<<PTT_OUT_BIT);
      repeaterStatus = 0;
      uart_putc(0);
    }

    uint8_t address = ((PINC >> 2) & 0x03) | ((PIND >> 1) & 0x04);
    PORTB &= ((data[address] << 3) & (1<<PB5)) | ((data[address] << 1) & (1<<PB4)) | ((data[address] >> 4) & (1<<PB0)) | ~((1<<PB5) | (1<<PB4) | (1<<PB0));
    PORTB |= ((data[address] << 3) & (1<<PB5)) | ((data[address] << 1) & (1<<PB4)) | ((data[address] >> 4) & (1<<PB0));
    PORTC &= ((data[address] << 1) & (1<<PC1)) | ((data[address] >> 1) & (1<<PC0)) | ~((1<<PC1) | (1<<PC0));
    PORTC |= ((data[address] << 1) & (1<<PC1)) | ((data[address] >> 1) & (1<<PC0));
    PORTD &= ((data[address] << 2) & (1<<PD7)) |  (data[address]       & (1<<PD6)) | ((data[address] >> 2) & (1<<PD5)) | ~((1<<PD7) | (1<<PD6) | (1<<PD5));
    PORTD |= ((data[address] << 2) & (1<<PD7)) |  (data[address]       & (1<<PD6)) | ((data[address] >> 2) & (1<<PD5));
    
  }
  return 0;
}

ISR(TIMER0_COMPA_vect) {
  repeaterTimer++;
}

ISR(TIMER2_COMPA_vect) {
  /*
  uint8_t address = ((PINC >> 2) & 0x03) | ((PIND >> 1) & 0x04);
    PORTB &= ((data[address] << 3) & (1<<PB5)) | ((data[address] << 1) & (1<<PB4)) | ((data[address] >> 4) & (1<<PB0)) | ~((1<<PB5) | (1<<PB4) | (1<<PB0));
    PORTB |= ((data[address] << 3) & (1<<PB5)) | ((data[address] << 1) & (1<<PB4)) | ((data[address] >> 4) & (1<<PB0));
    PORTC &= ((data[address] << 1) & (1<<PC1)) | ((data[address] >> 1) & (1<<PC0)) | ~((1<<PC1) | (1<<PC0));
    PORTC |= ((data[address] << 1) & (1<<PC1)) | ((data[address] >> 1) & (1<<PC0));
    PORTD &= ((data[address] << 2) & (1<<PD7)) |  (data[address]       & (1<<PD6)) | ((data[address] >> 2) & (1<<PD5)) | ~((1<<PD7) | (1<<PD6) | (1<<PD5));
    PORTD |= ((data[address] << 2) & (1<<PD7)) |  (data[address]       & (1<<PD6)) | ((data[address] >> 2) & (1<<PD5));
    */
    /*
  phaseRegA += phaseIncA;
  phaseRegB += phaseIncB;
  OCR1A = SINE_TABLE[(uint8_t)(phaseRegA >> 8)];
  OCR1B = SINE_TABLE[(uint8_t)(phaseRegB >> 8)];
  //repeaterTimer++;
  */
}


ISR(PCINT1_vect) {
}


/*
PORTB xx23 xxx4
PORTC xxxx xx01
PORTD 567x xxxx

0 PC1 << 1
1 PC0 >> 1
2 PB5 << 3 x
3 PB4 << 1 x
4 PB0 >> 4 x
5 PD7 << 2
6 PD6 
7 PD5 >> 2
*/
