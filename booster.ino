
#include <LocoNet.h>
#include <Arduino.h>
#include <EEPROM.h>         //EEPROM lib
#include <U8x8lib.h>        // reference https://github.com/olikraus/u8g2/wiki/u8x8reference
#include <EEPROM.h>         //EEPROM lib

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#define LN_TX_PIN     6		// CPU.PD6
#define LN_RX_PIN     8   // CPU.PB0 (ICP)
#define A_PIN         4   //CPU.PD4  (encoder A)
#define B_PIN         3  //CPU.PD3  (encoder B)
#define KEY           A6   //CPU.ADC6  (encoder KEY)
#define DCC           2  //CPU.PD2  (DCC input)
#define FAN           10  //CPU.PB2  (FAN cooler output)
#define VRAIL         A7  //CPU.ADC7  (current flow 1)
#define CURR1         A2  //CPU.PC2  (current flow 1)
#define CURR2         A3  //CPU.PC3  (current flow 2)
#define TMP1          A0  //CPU.PC0  (temperature 1)
#define TMP2          A1  //CPU.PC1  (temperature 2)
#define INH1           5  //CPU.PD5  (inhibit driver 1)
#define INH2           7  //CPU.PD7  (inhibit driver 2)
#define SHORT          9  //CPU.PB1  (short propagation E)
#define LED1           0  //CPU.PD0  (inhibit driver 1)
#define LED2           1  //CPU.PD1  (inhibit driver 2)
#define SCL            A5  //CPU.PC5  (inhibit driver 1)
#define SDA            A4  //CPU.PC4  (inhibit driver 2)

#define sampdel       10  //delay sampling in usec
#define led_lim       5   //pwm led

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   

lnMsg  *LnPacket;          // pointer to a received LNet packet
static  LnBuf        LnTxBuffer;

lnMsg SendPacket;

unsigned char i,k;
signed int analog;
unsigned int j,appo;
volatile signed int temp1,temp2,curr1,curr2,volt;
unsigned long acctemp1,acctemp2,acccurr1,acccurr2,accvolt;
unsigned int zerocurr1, zerocurr2, watt;
volatile signed int fcurr1,fcurr2;
unsigned int displayco=0;
volatile unsigned char dccpres;
unsigned char adco=0;
unsigned char status1,status2;
unsigned char dataready;
unsigned char maxcurr1,maxcurr2,maxtemp1,maxtemp2, minv,maxv,ustop,fantempon,fantempoff;
unsigned char span1,span2;
volatile unsigned char overflag1,overflag2;
volatile unsigned int rearm1,rearm2;
unsigned int rearm1_lim, rearm2_lim;
volatile unsigned char overc1,overc2;
unsigned char overc1_lim, overc2_lim, acckey, pacckey, keypressed;
unsigned char led1on,led2on,pwmled,flash;
unsigned long savems;
unsigned char lpulse,lkeypressed,preskey;
unsigned int llpulse;
volatile unsigned char refresh,modeb;
unsigned char menup,ix;
unsigned char anow, aold, bnow, bold, increase, decrease, ax, bx;
unsigned char corr1,corr2;
unsigned char lnstop, changed;
unsigned char estop, estop_old, dccpres_old;
unsigned char starterr, stoperr;

unsigned char* dispa[4][16];
 
//                       0123456789012345  
unsigned char* menu[]= {"MAXCURR1 (A):   ",     //0 - 5.0 - 1.0-9.9
                        "MAXCURR2 (A):   ",     //1 - 5.0 - 1.0-9.9
                        "DELAY1  (ms):   ",     //2 - 10 - 1-99
                        "DELAY2  (ms):   ",     //3 - 10 - 1-99
                        "REARM1   (s):   ",     //4 - 1 - 1-9
                        "REARM2   (s):   ",     //5 - 1 - 1-9
                        "MAXTEMP1(°C):   ",     //6 - 90 - 30-99
                        "MAXTEMP2(°C):   ",     //7 - 90 - 30-99
                        "FANON  T(°C):   ",     //8 - 50 - 30-99
                        "FANOFF T(°C):   ",     //9 - 45 - 30-99
                        "MIN VSUP (V):   ",     //10 - 14 - 10-24
                        "MAX VSUP (V):   ",     //11 - 18 - 10-24
                        "CURRZERO1(A):   ",     //12 - 80 - 0 - 255
                        "CURRZERO2(A):   ",     //13 - 80 - 0 - 255                 
                        "CURRSPAN1(A):   ",     //14 - 4.0 - 1.0-9.9
                        "CURRSPAN2(A):   ",     //15 - 4.0 - 1.0-9.9
                        "CURRFACT1(%):   ",     //16 - 70 - 130 
                        "CURRFACT2(%):   "      //17 - 70 - 130
                                          };    




ISR(INT0_vect) {
//  PORTB |= 0x04;
  if (dccpres!=255) dccpres++;
  adco++;
  if (adco==7) adco=0;
  switch(adco) {
    case 0:
      analog = ADCL; 
      analog |= ADCH<<8; 
      accvolt = accvolt + analog;       
      ADMUX = 0x40;                                               // convert  with AVCC reference the channel 0 (TMP1)
      delayMicroseconds(sampdel);                             //no crossing noise
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    break;
    case 1:
      analog = ADCL; 
      analog |= ADCH<<8; 
      acctemp1 = acctemp1 + analog; 
      ADMUX = 0x41;                                               // convert with AVCC reference the channel 2 (TMP2)
      delayMicroseconds(sampdel);                             //no crossing noise
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    break;
    case 2:
      analog = ADCL; 
      analog |= ADCH<<8; 
      acctemp2 = acctemp2 + analog;
      ADMUX = 0x42;                                               // convert with AVCC reference the channel 3 (CURR1)
      delayMicroseconds(sampdel);                             //no crossing noise
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    break;
    case 3:
      analog = ADCL; 
      analog |= ADCH<<8;
      fcurr1=analog-zerocurr1;
      if (fcurr1<0) fcurr1=0; 
      fcurr1=fcurr1*corr1/128;
      acccurr1 = acccurr1 + analog;  
      ADMUX = 0x43;                                               // convert with AVCC reference the channel 4 (CURR2)
      delayMicroseconds(sampdel);                             //no crossing noise
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    break;
    case 4:
      analog = ADCL; 
      analog |= ADCH<<8; 
      fcurr2=analog-zerocurr2; 
      if (fcurr2<0) fcurr2=0; 
      fcurr2=fcurr2*corr2/128;
      acccurr2 = acccurr2 + analog;     
      ADMUX = 0x46;                                               // convert with AVCC reference the channel 6 (KEY)
      delayMicroseconds(sampdel);                             //no crossing noise
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    break;
    case 5:
      analog = ADCL; 
      analog |= ADCH<<8; 
      if (analog > 500) acckey=0; else acckey=1;     
      ADMUX = 0x47;                                               // convert with AVCC reference the channel 7 (VOLT)
      delayMicroseconds(sampdel);                             //no crossing noise
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    break;
    case 6:
      displayco++;
      if (displayco==512) {
        displayco=0;
        dataready=1;
        volt=accvolt/468;
        acctemp1=acctemp1-51200;
        temp1=acctemp1/1024;
        acctemp2=acctemp2-51200;
        temp2=acctemp2/1024;
        curr1=acccurr1/512;
        curr1=curr1-zerocurr1;
        if (curr1<0) curr1=0;
        curr1=curr1*corr1/128;
        curr1=curr1/2;
        curr2=acccurr2/512;
        curr2=curr2-zerocurr2;
        if (curr2<0) curr2=0;
        curr2=curr2*corr2/128;
        curr2=curr2/2;
        accvolt=0;
        acctemp1=0;
        acctemp2=0;
        acccurr1=0;
        acccurr2=0;
      }
    break;
  }
//  PORTB &= 0xFB;
}

ISR(TIMER2_COMPA_vect) {

  if (modeb==0) {                               //booster RUN
    if (fcurr1>maxcurr1) {
  //    PORTB |= 0x04;
      if (overc1!=0) {
        overc1--;
      } else {
        PORTD &= 0xDF;
        rearm1=rearm1_lim;
        overflag1=1;
      }
  //    PORTB &= 0xFB;
    } else {
      overc1=overc1_lim;
      if (rearm1!=0) {
        rearm1--;
        if (rearm1==0) {
          PORTD |= 0x20;
          overflag1=0;
        }
      }
    }

    if (fcurr2>maxcurr2) {
  //    PORTB |= 0x04;
      if (overc2!=0) {
        overc2--;
      } else {
        PORTD &= 0x7F;
        rearm2=rearm2_lim;
        overflag2=1;
      }
  //    PORTB &= 0xFB;
    } else {
      overc2=overc2_lim;
      if (rearm2!=0) {
        rearm2--;
        if (rearm2==0) {
          PORTD |= 0x80;
          overflag2=0;
        }
      }
    }  

  } else {                                                  //booster configurate

    if (PIND & 0x10) anow=1; else anow=0;
    if (PIND & 0x08) bnow=1; else bnow=0;
    if ((aold==0) && (anow==1)) {
      if (bnow==0) increase=1; else decrease=1;
    }
    if ((aold==1) && (anow==0)) {
      if (bnow==1) increase=1; else decrease=1;
    }
    aold=anow;

    preskey++;
    if (preskey==32) {                                        //every 32 ms
      preskey=0;
      ADMUX = 0x46;                                               // convert  with AVCC reference the channel 6 (KEY)
      ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
      while (ADCSRA & 0x40);
      analog = ADCL; 
      analog |= ADCH<<8; 
      if (analog>500) acckey=0; else acckey=1;            
      if ((acckey==1) && (pacckey==0)) keypressed=1;
      pacckey=acckey;
      if (acckey==1) {
        llpulse++;
        if (llpulse==50) {
          llpulse=0;
          modeb=0;
          refresh=1;      
        }
      } else {
         llpulse=0;   
      }  

    } 
  }

  pwmled++;
  if (pwmled==led_lim) pwmled=0;

  if (led1on==1) {
    if (pwmled==0) PORTD |= 0x01; else PORTD &= 0xFE;
  } else PORTD &= 0xFE;
    
  if (led2on==1) {
    if (pwmled==0) PORTD |= 0x02; else PORTD &= 0xFD;
  } else PORTD &= 0xFD;

}



void sample_all() {

  ADMUX = 0x40;                                               // convert  with AVCC reference the channel 0 (TMP1)
  ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
  while (ADCSRA & 0x40);
  analog = ADCL; 
  analog |= ADCH<<8; 
  acctemp1 = acctemp1 + analog;

  ADMUX = 0x41;                                               // convert  with AVCC reference the channel 1 (TMP2)
  ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
  while (ADCSRA & 0x40);
  analog = ADCL; 
  analog |= ADCH<<8; 
  acctemp2 = acctemp2 + analog;

  ADMUX = 0x46;                                               // convert  with AVCC reference the channel 6 (KEY)
  ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
  while (ADCSRA & 0x40);
  analog = ADCL; 
  analog |= ADCH<<8; 
  if (analog>500) acckey=0; else acckey=1;

  ADMUX = 0x47;                                               // convert  with AVCC reference the channel 7 (VOLT)
  ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
  while (ADCSRA & 0x40);
  analog = ADCL; 
  analog |= ADCH<<8; 
  accvolt = accvolt + analog;

  acccurr1=0;
  acccurr2=0;

  displayco++;
  if (displayco==512) {
    displayco=0;
    dataready=1;
    volt=accvolt/468;
    acctemp1=acctemp1-51200;
    temp1=acctemp1/1024;
    acctemp2=acctemp2-51200;
    temp2=acctemp2/1024;
    accvolt=0;
    acctemp1=0;
    acctemp2=0;
    acccurr1=0;
    acccurr2=0;
    curr1=0;
    fcurr1=0;
    curr2=0;
    fcurr2=0;
  }  

}

void LN_OFF() {
  SendPacket.data[ 0 ] = 0x82;  
  SendPacket.data[ 1 ] = 0x7D;  
  SendPacket.data[ 2 ] = 0xFF;  
  LocoNet.send( &SendPacket ) ;
}

void LN_ON() {
  SendPacket.data[ 0 ] = 0x83;  
  SendPacket.data[ 1 ] = 0x7C;  
  SendPacket.data[ 2 ] = 0xFF;  
  LocoNet.send( &SendPacket ) ;
}

void LN_IDLE() {
  SendPacket.data[ 0 ] = 0x85;  
  SendPacket.data[ 1 ] = 0x7A;  
  SendPacket.data[ 2 ] = 0xFF;  
  LocoNet.send( &SendPacket ) ;
}

void lnmanager() {
  LnPacket = LocoNet.receive() ;                        // Check for any received LocoNet packets
  if ( LnPacket ) {
    // First print out the packet in HEX
    uint8_t msgLen = getLnMsgSize(LnPacket);

    if (((LnPacket->data[0])==0x82) && (msgLen==2)) {      //IDLE request
      
    }

    if (((LnPacket->data[0])==0x83) && (msgLen==2)) {      //power ON request
      lnstop=0;
      stoperr=1;
    }

    if (((LnPacket->data[0])==0x85) && (msgLen==2)) {      //power OFF request
      lnstop=1;
      starterr=1;
    }

/*
    if (debug) {
      Serial.print("RX: ");
      for (uint8_t i = 0; i < msgLen; i++)
      {
        uint8_t val = LnPacket->data[i];
        // Print a leading 0 if less than 16 to make 2 HEX digits
        if (val < 16)
          Serial.print('0');
    
        Serial.print(val, HEX);
        if (i==msgLen-1) Serial.println(' '); else Serial.print(' ');
      }
    }
*/    
  }
}

void evaluate_limits() {

  if ((volt>maxv) && (estop==0)) {
    status1=2;
    status2=2;
    estop=1;
  }

  if ((volt<minv) && (estop==0)) {
    status1=5;
    status2=5;
    estop=1;
  }

  if ((overflag1) && (rearm1==0) && (estop==0)) {
    status1=3;
    estop=1;
  }  

  if ((overflag2) && (rearm2==0) && (estop==0)) {
    status2=3;
    estop=1;
  }  
  
  if ((temp1>maxtemp1) && (estop==0)) {
    status1=4;
    estop=1;
  }  

  if ((temp2>maxtemp2) && (estop==0)) {
    status2=4;
    estop=1;
  }  

  if ((estop==1) && (estop_old==0)) {
    starterr=1;
  }
  estop_old=estop;


  if ((dccpres==0) && (dccpres_old!=0)) {       //inizia a mancare il DCC in ingresso
    starterr=1;
    if (estop==0) {
      status1=1;
      status2=1;
    }
  } else if ((dccpres!=0) && (dccpres_old==0)) {    //ritorna il DCC in ingresso
    stoperr=1;
    estop=0;
    lnstop=0;
    status1=0;
    status2=0;
  }
  dccpres_old=dccpres; 

  if ((acckey==1) && (pacckey==0)) keypressed=1;
  pacckey=acckey;

  if (acckey==1) {
    lpulse++;
    if (lpulse==50) {
      lpulse=0;
      modeb=1;
      refresh=1;
    } 
  } else {
    lpulse=0;
  }

  if (keypressed) {
    keypressed=0;
    if (estop==1) {
      estop=0;
      lnstop=0;
      stoperr=1;
      status1=0;
      status2=0;
    } else {
      estop=1; 
      starterr=1;
      status1=6;
      status2=6;
    }
  } 

  if ((status1==0) && (status2==0)) {
    led1on=1;
    led2on=0;
    flash=0;
  } else if ((status1==6) && (status2==6)) {
    led1on=0;
    led2on=1;
    flash=0;
  } else if ((status1==3) || (status2==3)) {
    led1on=1;
    led2on=1;
    flash=0;
  } else if ((status1==1) && (status2==1)) {
    if ((millis()-savems)>600) {
      savems=millis();
      if (flash==0) {
        flash=1;
        led1on=1;
        led2on=0; 
      } else {
        flash=0;
        led1on=0;
        led2on=0;
      }
    }  
  } else {
    if ((millis()-savems)>600) {
      savems=millis();
      if (flash==0) {
        flash=1;
        led1on=0;
        led2on=1; 
      } else {
        flash=0;
        led1on=0;
        led2on=0;
      }
    }  
  }  

  if ((temp1>fantempon) || (temp2>fantempon)) {
    PORTB |= 0x04;                                                                //fan ON
  } else if ((temp1<fantempoff) && (temp2<fantempoff)) {
    PORTB &= 0xFB;                                                              //fan OFF
  }

  if (starterr==1) {
    starterr=0;
    if (status1!=0) {
      digitalWrite(INH1,0);
      digitalWrite(SHORT,1);
      LN_OFF();
    }
    if (status2!=0) {
      digitalWrite(INH2,0);
      digitalWrite(SHORT,1);
      LN_OFF();
    }
  } else {
    digitalWrite(SHORT,0);
  }

  if (stoperr==1) {
    stoperr=0;
    if (status1==0) {
      digitalWrite(INH1,1);
      LN_ON();
    }
    if (status2==0) {
      digitalWrite(INH2,1);
      LN_ON();
    }
  }

}

void argument(char a) {
  switch(a) {
    case 0:
      ax=EEPROM.read(0);
      u8x8.print(ax/10);
      u8x8.print(".");
      u8x8.print(ax%10);
    break;
    case 1:
      ax=EEPROM.read(1);
      u8x8.print(ax/10);
      u8x8.print(".");
      u8x8.print(ax%10);
    break;    
    case 2:
      ax=EEPROM.read(2);
      if (ax<10) u8x8.print(" ");
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 3:
      ax=EEPROM.read(3);
      if (ax<10) u8x8.print(" ");
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 4:
      ax=EEPROM.read(4);
      u8x8.print("  ");
      u8x8.print(ax);
    break;
    case 5:
      ax=EEPROM.read(5);
      u8x8.print("  ");
      u8x8.print(ax);
    break;
    case 6:
      ax=EEPROM.read(6);
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 7:
      ax=EEPROM.read(7);
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 8:
      ax=EEPROM.read(8);
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 9:
      ax=EEPROM.read(9);
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 10:
      ax=EEPROM.read(10);
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 11:
      ax=EEPROM.read(11);
      u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 12:
      ax=EEPROM.read(12);
      if (ax<10) u8x8.print(" ");
      if (ax<100) u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 13:
      ax=EEPROM.read(13);
      if (ax<10) u8x8.print(" ");
      if (ax<100) u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 14:
      ax=EEPROM.read(14);
      u8x8.print(ax/10);
      u8x8.print(".");
      u8x8.print(ax%10);
    break;
    case 15:
      ax=EEPROM.read(15);
      u8x8.print(ax/10);
      u8x8.print(".");
      u8x8.print(ax%10);
    break;        
    case 16:
      ax=EEPROM.read(16);
      ax=ax*100/128;
      if (ax<100) u8x8.print(" ");
      u8x8.print(ax);
    break;
    case 17:
      ax=EEPROM.read(17);
      ax=ax*100/128;
      if (ax<100) u8x8.print(" ");
      u8x8.print(ax);
    break;        
  }
}
  

void incre(char a) {
  switch(a) {
    case 0:
      ax=EEPROM.read(0);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(0,ax);
      maxcurr1=ax*2;
    break;
    case 1:
      ax=EEPROM.read(1);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(1,ax);
      maxcurr2=ax*2;
    break;
    case 2:
      ax=EEPROM.read(2);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(2,ax);
      overc1_lim=ax;
    break;
    case 3:
      ax=EEPROM.read(3);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(3,ax);
      overc2_lim=ax;
    break;
    case 4:
      ax=EEPROM.read(4);
      ax++;
      if (ax>9) ax=9;
      EEPROM.write(4,ax);
      rearm1_lim=ax*1000;
    break;
    case 5:
      ax=EEPROM.read(5);
      ax++;
      if (ax>9) ax=9;
      EEPROM.write(5,ax);
      rearm2_lim=ax*1000;
    break;
    case 6:
      ax=EEPROM.read(6);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(6,ax);
      maxtemp1=ax;
    break;
    case 7:
      ax=EEPROM.read(7);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(7,ax);
      maxtemp2=ax;
    break;
    case 8:
      ax=EEPROM.read(8);
      ax++;
      if (ax>80) ax=99;
      EEPROM.write(8,ax);
      fantempon=ax;
    break;
    case 9:
      bx=EEPROM.read(8);
      ax=EEPROM.read(9);
      ax++;
      if (ax>bx) ax=bx;
      EEPROM.write(9,ax);
      fantempoff=ax;
    break;
    case 10:
      ax=EEPROM.read(10);
      ax++;
      if (ax>24) ax=24;
      EEPROM.write(10,ax);
      minv=ax*10;
    break;
    case 11:
      ax=EEPROM.read(11);
      ax++;
      if (ax>24) ax=24;
      EEPROM.write(11,ax);
      maxv=ax*10;
    break;
    case 14:
      ax=EEPROM.read(14);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(14,ax);
      span1=ax*10;
    break;    
    case 15:
      ax=EEPROM.read(15);
      ax++;
      if (ax>99) ax=99;
      EEPROM.write(15,ax);
      span2=ax*10;
    break;    
  }
}  

void decre(char a) {
  switch(a) {
    case 0:
      ax=EEPROM.read(0);
      ax--;
      if (ax<10) ax=10;
      EEPROM.write(0,ax);
      maxcurr1=ax*2;
    break;
    case 1:
      ax=EEPROM.read(1);
      ax--;
      if (ax<10) ax=10;
      EEPROM.write(1,ax);
      maxcurr2=ax*2;
    break;
    case 2:
      ax=EEPROM.read(2);
      ax--;
      if (ax<1) ax=1;
      EEPROM.write(2,ax);
      overc1_lim=ax;
    break;
    case 3:
      ax=EEPROM.read(3);
      ax--;
      if (ax<1) ax=1;
      EEPROM.write(3,ax);
      overc2_lim=ax;
    break;
    case 4:
      ax=EEPROM.read(4);
      ax--;
      if (ax==255) ax=0;
      EEPROM.write(4,ax);
      rearm1_lim=ax*1000;
    break;
    case 5:
      ax=EEPROM.read(5);
      ax--;
      if (ax==255) ax=0;
      EEPROM.write(5,ax);
      rearm2_lim=ax*1000;
    break;
    case 6:
      ax=EEPROM.read(6);
      ax--;
      if (ax<30) ax=30;
      EEPROM.write(6,ax);
      maxtemp1=ax;
    break;
    case 7:
      ax=EEPROM.read(7);
      ax--;
      if (ax<30) ax=30;
      EEPROM.write(7,ax);
      maxtemp2=ax;
    break;
    case 8:
      ax=EEPROM.read(8);
      ax--;
      if (ax<30) ax=30;
      EEPROM.write(8,ax);
      fantempon=ax;
    break;
    case 9:
      ax=EEPROM.read(9);
      ax--;
      if (ax<30) ax=30;
      EEPROM.write(9,ax);
      fantempoff=ax;
    break;
    case 10:
      ax=EEPROM.read(10);
      ax--;
      if (ax<10) ax=10;
      EEPROM.write(10,ax);
      minv=ax*10;
    break;
    case 11:
      ax=EEPROM.read(11);
      ax--;
      if (ax<10) ax=10;
      EEPROM.write(11,ax);
      maxv=ax*10;
    break;
    case 14:
      ax=EEPROM.read(14);
      ax--;
      if (ax<10) ax=10;
      EEPROM.write(14,ax);
      span1=ax*2;
    break;
    case 15:
      ax=EEPROM.read(15);
      ax--;
      if (ax<10) ax=10;
      EEPROM.write(15,ax);
      span2=ax*2;
    break;        
  }
}  

void drawmenu(char a) {
  ix=a;
  u8x8.setCursor(0, 0);
  u8x8.setInverseFont(1);
  u8x8.drawUTF8(0, 0, menu[ix]);
  u8x8.setCursor(13, 0);
  argument(ix);
  u8x8.setInverseFont(0);  
  ix++;
  if (ix>17) ix=0;
  u8x8.drawUTF8(0, 2, menu[ix]);
  u8x8.setCursor(13, 2);
  argument(ix);
  ix++;
  if (ix>17) ix=0;
  u8x8.drawUTF8(0, 4, menu[ix]);
  u8x8.setCursor(13, 4);
  argument(ix);
  ix++;
  if (ix>17) ix=0;
  u8x8.drawUTF8(0, 6, menu[ix]);
  u8x8.setCursor(13, 6);
  argument(ix);
}


void getzero1() {
  cli();  
  digitalWrite(INH1,1);         //on DCC output 1
  delay(10);  
  acccurr1=0;
  ADMUX = 0x42;                                          // convert  with AVCC reference the channel 2 (CURR1)
  delayMicroseconds(10);
  for (j=0;j<8192;j++) {
//    while (PIND2 & 0x04);
//    while (!(PIND2 & 0x04));
//    delayMicroseconds(sampdel);
    ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    while (ADCSRA & 0x40);
    analog = ADCL; 
    analog |= ADCH<<8; 
    acccurr1 = acccurr1 + analog;
  }
  acccurr1=acccurr1/8192;
  zerocurr1=acccurr1;
  if (zerocurr1>200) zerocurr1=200;
  EEPROM.write(12,zerocurr1);
  digitalWrite(INH1,0);         //off DCC output 1  
  sei();  
}

void getzero2() {
  cli();  
  digitalWrite(INH2,1);         //on DCC output 2
  delay(10);  
  acccurr2=0;
  ADMUX = 0x43;                                          // convert  with AVCC reference the channel 3 (CURR2)
  delayMicroseconds(10);
  for (j=0;j<8192;j++) {
//    while (PIND2 & 0x04);
//    while (!(PIND2 & 0x04));
//    delayMicroseconds(sampdel);
    ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    while (ADCSRA & 0x40);
    analog = ADCL; 
    analog |= ADCH<<8; 
    acccurr2 = acccurr2 + analog;
  }
  acccurr2=acccurr2/8192;
  zerocurr2=acccurr2;
  if (zerocurr2>200) zerocurr2=200;
  EEPROM.write(13,zerocurr2);
  digitalWrite(INH2,0);         //off DCC output 2  
  sei();  
}

void getspan1() {
  cli();  
  digitalWrite(INH1,1);         //on DCC output 1
  delay(10);  
  acccurr1=0;
  ADMUX = 0x42;                                          // convert  with AVCC reference the channel 2 (CURR1)
  delayMicroseconds(10);
  for (j=0;j<8192;j++) {
    while ((PIND & 0x04)!=0);
    while ((PIND & 0x04)==0);
//    PORTB |= 0x04;
    delayMicroseconds(sampdel);
//    PORTB &= 0xFB;
    ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    while (ADCSRA & 0x40);
    analog = ADCL; 
    analog |= ADCH<<8; 
    acccurr1 = acccurr1 + analog;
  }
  curr1=acccurr1/8192;
  curr1=curr1-zerocurr1;
  if (curr1<0) curr1=0;
  curr1=curr1/2;
  appo=span1*128/curr1;
  corr1=appo;
  if (corr1>168) corr1=168;
  if (corr1<88) corr1=88;
  EEPROM.write(16,corr1);
  digitalWrite(INH1,0);         //off DCC output 1  
  sei();  
}

void getspan2() {
  cli();  
  digitalWrite(INH2,1);         //on DCC output 2
  delay(10);  
  acccurr2=0;
  ADMUX = 0x43;                                          // convert  with AVCC reference the channel 3 (CURR2)
  delayMicroseconds(10);
  for (j=0;j<8192;j++) {
    while ((PIND & 0x04)!=0);
    while ((PIND & 0x04)==0);
    delayMicroseconds(sampdel);
    ADCSRA |= 0x40;                                        // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
    while (ADCSRA & 0x40);
    analog = ADCL; 
    analog |= ADCH<<8; 
    acccurr2 = acccurr2 + analog;
  }
  curr2=acccurr2/8192;
  curr2=curr2-zerocurr2;
  if (curr2<0) curr2=0;
  curr2=curr2/2;
  appo=span2*128/curr2;
  corr2=appo;
  if (corr2>168) corr2=168;
  if (corr2<88) corr2=88;
  EEPROM.write(17,corr2);
  digitalWrite(INH2,0);         //off DCC output 2  
  sei();  
}

void setup() {
  
  pinMode(LN_TX_PIN,OUTPUT);        // this is for the LN TX
  pinMode(LN_RX_PIN,INPUT);        // this is for the LN RX
  pinMode(DCC,INPUT);               // this is for the DCC Signal
  pinMode(A_PIN, INPUT_PULLUP);   //encoder A
  pinMode(B_PIN, INPUT_PULLUP);   //encoder B
  pinMode(KEY, INPUT_PULLUP);     //enecoder KEY
  pinMode(FAN,OUTPUT);       // this is for the FAN cooler
  pinMode(VRAIL,INPUT);       // track voltage
  pinMode(CURR1,INPUT);       // 
  pinMode(CURR2,INPUT);       // 
  pinMode(TMP1,INPUT);       // 
  pinMode(TMP2,INPUT);       // 
  pinMode(LED1,OUTPUT);       // 
  pinMode(LED2,OUTPUT);       // 
  pinMode(SHORT,OUTPUT);       //
  pinMode(INH1,OUTPUT);       // 
  pinMode(INH2,OUTPUT);       // 

  digitalWrite(FAN,0);       //set FAN OFF
  digitalWrite(INH1,0);         //off DCC output 1
  digitalWrite(INH2,0);        //off DCC output 2
  digitalWrite(SHORT,0);        //CDE short inactive
  digitalWrite(LED1,0);        //LED 1 OFF
  digitalWrite(LED2,0);        //LED 2 OFF
  
  LocoNet.init(LN_TX_PIN); 

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_7x14_1x2_f);
  u8x8.clear();

  ADCSRA=0x86;            //ADC ON; 250Khz clock
  EICRA=0x03;             //INT0 interrupt on rising edge
  EIMSK=0x01;             //enabled INT0

  TCCR2A=0x02;            //Clear timer on compare match
  OCR2A=125;               //16000/128=125 for 1ms
  TCCR2B=0x85;            //compare A, clock/128 source
  TIMSK2=0x02;            //enabled timer2 match A


/*  zerocurr1=76;     //ZERO CURR OUT1
  zerocurr2=95;     //ZERO CURR OUT2
  maxcurr1=50;      //MAX CURR OUT1 la corrente di soglia va moltiplicata per 2
  maxcurr2=50;      //MAX CURR OUT2 
  overc1_lim=10;    //DELAY ms for OVERC. OUT1
  overc2_lim=10;    //DELAY ms for OVERC. OUT2
  rearm1_lim=1000;  //DELAY ms for rearm OUT1
  rearm2_lim=1000;  //DELAY ms for rearm OUT2
  maxtemp1=90;      //MAX TEMP OUT1 
  maxtemp2=90;      //MAX TEMP OUT2 
  fantempon=45;     //FAN ON temperature
  fantempoff=40;    //FAN OFF temperature
  minv=140;         //MIN power supply
  maxv=240;         //MAX power supply
  span1=40;
  span2=40;
  corr1=128;
  corr2=128;
*/

//===============================================================================================================
// initialize EEPROM if EEPROM erased or key pressed at startup 
//===============================================================================================================

  ADMUX = 0x46;                                               // convert  with AVCC reference the channel 6 (KEY)
  ADCSRA |= 0x40;                                             // Convert while (bit_is_set(ADCSRA,ADSC)); //soc
  while (ADCSRA & 0x40);
  analog = ADCL; 
  analog |= ADCH<<8; 
  if ((analog<500) || (EEPROM.read(0)==255)) {
    EEPROM.write(0,25);
    EEPROM.write(1,25);
    EEPROM.write(2,10);
    EEPROM.write(3,10);
    EEPROM.write(4,1);
    EEPROM.write(5,1);
    EEPROM.write(6,90);
    EEPROM.write(7,90);
    EEPROM.write(8,45);
    EEPROM.write(9,40);
    EEPROM.write(10,14);
    EEPROM.write(11,18);
    EEPROM.write(12,80);
    EEPROM.write(13,80);
    EEPROM.write(14,20);
    EEPROM.write(15,20);
    EEPROM.write(16,128);
    EEPROM.write(17,128);

    u8x8.drawUTF8(0, 0, "  Booster init  "); 
    delay(2000);
    u8x8.clear();
  } 

//===============================================================================================================
// initialize control variables from EEPROM 
//===============================================================================================================

  maxcurr1=EEPROM.read(0)*2;      //MAX CURR OUT1
  maxcurr2=EEPROM.read(1)*2;      //MAX CURR OUT2 
  overc1_lim=EEPROM.read(2);    //DELAY ms for OVERC. OUT1
  overc2_lim=EEPROM.read(3);    //DELAY ms for OVERC. OUT2
  rearm1_lim=EEPROM.read(4)*1000;  //DELAY s for rearm OUT1
  rearm2_lim=EEPROM.read(5)*1000;  //DELAY s for rearm OUT2
  maxtemp1=EEPROM.read(6);      //MAX TEMP OUT1 
  maxtemp2=EEPROM.read(7);      //MAX TEMP OUT2 
  fantempon=EEPROM.read(8);     //FAN ON temperature
  fantempoff=EEPROM.read(9);    //FAN OFF temperature
  minv=EEPROM.read(10)*10;         //MIN power supply
  maxv=EEPROM.read(11)*10;         //MAX power supply
  zerocurr1=EEPROM.read(12);         //zero OUT1
  zerocurr1=77;                       //TO BE REMOVED
  zerocurr2=EEPROM.read(13);         //zero OUT2
  zerocurr2=97;                       //TO BE REMOVED
  span1=EEPROM.read(14);         //span current OUT1
  span2=EEPROM.read(15);         //span current OUT2
  corr1=EEPROM.read(16);            //correction value for OUT1
  corr2=EEPROM.read(17);            //correction value for OUT2

//===============================================================================================================
// initialize generic variables 
//===============================================================================================================

  ustop=0;
  dccpres=0;
  status1=0;
  status2=0;
  rearm1=0;
  rearm2=0;
  overc1=0;
  overc2=0;
  overflag1=0;
  overflag2=0;
  acckey=0;
  pacckey=0;
  ustop=0;
  led1on=0;
  led2on=0;
  pwmled=0;
  savems=0;
  flash=0;
  keypressed=0;
  lpulse=0;
  lkeypressed=0;
  modeb=0;
  preskey=0;
  llpulse=0;
  refresh=1;
  menup=0;
  increase=0;
  decrease=0;

}

void loop() {

  if (modeb==0) {
    EIMSK=0x01;             //enabled INT0  
    lnmanager();

    if ((refresh==1) && (modeb==0)) {
      refresh=0;
      u8x8.clear();
      //char pos           0123456789012345
      u8x8.drawUTF8(0, 0, "I1: . A  I2: . A");  
      u8x8.drawUTF8(0, 2, "T1:  °C  T2:  °C");  
      u8x8.drawUTF8(0, 4, "S1:      S2:    ");
      u8x8.drawUTF8(0, 6, "V:  . V   P:   W");        
    }

    while (dataready==0) {
      if (dccpres==0) {
        dccpres=0;
        for(i=0;i<32; i++) sample_all();
        delay(8);
        lnmanager();
        delay(8);
        lnmanager();
        delay(8);
        lnmanager();
        evaluate_limits();
      } else {
        dccpres=0;
        delay(10);
        lnmanager();
        delay(10);
        lnmanager();
        delay(10);
        lnmanager();
        evaluate_limits();
      }
    }

    if (dataready) {
      dataready=0;
//      PORTB |= 0x04;
      appo=curr1 / 10;
      u8x8.setCursor(3, 0);
      u8x8.print(appo);
      appo=curr1 % 10;
      u8x8.setCursor(5, 0);
      u8x8.print(appo);

      appo=curr2 / 10;
      u8x8.setCursor(12, 0);
      u8x8.print(appo);
      appo=curr2 % 10;
      u8x8.setCursor(14, 0);
      u8x8.print(appo);

      lnmanager();

      appo=temp1 / 10;
      u8x8.setCursor(3, 2);
      u8x8.print(appo);
      appo=temp1 % 10;
      u8x8.setCursor(4, 2);
      u8x8.print(appo);

      appo=temp2 / 10;
      u8x8.setCursor(12, 2);
      u8x8.print(appo);
      appo=temp2 % 10;
      u8x8.setCursor(13, 2);
      u8x8.print(appo);

      lnmanager();

      switch (status1) {
        case 0:  
          u8x8.setCursor(3, 4);
          u8x8.print("RUN ");
        break;
        case 1:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(3, 4);
          u8x8.print("NDCC");
          u8x8.setInverseFont(0);        
        break;
        case 2:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(3, 4);
          u8x8.print("OV.V");
          u8x8.setInverseFont(0);        
        break; 
        case 3:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(3, 4);
          u8x8.print("OV.C");
          u8x8.setInverseFont(0);        
        break; 
        case 4:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(3, 4);
          u8x8.print("OV.T");
          u8x8.setInverseFont(0);        
        break; 
        case 5:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(3, 4);
          u8x8.print("UN.V");
          u8x8.setInverseFont(0);        
        break; 
        case 6:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(3, 4);
          u8x8.print("STOP");
          u8x8.setInverseFont(0);        
        break; 
      }

      lnmanager();

      switch (status2) {
        case 0:  
          u8x8.setCursor(12, 4);
          u8x8.print("RUN ");
        break;
        case 1:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(12, 4);
          u8x8.print("NDCC");
          u8x8.setInverseFont(0);        
        break;
        case 2:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(12, 4);
          u8x8.print("OV.V");
          u8x8.setInverseFont(0);        
        break; 
        case 3:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(12, 4);
          u8x8.print("OV.C");
          u8x8.setInverseFont(0);        
        break; 
        case 4:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(12, 4);
          u8x8.print("OV.T");
          u8x8.setInverseFont(0);        
        break; 
        case 5:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(12, 4);
          u8x8.print("UN.V");
          u8x8.setInverseFont(0);        
        break; 
        case 6:  
          u8x8.setInverseFont(1);        
          u8x8.setCursor(12, 4);
          u8x8.print("STOP");
          u8x8.setInverseFont(0);        
        break; 
      }

      lnmanager();

      appo=volt / 100;
      u8x8.setCursor(2, 6);
      if (appo==0) u8x8.print(" "); else u8x8.print(appo);
      appo=(volt-(appo*100)) / 10;
      u8x8.setCursor(3, 6);
      u8x8.print(appo);
      appo=volt % 10;
      u8x8.setCursor(5, 6);
      u8x8.print(appo);

/*      
      u8x8.setCursor(12, 6);
      u8x8.print("    ");
      u8x8.setCursor(12, 6);
      u8x8.print(acckey);
*/
      watt=volt*(curr1+curr2)/100;
      if (watt>99) {
        u8x8.setCursor(12, 6);
        u8x8.print(watt);
      } else if (watt>9) {
        u8x8.setCursor(12, 6);
        u8x8.print(" ");
        u8x8.print(watt);
      } else {
        u8x8.setCursor(12, 6);
        u8x8.print("  ");
        u8x8.print(watt);
      }
//      PORTB &= 0xFB;
    }      
  } else if (modeb==1) {

    EIMSK=0x00;                   //disabled INT0
    digitalWrite(INH1,0);         //off DCC output 1
    digitalWrite(INH2,0);         //off DCC output 2
    led1on=0;                     //led 1 off
    led2on=0;                     //led 2 off

    if ((refresh==1) && (modeb==1)) {
      refresh=0;
      u8x8.clear();
      drawmenu(menup); 
    }

    if (increase==1) {
      increase=0;
      menup++;
      if (menup==18) menup=0;
      drawmenu(menup);
    }

    if (decrease==1) {
      decrease=0;
      menup--;
      if (menup==255) menup=17;
      drawmenu(menup);
    }

    if (keypressed==1) {
      keypressed=0;
      if (menup<16) {
        refresh=1;
        modeb=2;
      }
    }

  } else {

    if ((refresh==1) && (modeb==2)) {
      refresh=0;
      u8x8.setInverseFont(0);
      u8x8.drawUTF8(0, 0, menu[menup]);
      u8x8.setCursor(13, 0);
      u8x8.setInverseFont(1);
      argument(menup);
      u8x8.setInverseFont(0);  
    }

    if (increase==1) {
      increase=0;
      incre(menup);
      u8x8.setCursor(13, 0);
      u8x8.setInverseFont(1);
      argument(menup);
      u8x8.setInverseFont(0);  
    }

    if (decrease==1) {
      decrease=0;
      decre(menup);
      u8x8.setCursor(13, 0);
      u8x8.setInverseFont(1);
      argument(menup);
      u8x8.setInverseFont(0);  
    }

    if (keypressed==1) {
      keypressed=0;
      modeb=1;
      refresh=1;
      if (menup==12) {
        getzero1();
      }
      if (menup==13) {
        getzero2();
      }
      if (menup==14) {
        getspan1();
      }
      if (menup==15) {
        getspan2();
      }
    }

  }

}
