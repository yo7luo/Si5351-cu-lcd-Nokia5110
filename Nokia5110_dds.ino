#include <si5351.h>

#include <Rotary.h>

//////////////////////////////////////////////////////////////////////
//  si5351 VFO program ver.1.0
//    Copyright(C)2016.JA2GQP.All rights reserved.
//
//                                Arduino IDE 1.6.9 Compiled                                   
//
//                                                2016/5/30
//                                                  JA2GQP
//--------------------------------------------------------------------
//  Function
//    1.STEP(1M,100k,10k,1k,100,10)
//    2.Memory Channel ch0 - ch9(10ch)
//    3.Protection Operation At The Time Of Transmission
//--------------------------------------------------------------------
// Library
//  Rotary encoeder  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
//  si5351Arduino    https://github.com/etherkit/Si5351Arduino
//  LCD5110_Basic    http://www.rinkydinkelectronics.com/library.php?id=44
//////////////////////////////////////////////////////////////////////

#include <rotary.h>
#include <si5351.h>
#include <Wire.h>
#include <LCD5110_Basic.h>
#include <EEPROM.h>

//----------  Define Constant Value   -----------------------

////////////////////////////////
// I/O assign
////////////////////////////////

const byte  ENC_A = 2;                     // Encorder A
const byte  ENC_B = 3;                     // Encoeder B
const byte  SW_STEP = 4;                   // STEP SW
const byte  SW_RIT = 5;                    // RIT SW
const byte  SW_CH = 6;                     // CH SW
const byte  SW_TX = 15;                    // TX SW

const byte  AD_IN = A0;                    // analog input for left channel

////////////////////////////////
// default value
////////////////////////////////

const long  DEF_FRQ = 7100000L;            // Default Vfo(7.05MHz)
const long  DEF_STP = 1000L;               // Init STEP(1kHz)

////////////////////////////////
// Limited range
////////////////////////////////

const long  LW_FRQ = 0L;                   // Frequency Lower Limit
const long  HI_FRQ = 60000000L;            //           Upper Limit

const long  LW_RIT = -10000L;              // RIT Lower Limit
const long  HI_RIT = 10000L;               // RIT Upper Limit

const long  LW_VFO80 = 3500000L;           // 3.5MHz Lower
const long  MI_VFO80 = 3535000L;           //        Middle
const long  HI_VFO80 = 3575000L;           //        Upper
const long  LW_VFO40 = 7000000L;           // 7MHz   Lower
const long  MI_VFO40 = 7045000L;           //        Middle
const long  HI_VFO40 = 7200000L;           //        Upper
const long  LW_VFO20 = 14000000L;          // 14MHz  Lower
const long  MI_VFO20 = 14100000L;          //        Middle
const long  HI_VFO20 = 14350000L;          //        Upper
const long  LW_VFO15 = 21000000L;          // 21MHz  Lower
const long  MI_VFO15 = 21150000L;          //        Middle
const long  HI_VFO15 = 21450000L;          //        Upper
const long  LW_VFO10 = 28000000L;          // 28MHz  Lower
const long  MI_VFO10 = 28200000L;          //        Middle
const long  HI_VFO10 = 29000000L;          //        Upper

////////////////////////////////
// etc
////////////////////////////////

const byte  Max_Chn = 10;                  // Max Channel(1-10ch)
const byte  Int_End = 73;                  // Initial end code

//----------  EEPROM Memory Address   -----------------------

const byte  Frq_Eep = 0x00;                // Frequency(4byte*10)
const byte  Stp_Eep = 0x30;                // STEP(4byte*10)
const byte  Chn_Eep = 0x60;                // Channel(1byte*1)
const byte  Eep_Int = 0x6e;                // Eep Init(1byte*1)

//----------  LCD NOKIA 5110 definition --------------------

LCD5110 myGLCD(8,9,10,11,12);             // SCK,MOSI,DC,RST,CS
extern uint8_t SmallFont[];
extern uint8_t MediumNumbers[];
extern uint8_t BigNumbers[];

//----------  si5351 definition -----------------------------

Si5351 si5351;

//----------  Encorder Pin definition  ----------------------

Rotary r = Rotary(ENC_A, ENC_B);

//----------  Memory Assign  --------------------------------

volatile long LSB = 9998500L;              // 10.7015MHz
volatile long USB = 10001500L;              // 10.6985MHz
volatile long CW  = 10000000L;              // 10.7006MHz
volatile long bfo = 9998500L;              // start in LSB
volatile long IF  = 10000000L;              // 10.7000MHz

volatile long Vfo_Dat = DEF_FRQ;            // Default Frequency
volatile long Vfo_Datb;                     // Vfo data(old)
volatile long Lng_Wk1;                      // Long Work1
volatile long Lng_Wk2;                      // Long Work2
volatile long Dds_Dat;
String tbfo = "";

volatile long Rit_Dat = 0;                 // RIT Data
volatile long Rit_Datb = 0;
volatile long Enc_Stp = 1000;              // STEP

byte Flg_Tx = 0;                          // TX Flag
byte Flg_Rit = 0;                         // RIT Flag
byte Flg_Spl;                             // SPLIT Flag
byte Flg_Over;                            // Over Flag
byte Byt_Chn = 0;                         // Channel SW
byte Byt_Chnb = 0;                        // Channel SW Old

//----------  Initialization  Program  ----------------------

void setup(){
  myGLCD.InitLCD();                              // nokia5110 Init  
  Wire.begin();

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0);       //initialize the Si5351
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA); // Set PLL
                                                 // Set CLK0 Frequency(VFO)
  pinMode(SW_STEP,INPUT_PULLUP);
  pinMode(SW_RIT,INPUT_PULLUP);
  pinMode(SW_CH,INPUT_PULLUP);
  pinMode(SW_TX,INPUT_PULLUP);
 
  PCICR |= (1 << PCIE2);                        // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();                                        // INT Enable
  Rit_Dat = 0;
  Flg_Rit = 0;
 
  if(EEPROM.read(Eep_Int) != Int_End){         // Eep initialaz
    delay(10);
    Fnc_Eep_Int();
  }

  Byt_Chn = EEPROM.read(Chn_Eep);              // Eep Channel Read
  Byt_Chnb = Byt_Chn;
  Fnc_Eep_Rd();
  Fnc_Lcd();                                   // Display LCD
  Fnc_Step_Disp();
}

//----------  Main program  ---------------------------------

void loop(){
  if(Flg_Tx == 0){
    Fnc_Smeter();
   
    if(digitalRead(SW_STEP) == LOW)               // STEP Sw On?
      Fnc_Stp();                           
    if((digitalRead(SW_RIT) == LOW))              // RIT SW On?
      Fnc_Rit();
    if((digitalRead(SW_CH) == LOW))               // CH SW On?
      Fnc_Chsw();

    if(Byt_Chnb != Byt_Chn){                      // CH SW OLD != NEW?
      Fnc_Eep_Wt(Byt_Chnb);
      Byt_Chnb = Byt_Chn;
      Fnc_Eep_Rd();
    }
  }

  if(digitalRead(SW_TX) == LOW)                  // Tx On?
    Flg_Tx = 1;
  else
    Flg_Tx = 0;

  Fnc_Step_Disp();
  Fnc_Lcd();

  if((Flg_Tx == 0) && (Flg_Rit == 1))
      si5351.set_freq(((Vfo_Dat+IF+Rit_Dat) * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
  if((Flg_Tx == 1) || (Flg_Rit == 0))
     si5351.set_freq(((Vfo_Dat+IF) * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);

  Fnc_Band(Vfo_Dat);
   
  if(Flg_Over == 1)                             // Over?
    myGLCD.print("over",CENTER,16);             // Display(over)
  else   
    myGLCD.clrRow(2);                           // Clear(over)

  if((Flg_Over == 1) && (Flg_Tx == 1))
    si5351.output_enable(SI5351_CLK0, 0);       // VFO disable
  else
     si5351.output_enable(SI5351_CLK0, 1);      // VFO enable

  si5351.set_freq(( bfo* SI5351_FREQ_MULT), 0, SI5351_CLK2);
}

//----------  Function S-Meter Display  -----------------------

void Fnc_Smeter(){
  int ad1 = 0;
  int ad2 = 0;
 
  for(int i=0;i<6;i++){
    ad1 = map(sqrt( analogRead( AD_IN  )*20 ),0,143,0,140);
    ad2 = ad1 + ad2;
  }
  ad2 = ad2 / 5;
  int s_dat = map(ad2,0,140,0,14);

  for (int i=0; i<s_dat; i++)         
    myGLCD.print("\\",(i*6), 40);
  myGLCD.clrRow(5,(s_dat*6));

  if(s_dat < 1)
    myGLCD.print("S0   ",RIGHT,32);
  else if((s_dat <=1) &&(s_dat < 2))
    myGLCD.print("S1   ",RIGHT,32);
  else if((s_dat <= 2) && (s_dat < 4))
    myGLCD.print("S3   ",RIGHT,32);
  else if((s_dat <= 4) && (s_dat < 5))
    myGLCD.print("S5   ",RIGHT,32);            
  else if((s_dat <= 5) && (s_dat < 7))
    myGLCD.print("S7   ",RIGHT,32);
  else if((s_dat <= 7) && (s_dat < 9))
    myGLCD.print("S9   ",RIGHT,32);
  else if((s_dat <= 9) && (s_dat < 12))
    myGLCD.print("S9+10",RIGHT,32);
  else if((s_dat <= 12) && (s_dat < 13))
    myGLCD.print("S9+20",RIGHT,32);
  else if(s_dat >= 13)
    myGLCD.print("S9+40",RIGHT,32);
}

//----------  Encorder procedure(INT)  ---------------

ISR(PCINT2_vect) {
  unsigned char result = r.process();

  if(Flg_Tx == 0){
    if(result) {  
      if(result == DIR_CW){
        Lng_Wk1 = Vfo_Dat + Enc_Stp;
        Lng_Wk2 = Rit_Dat + Enc_Stp;
      }
      else{
          Lng_Wk1 = Vfo_Dat - Enc_Stp;
          Lng_Wk2 = Rit_Dat - Enc_Stp;
      }    

      if(Flg_Rit == 1)
        Rit_Dat = Lng_Wk2;
      else{
        Vfo_Dat = Lng_Wk1;
        Rit_Dat = 0;
      }

      Vfo_Dat = constrain(Vfo_Dat,LW_FRQ,HI_FRQ);  // VFO range check
      Rit_Dat = constrain(Rit_Dat,LW_RIT,HI_RIT);  // RIT range check
    }
  }
}

//----------  Function Encorder STEP  -----------------------

void Fnc_Stp(){
  if(Enc_Stp == 10)                       // Step = 10Hz ?
    Enc_Stp = 1000000;                    //   Yes,1Mhz set
    else
      Enc_Stp = Enc_Stp / 10;             // Step down 1 digit

  Fnc_Step_Disp();
  while(digitalRead(SW_STEP) == LOW)
    ;
}

//----------  Function STEP Display  ------------------------

void Fnc_Step_Disp(){
  switch(Enc_Stp){
    case 10:
      myGLCD.print("  10", RIGHT,24);
      break;
    case 100:
      myGLCD.print(" 100", RIGHT,24);
      break;
    case 1000:
      myGLCD.print("  1k", RIGHT,24);
      break;
    case 10000:
      myGLCD.print(" 10k", RIGHT,24);
      break;
    case 100000:
      myGLCD.print("100k", RIGHT,24);
      break;
    case 1000000:
      myGLCD.print("  1M", RIGHT,24);
      break;
    default:
      myGLCD.print("  1k", RIGHT,24);
      Enc_Stp = 1000;
      break;
  }
}

//----------  Function Save EEPROM 4byte  ---------

void Fnc_Eep_Sav4(long value,int address){
  address += 3;
  for(int i = 0;i < 4;i++){
    byte toSave = value & 0xFF;
    if(EEPROM.read(address) != toSave){
      EEPROM.write(address,toSave);
      }
    value = value >> 8;
    address--;
  }
}

//----------  Function Load EEPROM 4byte  ---------

long Fnc_Eep_Lod4(int address){
  long value = 0;

  for(int i = 0;i < 4;i++){
    value = value | EEPROM.read(address);
    if( i < 3){
      value = value << 8;
      address++;
    }
  }
  return value;
}


//---------- LCD Display --------------------------

void Fnc_Lcd(){
  char s[6] ={'\0'};
 
  Fnc_Fdsp(Vfo_Dat);
  myGLCD.setFont(SmallFont);
  if(Flg_Tx == 0)
    myGLCD.printNumI(Byt_Chn,RIGHT, 0);
  else
    myGLCD.print("T",RIGHT,0);
  myGLCD.print(tbfo,LEFT,24);           // Mode
  if(Flg_Rit == 1){
    myGLCD.print("R",0,32);
    if(Rit_Dat != Rit_Datb){
      myGLCD.print("      ",6, 32);
      Rit_Datb = Rit_Dat;
    }
    sprintf(s,"%+d",Rit_Dat);
    myGLCD.print(s,6, 32);
  }
}

//----------  Function Frequency Display  ---------

void Fnc_Fdsp(long f_disp){ 
  long f1 = f_disp / 1000L;
  long f2 = f_disp % 1000L;
  char s1[6] ={'\0'};
  char s2[4] ={'\0'};

  if(f_disp != Vfo_Datb){
    myGLCD.clrRow(0);
    myGLCD.clrRow(1);
    Vfo_Datb = Vfo_Dat;
  }
  myGLCD.setFont(MediumNumbers);
  myGLCD.printNumI(f1,LEFT, 0);
  myGLCD.setFont(SmallFont);
  sprintf(s2,"%03d",f2);
  myGLCD.print(s2,RIGHT,8);
}

//----------  Function Rit  ---------

void Fnc_Rit(){
  char s[6] ={'\0'};
 
  if(Flg_Rit == 0){
    Rit_Dat = 0;
    Fnc_Eep_Wt(Byt_Chn);
    myGLCD.print("R",0,32);
    sprintf(s,"%+d",Rit_Dat);
    myGLCD.print(s,6, 32);
    Flg_Rit = 1;
  }
  else{
    Flg_Rit = 0;
    myGLCD.print("        ",0, 32);
  }

  while(digitalRead(SW_RIT) == LOW)
    ;
}

//----------  Function CH SW Check  ---------

void Fnc_Chsw(){
  byte cnt = 0;
 
  Byt_Chn++;
 
  while(digitalRead(SW_CH) == LOW){
    delay(500);
    cnt++;
    if(6 <= cnt){                               // Eep Initial start(3sec)?
      Fnc_Eep_Int();                            // Initialization
      Byt_Chn = EEPROM.read(Chn_Eep);           // Channel Read
      Byt_Chnb = Byt_Chn;
      Fnc_Eep_Rd();                             // EEPROM Read
      Fnc_Fdsp(Vfo_Dat);
      myGLCD.print("default",CENTER,16); 
    }
  }
}

//----------  Function Band  -------------------------------

void Fnc_Band(long vfo){
  if((vfo >= LW_VFO80) && (vfo < MI_VFO80)){          // 3.5MHz
    Flg_Over = 0;
    tbfo = "CW ";
    bfo = CW;
  }
  else if((vfo >= MI_VFO80) && (vfo <= HI_VFO80)){
    Flg_Over = 0;
    tbfo = "LSB";
    bfo = LSB;
  }

  else if((vfo >= LW_VFO40) && (vfo < MI_VFO40)){     // 7MHz
    Flg_Over = 0;
    tbfo = "CW ";
    bfo = CW;
  }
  else if((vfo >= MI_VFO40) && (vfo <= HI_VFO40)){
    Flg_Over = 0;
    tbfo = "LSB";
    bfo = LSB;
  }

  else if((vfo >= LW_VFO20) && (vfo < MI_VFO20)){     // 14MHz
    Flg_Over = 0;
    tbfo = "CW ";
    bfo = CW;
  }
  else if((vfo >= MI_VFO20) && (vfo <= HI_VFO20)){
    Flg_Over = 0;
    tbfo = "USB";
    bfo = USB;
  }

  else if((vfo >= LW_VFO15) && (vfo < MI_VFO15)){     // 21MHz
    Flg_Over = 0;
    tbfo = "CW ";
    bfo = CW;
  }
  else if((vfo >= MI_VFO15) && (vfo <= HI_VFO15)){
    Flg_Over = 0;
    tbfo = "USB";
    bfo = USB;
  }

  else if((vfo >= LW_VFO10) && (vfo < MI_VFO10)){     // 28MHz
    Flg_Over = 0;
    tbfo = "CW ";
    bfo = CW;
  }
  else if((vfo >= MI_VFO10) && (vfo <= HI_VFO10)){
    Flg_Over = 0;
    tbfo = "USB";
    bfo = USB;
  }

  else if (Vfo_Dat < 10000000L){
    bfo = LSB;
    tbfo = "LSB";
    Flg_Over = 1;
  }
  else{
    bfo = USB;
    tbfo = "USB";
    Flg_Over = 1;
  }
}

//---------- Function Eeprom Initialization -----------------

void Fnc_Eep_Int(){
  int i;

  for (i=0;i<112;i++)                            // 0 clear(112byte)
    EEPROM.write(i, 0);

  for(i=0;i<Max_Chn;i++){
    Fnc_Eep_Sav4(DEF_FRQ,Frq_Eep+i*4);            // Frequency(7.05MHz)
    Fnc_Eep_Sav4(DEF_STP,Stp_Eep+i*4);            // Step(1kHz)
  }

  EEPROM.write(Eep_Int,Int_End);                  // Init end set(73) 
}

//----------  Function EEPROM Read  ---------

void Fnc_Eep_Rd(){
  if((0 <= Byt_Chn) && (Byt_Chn < Max_Chn))
    Vfo_Dat = Fnc_Eep_Lod4(Frq_Eep+Byt_Chn*4);
  else{
    Vfo_Dat = Fnc_Eep_Lod4(Frq_Eep+0*4);
    Byt_Chn = 0;
  }

  if((0 <= Byt_Chn) && (Byt_Chn < Max_Chn))
    Enc_Stp = Fnc_Eep_Lod4(Stp_Eep+Byt_Chn*4);
  else
    Enc_Stp = Fnc_Eep_Lod4(Stp_Eep+0*4);
}

//----------  Function EEPROM Write  ---------

void Fnc_Eep_Wt(byte chn){
  if((0 <= chn) && (chn < Max_Chn)){
    Fnc_Eep_Sav4(Vfo_Dat,Frq_Eep+chn*4);
    Fnc_Eep_Sav4(Enc_Stp,Stp_Eep+chn*4);
  }

  EEPROM.write(Chn_Eep,chn);
}
