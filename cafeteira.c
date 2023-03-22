// DEFINES GLOBAIS
#define F_CPU 16000000 // run CPU at 16 MHz
#define ClearBit(x,y) x &= ~_BV(y) // equivalent to cbi(x,y)
#define SetBit(x,y) x |= _BV(y) // equivalent to sbi(x,y)

// INCLUDES
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <virtuabotixRTC.h> 

// TYPEDEFS
typedef uint8_t byte;
typedef int8_t sbyte;

//----------------------------------------------------------------------------
// VARIAVEIS GLOBAIS
byte h1,m1=27;    // HORA A MINUTO  A SER SETADO PARA O DS
int counter=0;    // CONTAR  59 SEG.

// ROTINA DE DEFINICAO DE ENTRADAS E SAIDAS

void InitAVR()
{
  DDRB = 0x3F; // 0011.1111; SET B0-B5 COMO SAIDA
  DDRC = 0x00; // 0000.0000; SET PORTC ENTRADA
  DDRD = 0xFF; // 1111.1111; SET d0-d7 como saidas
  
}

// ROTINA PARA DELAY NO DEBOUNCING DOS BOTOES E LCD
void msDelay(int delay)
{
  for (int i=0;i<delay;i++)
  _delay_ms(1);
}

//----------------------------------------------------------------------------
// HD44780-LCD DRIVER ROUTINES
//
// Rotinas
// LCD_Init inicia o LCD
// LCD_Cmd Manda o comando par ao lcd
// LCD_Clear Limpa o LCD
// LCD_Line colo o cursor na linha
// LCD_Hex printa valores em hexadecimal
// LCD_Integer printa valores inteiros no LCD
// LCD_String Printa strings no LCD
//
// o LCD precisa de 6 pinos para se comunicar o RS E e 4 para comunicação
// usa o PORTB  para comunicaçao.

#define LCD_RS 0
#define LCD_E 1 //enable lCD
#define DAT4 2
#define DAT5 3
#define DAT6 4
#define DAT7 5

// comandos para dar clear e set do cursor
#define CLEARDISPLAY 0x01
#define SETCURSOR 0x80

void PulseEnableLine ()
{
  SetBit(PORTB,LCD_E); // habilita lcd
  _delay_us(40);
  ClearBit(PORTB,LCD_E); // desabilita LCD
}

void SendNibble(byte data)
{
  PORTB &= 0xC3; // clear nos 4 pinos da comunicação
  if (data & _BV(4)) SetBit(PORTB,DAT4);
  if (data & _BV(5)) SetBit(PORTB,DAT5);
  if (data & _BV(6)) SetBit(PORTB,DAT6);
  if (data & _BV(7)) SetBit(PORTB,DAT7);
  PulseEnableLine(); // habilita os 4 bits para comunicação
}

void SendByte (byte data)
{
  SendNibble(data); // envia 4 bits mais significativos
  SendNibble(data<<4); // send lower 4 bits
  ClearBit(PORTB,5); // turn off boarduino LED
}

void LCD_Cmd (byte cmd)
{
  ClearBit(PORTB,LCD_RS); // R/S line 0 = command data
  SendByte(cmd);
}

void LCD_Char (byte ch)
{
  SetBit(PORTB,LCD_RS); // R/S line 1 = character data
  SendByte(ch); // send it
}

void LCD_Init()
{
  LCD_Cmd(0x33);
  LCD_Cmd(0x32); // comunicaco em 4 bit
  LCD_Cmd(0x28); // 2 linhas
  LCD_Cmd(0x0C); // cursor off
  LCD_Cmd(0x06); // cursor para  direita
  LCD_Cmd(0x01); // clear display
  msDelay(3); // delay para iniciar o lcd
}

void LCD_Clear() // limpa o lcd
{
  LCD_Cmd(CLEARDISPLAY);
  msDelay(3);
}

void LCD_Home() // coloca o cursor na primeira linha
{
  LCD_Cmd(SETCURSOR);
}

void LCD_Goto(byte x, byte y) // coloca o cursor em um ponto especifico
{
  byte addr = 0;
  switch (y)
  {
    case 1: addr = 0x40; break;
    case 2: addr = 0x14; break;
    case 3: addr = 0x54; break;
  }
  LCD_Cmd(SETCURSOR+addr+x);
}

void LCD_Line(byte row) // coloca o cursor em uma determinada linha
{
  LCD_Goto(0,row);
}

void LCD_String(const char *text) // printa uma string no LCD
{
  while (*text)
  LCD_Char(*text++); // envia uma letra por vez
}

void LCD_Hex(int data)

{
  char st[8] = "";
  itoa(data,st,16); // converte para ascii
  LCD_String(st); // printa a string
}

void LCD_Integer(int data) // printa um inteiro
{
  char st[8] = "";
  itoa(data,st,10); // converte para ascii
  LCD_String(st); // printa a string
}

//----------------------------------------------------------------------------
// DS1302 RTC rotinas
#define   clk   5
#define   dat   6
#define   rst   7
#define   segL		1
#define   minL		1
#define   horL		1
#define   d_semL	3
#define   d_mesL	1
#define   mesL		1
#define   anoL		2019

virtuabotixRTC   myRTC(clk, dat, rst);
myRTC.setDS1302Time(segL, minL, horL, d_semL, d_mesL, mesL, anoL);

//----------------------------------------------------------------------------
// DS1307 RTC rotinas
#define DS1307 0xD0 // I2C bus address of DS1307 RTC
#define SECONDS_REGISTER 0x00
#define MINUTES_REGISTER 0x01
#define HOURS_REGISTER 0x02
#define DAYOFWK_REGISTER 0x03
#define DAYS_REGISTER 0x04
#define MONTHS_REGISTER 0x05
#define YEARS_REGISTER 0x06
#define CONTROL_REGISTER 0x07
#define RAM_BEGIN 0x08
#define RAM_END 0x3F

// retorn hora, minuto, segundo.
void DS1307_GetTime(byte *hours, byte *minutes, byte *seconds)
{
  *hours = I2C_ReadRegister(DS1307,HOURS_REGISTER);
  *minutes = I2C_ReadRegister(DS1307,MINUTES_REGISTER);
  *seconds = I2C_ReadRegister(DS1307,SECONDS_REGISTER);
  if (*hours & 0x40) // modo de 12 horas
  *hours &= 0x1F; // modo pm am
  else *hours &= 0x3F; // modo 24 horas
}

// retorna o dias mes e ano.
void DS1307_GetDate(byte *months, byte *days, byte *years)
{
  *months = I2C_ReadRegister(DS1307,MONTHS_REGISTER);
  *days = I2C_ReadRegister(DS1307,DAYS_REGISTER);
  *years = I2C_ReadRegister(DS1307,YEARS_REGISTER);
}

// setar a data e horario no rtc
void SetTimeDate()
{
  I2C_WriteRegister(DS1307,MONTHS_REGISTER, 0x08);
  I2C_WriteRegister(DS1307,DAYS_REGISTER, 0x31);
  I2C_WriteRegister(DS1307,YEARS_REGISTER, 0x13);
  I2C_WriteRegister(DS1307,HOURS_REGISTER, 0x08+0x40); // add 0x40 for PM
  I2C_WriteRegister(DS1307,MINUTES_REGISTER, 0x51);
  I2C_WriteRegister(DS1307,SECONDS_REGISTER, 0x00);
}

void SetAno(byte ano){
  I2C_WriteRegister(DS1307,YEARS_REGISTER, ano);
}

void SetMes(byte mes){

  I2C_WriteRegister(DS1307,MONTHS_REGISTER, mes);
}

void SetDia(byte dia){

  I2C_WriteRegister(DS1307,DAYS_REGISTER, dia);
}

void SetHora(byte hora){

  I2C_WriteRegister(DS1307,HOURS_REGISTER, hora); // add 0x40 for PM
}

void SetMinutos(byte minuto){
  I2C_WriteRegister(DS1307,MINUTES_REGISTER, minuto);
}

void SetSegundos(byte segundo){
  I2C_WriteRegister(DS1307,SECONDS_REGISTER,segundo);
}

//----------------------------------------------------------------------------
// ROTINAS EEPROM
void eeprom_write_byte1 (uint16_t addr, uint8_t data)
{
  while(EECR & (1<<EEPE))
  ;
  EEAR = addr;
  EEDR = data;
  EECR |= (1<<EEMPE);
  EECR |= (1<<EEPE);
}

byte eeprom_read_byte1(uint16_t addr)
{
  while(EECR & (1<<EEPE))
  ;
  EEAR = addr;
  EECR |= (1<<EERE);
  return EEDR;
}

void eeprom_write_string(uint16_t addr, char *s)
{
  while(*s)
  {
    eeprom_write_byte1(addr, *s );
    ++s;
    ++addr;
  }
}

// le a string
void eeprom_read_string(uint16_t addr, char *s, int len)
{
  while(len)
  {
    *s = eeprom_read_byte1(addr);
    if( *s == '\0' )
    return;
    --len;
    ++s;
  }
  *s = '\0';
  return;
}

//----------------------------------------------------------------------------
// APPLICATION ROUTINES

void ShowDevices()
{
  LCD_Line(1); LCD_String("Found:");
  byte addr = 1;
  while (addr>0)
  {
    LCD_Char(' ');
    addr = I2C_FindDevice(addr);
    if (addr>0) LCD_Hex(addr++);
  }
}

// printa dois numeros no lCD
void LCD_TwoDigits(byte data)
{
  byte temp = data>>4;
  LCD_Char(temp+'0');
  data &= 0x0F;
  LCD_Char(data+'0');
}

/*void WriteDate()
{
  byte months, days, years;
  DS1307_GetDate(&months,&days,&years);
  LCD_Hex(days);
  LCD_Char('/');
  LCD_Hex(months);
  LCD_Char('/');
  LCD_Hex(years);
}

// escreve a hora no DS1307
void WriteTime()
{
  byte hours, minutes, seconds;
  DS1307_GetTime(&hours,&minutes,&seconds);
  LCD_Hex(hours);
  LCD_Char(':');
  LCD_Hex(minutes);
  LCD_Char(':');
  LCD_TwoDigits(seconds);
}

//escreve a hora no DS1307
void LCD_TimeDate()
{
  LCD_Line(0); WriteTime();
  LCD_Line(1); WriteDate();
}*/

//----------------------------------------------------------------------------
// Rotinas de teclado

#define le_bit(reg,bit)   (reg & (1<<bit))

// funções lebotao realiza o debouncing e verifica se o botao foi solto
int LeBotao1(){
  if(le_bit(PINC,PINC0)){
    _delay_ms(2);
    if(le_bit(PINC,PINC0)==0){
      _delay_ms(2);
      if(le_bit(PINC,PINC0)==0){
        return 1;
      }
    }
  }
  return 0;
}

int LeBotao2(){
  if(le_bit(PINC,PINC1)){
    _delay_ms(2);
    if(le_bit(PINC,PINC1)==0){
      _delay_ms(2);
      if(le_bit(PINC,PINC1)==0){
        return 1;
      }
    }
  }
  return 0;
}

int LeBotao3(){
  if(le_bit(PINC,PINC2)){
    _delay_ms(2);
    if(le_bit(PINC,PINC2)==0){
      _delay_ms(2);
      if(le_bit(PINC,PINC2)==0){
        return 1;
      }
    }
  }
  return 0;
}

int LeBotao4(){
  if(le_bit(PINC,PINC3)){
    _delay_ms(2);
    if(le_bit(PINC,PINC3)==0){
      _delay_ms(2);
      if(le_bit(PINC,PINC3)==0){
        return 1;
      }
    }
  }
  return 0;
}

//----------------------------------------------------------------------------
// ROTINAS DE MENU
// configuracao do alarme se alarme
// botao 1 para o escolher o alarme a ser disparado e botao 4 para selecionar
// como o botao 1 e 2 seta a hora e o minuto e grava na memoria

void configuraAlarme(){
  byte h=1,m=0,botao4=1;
  int n=0;
  
  LCD_Clear();
  LCD_String("Escolha o alarme:");
  while(botao4==1){
    if(LeBotao1()){
      if(n>=8){
        n=0;
      }
      n++;
      LCD_Clear();
      LCD_String("Escolha o alarme:");
      LCD_Line(1);
      LCD_Integer(n);
    }
    if(LeBotao4()){
      botao4=0;
    }
  }
  botao4=1;
  LCD_Clear();
  LCD_String("Defina o horario");
  LCD_Line(1);
  LCD_Integer(h);
  LCD_Char(':');
  LCD_Integer(m);
  
  while(botao4==1){
    if(LeBotao1()){
      if(h>=24){
        h=0;
      }
      h++;
      LCD_Clear();
      LCD_String("Defina o horario");
      LCD_Line(1);
      LCD_Integer(h);
      LCD_Char(':');
      LCD_Integer(m);
    }
    if(LeBotao2()){
      if(m>=60){
        m=0;
      }
      m++;
      LCD_Clear();
      LCD_String("Defina o horario");
      LCD_Line(1);
      LCD_Integer(h);
      LCD_Char(':');
      LCD_Integer(m);
    }
    if(LeBotao4()){
      botao4=0;
    }
  }
  if(n==1){
    eeprom_write_byte1(0x01,h);
    eeprom_write_byte1(0x11,m);
  }
  if(n==2){
    eeprom_write_byte1(0x02,h);
    eeprom_write_byte1(0x12,m);
  }
  if(n==3){
    eeprom_write_byte1(0x03,h);
    eeprom_write_byte1(0x13,m);
  }
  if(n==4){
    eeprom_write_byte1(0x04,h);
    eeprom_write_byte1(0x14,m);
  }
  if(n==5){
    eeprom_write_byte1(0x05,h);
    eeprom_write_byte1(0x15,m);
  }
  if(n==6){
    eeprom_write_byte1(0x06,h);
    eeprom_write_byte1(0x16,m);
  }
  if(n==7){
    eeprom_write_byte1(0x07,h);
    eeprom_write_byte1(0x17,m);
  }
  if(n==8){
    eeprom_write_byte1(0x08,h);
    eeprom_write_byte1(0x18,m);
  }
  
  LCD_Clear();
  LCD_String("Configura Alarme");
  LCD_Line(1);
  LCD_String("Novo(1) Apagar(2)");
  
}

// apagar alarme seleciona o alarme a ser apagado com o botao 1 e botao 4 para confirmar
void apagaAlarme(){
  byte n=1,x=0,botao4=0;
  LCD_Clear();
  LCD_String("Apagar alarme(1)");
  
  while(botao4==0){
    if(LeBotao1()){
      n++;
      x=0;
    }
    if(LeBotao4()){
      botao4=1;
    }
    
    if(n>8){
      n=1;
    }
    
    if(n==1){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 1 ->");
        LCD_Integer(eeprom_read_byte1(0x01));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x11));
      }
    }
    
    if(n==2){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 2 ->");
        LCD_Integer(eeprom_read_byte1(0x02));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x12));
      }
      
    }

    if(n==3){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 3 ->");
        LCD_Integer(eeprom_read_byte1(0x03));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x13));
      }
    }
    if(n==4){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 4 ->");
        LCD_Integer(eeprom_read_byte1(0x04));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x14));
      }
    }
    
    if(n==5){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 5 ->");
        LCD_Integer(eeprom_read_byte1(0x05));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x15));
      }
    }

    if(n==6){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 6 ->");
        LCD_Integer(eeprom_read_byte1(0x06));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x16));
      }
    }
    
    if(n==7){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 7 ->");
        LCD_Integer(eeprom_read_byte1(0x07));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x17));
      }
    }

    if(n==8){
      if(x==0){
        LCD_Clear();
        LCD_String("Apagar alarme(1)");
        LCD_Line(1);
        x=1;
        LCD_String("n = 8 ->");
        LCD_Integer(eeprom_read_byte1(0x08));
        LCD_String(":");
        LCD_Integer(eeprom_read_byte1(0x18));
      }
    }
  }
  if(n==1){
    eeprom_write_byte1(0x01,255);
    eeprom_write_byte1(0x11,255);
  }
  if(n==2){
    eeprom_write_byte1(0x02,255);
    eeprom_write_byte1(0x12,255);
  }
  if(n==3){
    eeprom_write_byte1(0x03,255);
    eeprom_write_byte1(0x13,255);
  }
  if(n==4){
    eeprom_write_byte1(0x04,255);
    eeprom_write_byte1(0x14,255);
  }
  if(n==5){
    eeprom_write_byte1(0x05,255);
    eeprom_write_byte1(0x15,255);
  }
  if(n==6){
    eeprom_write_byte1(0x06,255);
    eeprom_write_byte1(0x16,255);
  }
  if(n==7){
    eeprom_write_byte1(0x07,255);
    eeprom_write_byte1(0x17,255);
  }
  if(n==8){
    eeprom_write_byte1(0x08,255);
    eeprom_write_byte1(0x18,255);
  }
  LCD_Clear();
  LCD_String("alarme apagado");
  _delay_ms(3000);
  LCD_Clear();
  LCD_String("Configura Alarme");
  LCD_Line(1);
  LCD_String("Novo(1) Apagar(2)");
}

// funcao para selecionar as funcoes de configura alarme e apaga alarme
void configAlarme(){
  byte botao4 =1;
  LCD_Clear();
  LCD_String("Configura Alarme");
  LCD_Line(1);
  LCD_String("Novo(1)Apagar(2)");
  
  while(botao4 ==1){
    if (LeBotao4()){
      botao4 =0;
    }
    
    if (LeBotao1()){
      configuraAlarme();
    }
    if (LeBotao2()){
      apagaAlarme();
    }
  }
  LCD_Clear();
  LCD_String("Cafeteira");
  LCD_Line(1);
  LCD_String("Alarme(1)Hora(2)");
}

// transforma decimal para hexadecimal matematicamente.
int DectoHex(int x) {
  if (x>=10){
    if (x>=20){
      if (x>=30){
        if (x>=40){
          if (x>=50){
            if (x>=60){
              x+=6;
            }
            x+=6;
          }
          x+=6;
        }
        x+=6;
      }
      x+=6;
    }
    x+=6;
  }
  return x;
}

// com o botao 1 2 e 3 configura o dia o mes e o ano a ser setada botao 4 para confirmar salvando em hexadecimal
void configData(){
  byte d=1,m=1,a=19,botao4=1;
  LCD_Clear();
  SetDia(d);
  SetMes(m);
  SetAno(DectoHex(a));
  WriteDate();
  while(botao4==1){
    if(LeBotao1()){
      if(d>=31){
        d=0;
      }
      d++;
      SetDia(DectoHex(d));
      LCD_Clear();
      
      WriteDate();
    }
    if(LeBotao2()){
      if(m>=12){
        m=0;
      }
      m++;
      SetMes(DectoHex(m));
      LCD_Clear();
      WriteDate();
    }
    if(LeBotao3()){
      if(a>=30){
        a=17;
      }
      a++;
      SetAno(DectoHex(a));
      LCD_Clear();
      WriteDate();
    }
    if(LeBotao4()){
      botao4=0;
    }
  }
  LCD_Clear();
  LCD_String("Config Data(1)");
  LCD_Line(1);
  LCD_String("Config Hora(2)");
}

// com o botao 1 e 2 escolhe a hora e minuto com o 4 confirma e salva em hexadecimal
void setaHora(){
	byte h=0,m=0,botao4=1;
	LCD_Clear();
	// LCD_String("Config Hora");
	// LCD_Line(1);
	// LCD_String("Config minuto");
	
	while(botao4==1){
    if(LeBotao1()){
      if(h>=23){
			h=0;
		}
		h++;
		// SetHora(DectoHex(h));
		LCD_Clear();
		LCD_String("Config Hora");
		LCD_Line(1);
		LCD_Hex(h);
    }
}

void configHora(){
  byte h=0,m=0,botao4=1;
  LCD_Clear();
  SetHora(h);
  SetMinutos(m);
  WriteTime();
  while(botao4==1){
    if(LeBotao1()){
      if(h>=23){
        h=0;
      }
      h++;
      SetHora(DectoHex(h));
      LCD_Clear();
      WriteTime();
    }
    if(LeBotao2()){
      if(m>=59){
        m=0;
      }
      m++;
      SetMinutos(DectoHex(m));
      LCD_Clear();
      WriteTime();
    }
    if(LeBotao4()){
      botao4=0;
      I2C_WriteRegister(DS1307,SECONDS_REGISTER, 0x00);
      LCD_Clear();
      LCD_String("alarme configurado");
      _delay_ms(3000);
    }
  }
  LCD_Clear();
  LCD_String("Config Data(1)");
  LCD_Line(1);
  LCD_String("Config Hora(2)");
}

// funcao para escolher a funcao configura dara e configura hora
void configDataHora(){
  byte botao4=0;
  LCD_Clear();
  LCD_String("Config Data(1)");
  LCD_Line(1);
  LCD_String("Config Hora(2)");
  while (botao4==0)
  {
    if (LeBotao1()){
      setaHora();
    }
    if (LeBotao2()){
      configHora();
    }
    if(LeBotao4()){
      botao4 =1;
    }
  }
  LCD_Clear();
  LCD_String("Cafeteira");
  LCD_Line(1);
  LCD_String("Alarme(1)Hora(2)");
}

// funçao chamada pela interrupcao a cada 59 segundos
// le a hora e o minuto no RTC converte para ascii e depois para inteiro.
// compara a hora e o minuto setados em todos os depertadores com o do RTC
// caso encontre um alarme com hora e minuto igual acende o led referente ao compartimento
// somente ira apagar o led quando o botao 4 for apertado desligando o led.
void testeAlarme(){
  
  char hora[8];
  char minuto[8];
  
  byte horaD;
  byte minutoD;

  itoa(I2C_ReadRegister(DS1307,HOURS_REGISTER),hora,16); // convert to ascii
  horaD = atoi(hora);
  itoa(I2C_ReadRegister(DS1307,MINUTES_REGISTER),minuto,16); // convert to ascii
  minutoD = atoi(minuto);
  
  if (horaD==eeprom_read_byte1(0x01) && minutoD==eeprom_read_byte1(0x11) )
  {
    SetBit(PORTD,PORTD0);
    SetBit(PORTC,PORTC6);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
    ClearBit(PORTC,PORTC6);
  }
  if (horaD==eeprom_read_byte1(0x02) && minutoD==eeprom_read_byte1(0x12) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
  if (horaD==eeprom_read_byte1(0x03) && minutoD==eeprom_read_byte1(0x13) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
  if (horaD==eeprom_read_byte1(0x04) && minutoD==eeprom_read_byte1(0x14) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
  if (horaD==eeprom_read_byte1(0x05) && minutoD==eeprom_read_byte1(0x15) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
  if (horaD==eeprom_read_byte1(0x06) && minutoD==eeprom_read_byte1(0x16) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
  if (horaD==eeprom_read_byte1(0x07) && minutoD==eeprom_read_byte1(0x17) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
  if (horaD==eeprom_read_byte1(0x08) && minutoD==eeprom_read_byte1(0x18) )
  {
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }
}

// chama a função configura data ou configura hora referente aos botões 1 e 2
// com o  botão 3 imprime no LCD o a hora e a data
void menu(){
  LCD_Clear();
  LCD_String("Cafeteira");
  LCD_Line(1);
  LCD_String("Alarme(1)Hora(2)");

  INICIO:
  if(LeBotao1()){
    configAlarme();
  }
  if(LeBotao2()){
    configDataHora();
  }
  if(LeBotao3()){
    LCD_Clear();
    LCD_TimeDate();
    _delay_ms(1000);
    LCD_Clear();
    LCD_String("Cafeteira");
    LCD_Line(1);
    LCD_String("Alarme(1)Hora(2)");
  }
  if(LeBotao4()){
    SetBit(PORTD,PORTD0);
    while(LeBotao4()==0){
    }
    ClearBit(PORTD,PORTD0);
  }

  goto INICIO;
  
}

//----------------------------------------------------------------------------
// MAIN PROGRAM
// inicia o LCD e comunicação I2C e ativa a interrupção.
int main(void)
{
  InitAVR(); // set port direction
  LCD_Init(); // initialize HD44780 LCD controller
  I2C_Init(); // set I2C clock frequency
  LCD_Clear();
  // _delay_ms(10);
  
  // Set the Timer Mode to CTC
  TCCR0A |= (1 << WGM01);

  // Set the value that you want to count to
  OCR0A = 0xF9;

  TIMSK0 |= (1 << OCIE0A);    // Set the ISR COMPA vect

  sei();         // enable interrupts

  TCCR0B |= (1 << CS02);
  // set prescaler to 256 and start the timer

  // ShowDevices();
  // _delay_ms(200);

  menu();
}

// interrupacao chamada a cada 4,08ms pelo main
// realiza a soma de um counter até 14460 referente a 59 segundos
// quando a soma chegar no valor referente a 59s chama a função testa alarme
ISR (TIMER0_COMPA_vect)  // timer0 overflow interrupt
{
  counter++;
  if (counter == 14460 ){
    testeAlarme();
    counter = 0;
  }
  
}