#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include "uart.h"

#define pi 3.14159265358979323

volatile int g_cnt = 0;
volatile int fast;
volatile int set = 0;
volatile int water_on = 0;
volatile int led_on = 0;
volatile int fan_on =0;

// LPF Parameter
volatile double dt=0.01;
volatile double LPF_fc = 5;
volatile double LPF_tau = 1/(2*5*pi);
volatile double LPF=0;
volatile double LPF_past=0;
// Moving Average Filter Parameter
volatile double MAF_sample[10]={0,};
volatile int m=0;
volatile double MAF=0;

void adc_func(double num);
void cds_func(double num);
void lm35_func(double num);
void psd_func(double num);
void moist_func(double num);
void gas_func(double num);

double Low_Pass_Filter(double data);
double Moving_Average_Filter(double data);

double Low_Pass_Filter(double data)
{
	LPF = (dt*data+LPF_tau*LPF_past)/(LPF_tau+dt);
	LPF_past = LPF;
	
	return LPF;
}

double Moving_Average_Filter(double data)
{
	MAF = 0;
	MAF_sample[m] = data;
	int j=0;
	for(j=0;j<10;j++)
	{
		MAF += MAF_sample[j];
	}
	if (m==9) m=0;
	else m++;
	
	return (MAF/10);
}

//***ADC
double get_ADC(void)
{
   ADMUX = 0x00;
   ADCSRA |= (1<<ADSC);//0x40;
   while(!(ADCSRA & (1<<ADIF)));   //flag check
   int adc = ADC;
   double Vadc = adc*5./1023.;

   return Vadc;
}

//***LM35 온도센서
double get_LM35(void)
{
   ADMUX = 0x02;
   ADCSRA |= (1<<ADSC);//0x40;
   while(!(ADCSRA & (1<<ADIF)));   //flag check
   int adc = ADC;
   double Vadc = adc*5./1023.;
   
   return Vadc;
}

double LM35_calculate(double num)
{
   double T = 100*num;
   return T; //['c]
}

//***CDS 조도센서
double get_CDS(void)
{
   ADMUX = 0x01;
   ADCSRA |= (1<<ADSC);//0x40;
   while(!(ADCSRA & (1<<ADIF)));   //flag check
   int adc = ADC;
   double Vadc = adc*5./1023.;
   
   return Vadc;
}

double CDS_calculate(double num)
{
   double Rcds = 5*4700/num - 4700;
   long y = (1- (log(Rcds)-log(35000.))/0.7); 
   double x = 10^y;
   return x; //[Lux]
}

//***PSD 센서
double get_PSD(void)
{
   ADMUX = 0x04;
   ADCSRA |= (1<<ADSC);//0x40;
   while(!(ADCSRA & (1<<ADIF)));   //flag check
   int adc = ADC;
   double Vadc = adc*5./1023.;
   
   return Vadc;
}

double PSD_calculate(double num)
{
   double distance = (27.61/ (num - 0.1696));
   if(distance < 10)
   {
	   distance = 10;
   }
   return distance; //[cm]
}

//***토양 습도 센서
double get_MOIST(void)
{
   ADMUX = 0x05;
   ADCSRA |= (1<<ADSC);//0x40;
   while(!(ADCSRA & (1<<ADIF)));
   int adc = ADC;
   double Vadc = adc*5./1023.;

   return Vadc;
}

double MOIST_calculate(double num)
{
   double soil_moist = ((5-num)/5.)*(100);
   return soil_moist; //[per]
}

//***가스 센서
double get_GAS(void)
{
   ADMUX = 0x06; 
   ADCSRA |= (1<<ADSC);//0x40;
   while(!(ADCSRA & (1<<ADIF)));   //flag check
   int adc = ADC;
   double Vadc = adc*5./1023.;
   
   return Vadc;
}

double GAS_calculate(double num)
{
	double a = 574.25, b=-2.222; //LPG
	double Rs = (5/num-1)*10;
	double ratio = Rs/10;
	double ppm = a*pow(ratio,b);	
	return ppm; //[ppm]
}

ISR(TIMER2_OVF_vect)
{
   TCNT2 = 255 - 156;
   g_cnt++;
   
   if(g_cnt == 100)
   {
      g_cnt = 0;
     
      double adc = get_ADC();//[V]
      double adc_cds = get_CDS();   //[V]
      double adc_lm35 = get_LM35();   //[V]
      double adc_psd = get_PSD();   //[V]
      double adc_moist = get_MOIST();   //[V]
      double adc_gas = get_GAS();   //[V]
      
      double cds = CDS_calculate(adc_cds); //[Lux]
      double lm35 = LM35_calculate(adc_lm35);   //['C]
      double psd = PSD_calculate(adc_psd); //[cm]
      double moist = MOIST_calculate(adc_moist); //[per]
      double gas = GAS_calculate(adc_gas); //[ppm]
	  
	  double LPF_adc = Low_Pass_Filter(adc);	  
	  double MAF_adc = Moving_Average_Filter(adc);
	  double LPF_cds = Low_Pass_Filter(cds);
	  double MAF_cds = Moving_Average_Filter(cds);
	  double LPF_lm35 = Low_Pass_Filter(lm35);
	  double MAF_lm35 = Moving_Average_Filter(lm35);
	  double LPF_psd = Low_Pass_Filter(psd);
	  double MAF_psd = Moving_Average_Filter(psd);
	  double LPF_moist = Low_Pass_Filter(moist);
	  double MAF_moist = Moving_Average_Filter(moist);
	  double LPF_gas = Low_Pass_Filter(gas);
	  double MAF_gas = Moving_Average_Filter(gas);
	  
	  
      UART_TX_string(DEC_TO_CHAR(LPF_adc));   UART_TX(','); 
	  UART_TX_string(DEC_TO_CHAR(LPF_cds));   UART_TX(',');
	  UART_TX_string(DEC_TO_CHAR(LPF_lm35));  UART_TX(',');
	  UART_TX_string(DEC_TO_CHAR(LPF_psd));   UART_TX(',');
	  UART_TX_string(DEC_TO_CHAR(LPF_moist));  UART_TX(',');
	  UART_TX_string(DEC_TO_CHAR(LPF_gas));  UART_TX(',');
	  UART_TX_string(DEC_TO_CHAR(fast));  UART_TX(','); 


      UART_TX('\n');
      
      //ADC
      adc_func(LPF_adc);
      //CDS
      cds_func(LPF_cds);
      //LM35
      lm35_func(LPF_lm35);
      //PSD
      psd_func(LPF_psd);
      //MOIST
      moist_func(LPF_moist);
      //GAS
      gas_func(LPF_gas);
   }
}


int main(void)
{
   DDRA = 0xFF;  //led
   DDRB = 0xff; 
   DDRC = 0xff;
   DDRE = 0xff;
   DDRF = 0x00;   //adc input pin
   
    //adc init
   ADMUX = 0x00;   //설정안하면 그냥 0으로됨
   ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//=0x87;
   
   //timer2 interrupt
   TCCR2 = 0x05;
   TCNT2 = 255 - 156;
   TIMSK = (1<<TOIE2);
   sei();
 
   //Motor PWM
   //B channel clear(compare match) & set(overflow), 14 mode: Fast PWM
   TCCR1A = (1<<COM1B1)|(0<<COM1B0)|(1<<WGM11);
   TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS02)|(0<<CS01)|(1<<CS00);
   //14 mode: Fast PWM, Prescaler 1024
   ICR1 = 312; //Top 값
   OCR1B = 200; //B channel PWm
   
   //uart init
   UART1_initialize_polling();
   
   PORTC = 0x00;
   
   while (1)
   {  
     //0b(0)(0)(펌프-)(펌프+)(0)(0)(led)(fan)
     if((!fan_on)&&(!led_on)&&(!water_on)&&(!set))
     {
        PORTC = 0b00000000;
     }
      else if((fan_on)&&(!led_on)&&(!water_on)&&(!set))
      {
          PORTC = 0b00000001;
     }
     else if((!fan_on)&&(led_on)&&(!water_on)&&(!set))
     {
        PORTC = 0b00000010;
     }
     else if((!fan_on)&&(!led_on)&&(water_on)&&(!set))
     {
        PORTC = 0b00000000;
     }
     else if((!fan_on)&&(!led_on)&&(!water_on)&&(set))
     {
        PORTC = 0b00000000;
     }
     else if((fan_on)&&(led_on)&&(!water_on)&&(!set))
     {
        PORTC = 0b00000011;
     }
     else if((fan_on)&&(!led_on)&&(water_on)&&(!set))
     {
        PORTC = 0b00000001;
     }
     else if((fan_on)&&(!led_on)&&(!water_on)&&(set))
     {
        PORTC = 0b00000001;
     }
     else if((!fan_on)&&(led_on)&&(water_on)&&(!set))
     {
        PORTC = 0b00000010;
     }
     else if((!fan_on)&&(led_on)&&(!water_on)&&(set))
     {
        PORTC = 0b00000010;
     }
     else if((!fan_on)&&(!led_on)&&(water_on)&&(set))
     {
        PORTC = 0b00010000;
     }
     else if((fan_on)&&(led_on)&&(water_on)&&(!set))
     {
        PORTC = 0b00000011;
     }
     else if((fan_on)&&(led_on)&&(!water_on)&&(set))
     {
        PORTC = 0b00000011;
     }
     else if((fan_on)&&(!led_on)&&(water_on)&&(set))
     {
        PORTC = 0b00010001;
     }
     else if((!fan_on)&&(led_on)&&(water_on)&&(set))
     {
        PORTC = 0b00010010;
     }
     
     else if((fan_on)&&(led_on)&&(water_on)&&(set))
     {
        PORTC = 0b00010011;
     }    
     else
     {
        PORTC =0b00000000;
     }
   }
}
//ADC
void adc_func(double num)
{
   ICR1 = 312;
   if(num < 1)
   {
       fast = 0;
   }
   else if (num < 2 )
   {
       fast = 128;
   }
   else if (num < 3 )
   {
       fast = 130;
   }
   else if (num < 4 )
   {
       fast = 132;
   }
   else if (num < 5 )
   {
       fast = 200;
   }
   else if (num < 6 )
   {
      fast = 280;
   }
   else
   {
      fast = 0;
   }
}

//CDS
void cds_func(double num)
{   
   if(num > 11)
   {
       led_on = 0; //천장 조명 off
   }
   else
   {
       led_on = 1; //천장 조명 on
   }
}

//LM35
void lm35_func(double num)
{
   if(num <= 18)
   {
       PORTA = 0b11111110;
   }
   else if(num <= 20)
   {
       PORTA = 0b11111100;
   }
   else if(num <= 22)
   {
       PORTA =0b11111000;
   }
   else if(num <= 24)
   {
       PORTA = 0b11110000;
   }
   else if(num <= 26)
   {
       PORTA = 0b11100000;
   }
   else if(num <= 28)
   {
       PORTA = 0b11100000;
   }
   else if(num <= 30)
   {
       PORTA = 0b11000000;
   }
   else
   {
       PORTA = 0b00000000;
   }
}

//PSD
void psd_func(double num)
{
   if((num >= 11)&&(num <= 13))
   {
       set = 1;
       PORTE = (0<<PORTE0)|(0<<PORTE1);
       OCR1B = fast;
   }
   else if(num > 13)
   {
       set = 0;
       PORTE = (0<<PORTE0)|(1<<PORTE1);
       OCR1B = fast;
   }
   else if(num < 11)
   {
       set = 0;
       PORTE = (1<<PORTE0)|(0<<PORTE1);
       OCR1B = fast;
   }
   else
   {
       set = 0;      
       PORTE = (0<<PORTE0)|(0<<PORTE1);
       OCR1B = fast;
   }
}

//MOIST
void moist_func(double num)
{   
   if(num < 60)
   {
      water_on = 1;
   }
   else
   {
      water_on = 0;
   }
}

//MQ2
void gas_func(double num)
{
   if(num > 150)
   {
      fan_on = 1;
   }
   else
   {
      fan_on = 0;
   }
}