/*
 * lfw2.c
 *
 * Created: 4/7/2014 06:46:07
 *  Author: kxm
 */ 
/*
6 sensor
hitam base
s0,		s1,		s2,		s3,	s4,		s5
adc0,	adc1, adc2,  adc3,  adc4,  adc5
pin20, pin19, pin18, pin14, pin13, pin12

mapping error:, Present Value (PV)
111110	// -3, ujung kiri
111001	// -2
111011	// -1
110011	// 0, tengah
110111	// 1
100111	// 2
011111	//3, ujung kanan

111111	//out of order, lost base


*/

#ifndef F_CPU
#define F_CPU 8000000UL // 8 MHz clock speed
#endif

#define RodaKiri  PB3	// OCR1B pin 4
#define RodaKanan PB1	// OCR1A pin2

//sensor utk ADC konversi
#define s0 0
#define s1 1
#define s2 2
#define s3 3
#define s4 4
#define s5 5

#define LF_kec 130 // PWM normal ideal

#define SET(port,pin) PORT ## port |= (1<<pin)
#define CLEAR(port,pin) PORT ## port &= ~(1<<pin)

#include <avr/io.h>
#include <util/delay.h>

void adc_setup (void)
{	
	ACSR =(1<<ACD); //disable analog comparator
	
	ADMUX=(0<<REFS0) |(0<<REFS1);                                 // For Aref=AVcc;
	
	ADMUX |= (1 << ADLAR); // Comment out for 10-bit resolution
		
	// Set the prescaler to clock/128 & enable ADC	
	ADCSR=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //Prescalar div factor =128, page 103-104
}

int adc_read (uint8_t ch)
{
	ACSR =(1<<ACD); //disable analog comparator
	// select the corresponding channel 0~7
	// ANDing with '7' will always keep the value
	// of 'ch' between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing
	
	//ADMUX |= (1 << MUX0);//ch 1, default
	
	
	ADCSR |= (1 << ADSC);// Start the conversion
	while (ADCSR & (1 << ADSC));// Wait for it to finish
	
	return ADCH;	//8 bit, 0-255
}

void pwm_setup (void){
	
	// Set Timer1 prescaler to PCK/16 for a 15 KHz overflow frequency of 8-bit counter
	TCCR1B |= (1<<CS12) | (1<<CS10);

	// Configure Timer1 for PWM on PWM1A and PWM1B without inverted outputs, hanya PB1, PB3
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);

	// Use maximum number of bits for speed resolution. We could finetune PWM frequency by
	// setting a lower number here but then we would not get 8 bits resolution on speed
	OCR1C = 0xff; //val utk 8 bit

	TCCR1A |= (1<<PWM1A) | (1<<PWM1B);
}

void maju(uint8_t kiri_PWM, uint8_t kanan_PWM) {
	OCR1A = kiri_PWM; //pwm speed untuk roda kiri
	OCR1B = kanan_PWM; //pwm speed untuk roda kanan
}

int main(void)
{
	DDRB=0b11111111;// Port B untuk output semua
			
	//tunggu
	_delay_ms(4000);
	
	adc_setup();
	pwm_setup();
	
	//ambil nilai sensor
	uint8_t putih_s0=0;
	uint8_t putih_s1=0;
	uint8_t putih_s2=0;
	uint8_t putih_s3=0;
	uint8_t putih_s4=0;
	uint8_t putih_s5=0;
	
	
	uint8_t NilaiMedanPutihMin=255;
	uint8_t NilaiMedanPutihMax=0;
	uint8_t NilaiMedanPutihMid=128;
	
	//variabel u status sensor
	uint8_t ST_s0=0;
	uint8_t ST_s1=0;
	uint8_t ST_s2=0;
	uint8_t ST_s3=0;
	uint8_t ST_s4=0;
	uint8_t ST_s5=0;
	
	uint8_t map_s=0b00000000;
	
	//PID
	int error=0;
	int error1=0;
	int nil_pid; 
	
	//motor
	int kecMKiri=0;
	int kecMkanan=0;
	
	//tuning disini
	int Kp = 20;
	int Ki = 0;
	int Kd = 5;
	int Ts= 15;
	
	int sum_error=0;
	
	// menentukan mid treshold
	// awalnya min value=255 nanti ngecilin, max val awalnya 0 nanti membesar
	// threshold = min value + (max value - min value) /2
	
	
	//kalibrasi cari treshold dari putih dan hitam
	//pin1 hidup utk s0, s1, s2 :B,0
	//pin3 hidup utk s3, s4, s5	:B,2
	
	for(uint8_t i=0;i<10;i++){
		SET(B,0);//pin 1
		putih_s0=adc_read(s0);		
		putih_s1=adc_read(s1);		
		putih_s2=adc_read(s2);
		CLEAR(B,0);
		
		SET(B,2);//pin 3
		putih_s3=adc_read(s3);		
		putih_s4=adc_read(s4);		
		putih_s5=adc_read(s5);
		CLEAR(B,2);
		
		//minimum
		if(putih_s0 < NilaiMedanPutihMin) {NilaiMedanPutihMin=putih_s0;}
		if(putih_s1 < NilaiMedanPutihMin) {NilaiMedanPutihMin=putih_s1;}
		if(putih_s2 < NilaiMedanPutihMin) {NilaiMedanPutihMin=putih_s2;}
		if(putih_s3 < NilaiMedanPutihMin) {NilaiMedanPutihMin=putih_s3;}
		if(putih_s4 < NilaiMedanPutihMin) {NilaiMedanPutihMin=putih_s4;}
		if(putih_s5 < NilaiMedanPutihMin) {NilaiMedanPutihMin=putih_s5;}	
				
		//max
		if(putih_s0 > NilaiMedanPutihMax) {NilaiMedanPutihMax=putih_s0;}
		if(putih_s1 > NilaiMedanPutihMax) {NilaiMedanPutihMax=putih_s1;}
		if(putih_s2 > NilaiMedanPutihMax) {NilaiMedanPutihMax=putih_s2;}
		if(putih_s3 > NilaiMedanPutihMax) {NilaiMedanPutihMax=putih_s3;}
		if(putih_s4 > NilaiMedanPutihMax) {NilaiMedanPutihMax=putih_s4;}
		if(putih_s5 > NilaiMedanPutihMax) {NilaiMedanPutihMax=putih_s5;}	
		
				
		_delay_ms(150);
	}
	NilaiMedanPutihMid = NilaiMedanPutihMin +(NilaiMedanPutihMax-NilaiMedanPutihMin)/2;
	
	
	
    while(1)
    {		
        _delay_ms(Ts);
					
		//reset nilai adc sensor
		putih_s0=0;
		putih_s1=0;
		putih_s2=0;
		putih_s3=0;
		putih_s4=0;
		putih_s5=0;
		
		SET(B,0);//pin 1
		putih_s0=adc_read(s0);
		putih_s1=adc_read(s1);
		putih_s2=adc_read(s2);
		CLEAR(B,0);
		
		SET(B,2);//pin 3
		putih_s3=adc_read(s3);
		putih_s4=adc_read(s4);
		putih_s5=adc_read(s5);
		CLEAR(B,2);
		
		//reset status sensor, 0=hitam, 1=putih
		ST_s0=0;
		ST_s1=0;
		ST_s2=0;
		ST_s3=0;
		ST_s4=0;
		ST_s5=0;
		
				
		//status map sensor
		if(putih_s0 > NilaiMedanPutihMid){ST_s0=1;}
		if(putih_s1 > NilaiMedanPutihMid){ST_s1=1;}
		if(putih_s2 > NilaiMedanPutihMid){ST_s2=1;}
		if(putih_s3 > NilaiMedanPutihMid){ST_s3=1;}
		if(putih_s4 > NilaiMedanPutihMid){ST_s4=1;}
		if(putih_s5 > NilaiMedanPutihMid){ST_s5=1;}
			
		//Logik PID atau PD
		//
		//0b00000000
		//center
		//0b00 110011
		
		map_s =(ST_s0<<5)|(ST_s1<<4)|(ST_s2<<3)|(ST_s3<<2)|(ST_s4<<1)|(ST_s5<<0);
		
		if(map_s==0b00110011){
			error=0;
		}
		else if(map_s==0b00111110){
			error=-3;
		}
		else if(map_s==0b00111001){
			error=-2;
		}
		else if(map_s==0b00111011){
			error=-1;
		}
		else if(map_s==0b00110111){
			error=1;
		}
		else if(map_s==0b00011111){
			error=2;
		}
		else if(map_s==0b00100111){
			error=3;
		}
		else if(map_s==0b00111111){
			//putih semua
			//bandingkan dengan error sebelumnya, cenderung kemana?
			if(error<0){
				error= -4;
			}
			else if(error>0){
				error= 4;
			}
		}
		
		//PID formula		
		
		//P
		nil_pid=Kp * error;
		
		//I
		Ki=0;
		sum_error +=error;
		nil_pid+=Ki * sum_error;		
		
		//D
		nil_pid+=Kd * (error-error1);		
		
		error1=error;
		
		//
		kecMKiri=LF_kec + nil_pid;
		kecMkanan=LF_kec - nil_pid;
		
		if(kecMKiri>255){kecMKiri=255;}
		if(kecMKiri<0){kecMKiri=0;}	
		
		if(kecMkanan>255){kecMkanan=255;}	
		if(kecMkanan<0){kecMkanan=0;}	
			
		maju(kecMKiri,kecMkanan);
		
		/*TEST
		if(error<0){			
			maju(0,LF_kec);
		}
		else{
			maju(LF_kec,0);
		}
		*/
		
    }
}