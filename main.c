 #define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/interrupt.h>
#include <avr/io.h>

#include <util/delay.h>
void USART_Init( unsigned int ubrr);
unsigned char USART_Read();
void USART_Transmit( unsigned char data );


void Color_Init(void);
void timerInitColor(void);
void timerInitColor2(void);

void buz_Init(void);
int Color_Read(char channel);

////////////////////////////////////////////////////////////////////////////////////////////
int check=0;
int onRoad=0;
int endd=0;
int cnt=0;
int offed=0;
int e=0;
int flag=0;

int red;
int blue;
int green;
int coll;
int colAvg [2] = {0,0};
float avgGrnDif=0;
float avgRedDif=0;
float avgBlueDif=0;

////////////////////////////////////////////////////////////////////////////////////////////

void initFastPwm(unsigned int d){
  DDRB |= 1<< PB1 | 1<< PB2;
  DDRD|= 1<< PD6 | 1<<PD5;
  TCCR1A = 1<<COM1A1 | 1<<COM1B1 | 1<<WGM11 ;    
  TCCR1B = 1<< WGM13 | 1<< WGM12| 1<<CS11;  
  ICR1=d;
}

void initIRSensor(){
  DDRC =0x00;
  DDRD |=0<<PD4; 
  DDRD |=0<<PD3;
	DDRB |=1<<PB0;
  
}

void fastPwm(unsigned char i,unsigned int p){
 
	if(i==0){
  	OCR1A = p;
	}
	else if(i==1){
  	OCR1B = p;
	}
 
}

void go(unsigned char c,unsigned int s){
  if(c==0){
	PORTD = 1<< PD6 | 1<<PD5;
	fastPwm(0,20000-s);
	fastPwm(1,20000-s);
  }
  else if(c==1){
	PORTD = 0<< PD6 | 0<<PD5;
	fastPwm(0,s);
	fastPwm(1,s);
    
    
  }
}

void turn(unsigned char t,unsigned int s){
 
  //PORTD = 0b01100000;
 
  if(t==0){
	fastPwm(1,s);
	fastPwm(0,18000);
  }
  else if(t==1){
	fastPwm(0,s);
	fastPwm(1,18000);
  }
 
 
}

////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////



static volatile int pulse = 0;
static volatile int i = 0;

ISR(INT0_vect)
{
	if(i == 1)
	{
		TCCR0B = 0;
		pulse = TCNT0;
		TCNT0 = 0;
		i = 0;
	}

	if(i==0)
	{
		
         TCCR0B |= (1 << CS02) |  (1<<CS00 );
		i = 1;
	}

}

char string[10];
long count;
double distance;


void initSensor(){
	
	
(DDRD) &= ~(1<<2);
	DDRD |=(1<<7);
	_delay_ms(50);

	//Initialise();

	EICRA |= (1 << ISC00);    // set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0);     // Turns on INT0

	sei();                    // turn on interrupts


}

int getDistance(){
		(PIND) |= (1<<7);
		_delay_us(15);

		(PIND) &= ~(1<<7);
		
		int count_a = pulse/3.643;

		
		//USART_Transmit(count_a+10);
		if(onRoad==1){
			_delay_ms(80);	
		
		}
		return count_a;
}

////////////////////////////////////////////////////////////////////////////////////////////
int isFinish=0;
////////////////////////////////////////////////////////////////////////////////////////////
long irVals[8] = {0,0,0,0,0,0,0,0};
void goOnLine(){

		int ar[8] = {0,0,0,0,0,0,0,0};
		int sum=0;
		int c=1;
		
			
			if(PINC & (1<<1)){
				//USART_Transmit(flag+2);

				ar[1]=3;
				irVals[1]+=1;
				//_delay_ms(1);
			}
			if(PINC & (1<<2)){
				//USART_Transmit(flag+3);
				ar[2]=2;
				irVals[2]+=1;
				//_delay_ms(1);
			}
			if(PINC & (1<<3)){
				//USART_Transmit(flag+4);
				ar[3]=1;
				irVals[3]+=1;
				//_delay_ms(1);
			}

			if(PINC & (1<<4)){
				//USART_Transmit(flag+5);
				ar[4]=-1;
				irVals[4]+=1;
				//_delay_ms(1);
			}
			if(PINC & (1<<5)){
				//USART_Transmit(flag+6);
				ar[5]=-2;
				irVals[5]+=1;
				//_delay_ms(1);
			}
			if(PIND & (1<<3)){
				//USART_Transmit(flag+7);
				ar[6]=-3;
				irVals[6]+=1;
				//_delay_ms(1);
			}
			if(PIND & (1<<4)){
				//USART_Transmit(flag+8);
				ar[7]=-4;
				irVals[7]+=1;
				
				//_delay_ms(1);
				check=2;
			}
			
			if(PINC & (1<<0)){
				//USART_Transmit(flag+1);
				ar[0]=4;
				irVals[0]+=1;
				//check=1;
				//_delay_ms(1);
			}

			for(int i=0;i<8;i++){
				
				sum+=ar[i];
			
			}
			
			
			if(sum==0){
				go(1,18000);
			}
			else if(sum>0 && sum<=6){
				
				turn(1,3000);
				
			}else if(sum>6){

				turn(1,1000);
			}
			else if(sum<0 && sum>=-6){
				turn(0,3000);
			}else if(sum<-6){
				turn(0,1000);
			}
			//USART_Transmit(cnt);
			
			if( PINC==0b00111111 &&(PIND &(1<<3)) &&(PIND &(1<<4)) && c==1 && onRoad==0){
				if(cnt==0){
					go(1,15000);
					_delay_ms(900);
				}
				else if(cnt==1){
					go(1,15000);
					_delay_ms(1200);
				}
				else if(cnt==2){
					go(1,15000);
					_delay_ms(500);
				}
				else if(cnt==3){
					turn(1,0);
					_delay_ms(1000);
				}
				else if(cnt==4){
					turn(1,0);
					_delay_ms(1700);
				}
				else if(cnt==5){
					turn(1,0);
					_delay_ms(550);
				}
				else if(cnt==6){
					go(1,15000);
					_delay_ms(500);
				}
				
				
				cnt+=1;
				c=0;
			}
			if( PINC==0b00000001 &&(~PIND &(1<<3)) &&(~PIND &(1<<4)) && onRoad==1 && endd==1){
				
				turn(1,0);
				_delay_ms(2000);
				go(1,15000);
				_delay_ms(1000);
				go(1,0);
				USART_Transmit(9);
				float maxx=0;

				for(int f=0;f<8;f++){
					if(irVals[f]>maxx){
						maxx=irVals[f];
					}
				}
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);
				USART_Transmit(0);

				USART_Transmit( 200-(irVals[0]/maxx)*200 );
				USART_Transmit( 200-(irVals[0]/maxx)*200 );
				USART_Transmit( 200-(irVals[0]/maxx)*200 );

				USART_Transmit( 200-(irVals[1]/maxx)*200 );
				USART_Transmit( 200-(irVals[1]/maxx)*200 );
				USART_Transmit( 200-(irVals[1]/maxx)*200 );

				USART_Transmit( 200-(irVals[2]/maxx)*200 );
				USART_Transmit( 200-(irVals[2]/maxx)*200 );
				USART_Transmit( 200-(irVals[2]/maxx)*200 );
				
				USART_Transmit( 200-(irVals[3]/maxx)*200 );
				USART_Transmit( 200-(irVals[3]/maxx)*200 );
				USART_Transmit( 200-(irVals[3]/maxx)*200 );

				USART_Transmit( 200-(irVals[4]/maxx)*200 );
				USART_Transmit( 200-(irVals[4]/maxx)*200 );
				USART_Transmit( 200-(irVals[4]/maxx)*200 );

				USART_Transmit( 200-(irVals[5]/maxx)*200 );
				USART_Transmit( 200-(irVals[5]/maxx)*200 );
				USART_Transmit( 200-(irVals[5]/maxx)*200 );

				USART_Transmit( 200-(irVals[6]/maxx)*200 );
				USART_Transmit( 200-(irVals[6]/maxx)*200 );
				USART_Transmit( 200-(irVals[6]/maxx)*200 );

				USART_Transmit( 200-(irVals[7]/maxx)*200 );
				USART_Transmit( 200-(irVals[7]/maxx)*200 );
				USART_Transmit( 200-(irVals[7]/maxx)*200 );
				
				
				isFinish=1;
			}
			

		
}

////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////


int main(void){
	initFastPwm(20000);
	USART_Init(MYUBRR);
	initIRSensor();
	int ultraAvg [8] = {0,0,0,0,0,0,0,0,0,0};

	initSensor();//ultra

	Color_Init();
	buz_Init();
	timerInitColor2();
	int u=0;
	while(1){ 
		
		if(onRoad==1){
			if(u<5){
			ultraAvg[u]=getDistance();	
			u+=1;
			}else if(u==5){
			
			if( (ultraAvg[0]+ultraAvg[1]+ultraAvg[2]+ultraAvg[3]+ultraAvg[4])/5 <15){
				USART_Transmit(100);
				
			
			}
			else if((ultraAvg[0]+ultraAvg[1]+ultraAvg[2]+ultraAvg[3]+ultraAvg[4])/5 >= 15){
				USART_Transmit(0);

			}
			u=0;
			}
		
		}
		
		//USART_Transmit(getDistance());

		goOnLine();
		
			//USART_Transmit(cnt);

		if(isFinish==1){
			break;
		}
		
	}
   
	
   
	return 0;
}
long countt=0;
long ccc=0;

ISR (TIMER2_COMPA_vect)
{
    // action to be done every 250 usec
    countt+=1;
	ccc+=1;
	if(ccc%3300==0){
		
		onRoad=1;
	}
	if(ccc%5000==0){
		endd=1;
	}

	if(countt%(10)==0){
		if(PINB & (1<<3)){
			PORTB &=~(1<<3);//off red
			PORTB |= (1<<5);//on green
			green=Color_Read(6);
		}else{
			PORTB &=~(1<<5);//off green
			PORTB |= (1<<3);//on red
			red=Color_Read(6);
		}
		
		PORTB &=~(1<<4);//OFFING BUZZER
		
		
		int colll=0;
		coll=red-green;
			if(e<2){
				colAvg[e]=coll;	
				e+=1;
			}else if(e==2){
			
				colll=( colAvg[0]+colAvg[1])/2;
				//USART_Transmit(colll);
			e=0;
			}

		
		
		
		//_delay_ms(100);
		if(colll>-126 && colll<-49)
		{
			//PORTD |= (1<<7);  //RED color indicates (PD7)
			 //BUZER!!
			PORTB |= (1<<4);
			USART_Transmit(80);
			//PORTB &=~(1<<4);
			//USART_Transmit(40);
			//_delay_ms(30);

			//USART_Transmit(50);
			//USART_Transmit(red);
		}
		
		else if(colll>10 && colll<40)
		{
			//PORTB &=~(1<<4);//OFFING BUZZER
			//PORTB |= (1<<4);
			USART_Transmit(120);
			//_delay_ms(30);
			//USART_Transmit(50);
			//PORTD |= (1<<6);  //GREEN color indicates (PD6)
			//USART_Transmit(green);
			//USART_Transmit(60);
		}
		else{
			//USART_Transmit(60);
		}
	}
}


int Color_Read(char channel)
{
	int Ain,AinLow;
	
	ADMUX=ADMUX|(channel & 0x0f);	/* Set input channel to read */

	ADCSRA |= (1<<ADSC);		/* Start conversion */
	while((ADCSRA&(1<<ADIF))==0);	/* Monitor end of conversion interrupt */
	
	_delay_us(10);
	AinLow = (int)ADCL;		/* Read lower byte*/
	Ain = (int)ADCH*256;		/* Read higher 2 bits and 
					Multiply with weight */
	Ain = Ain + AinLow;				
	return(Ain*2);			/* Return digital value*/
}
void timerInitColor2(){

	OCR2A = 0xF9;

    TCCR2A |= (1 << WGM21);
    // Set to CTC Mode

    TIMSK2 |= (1 << OCIE2A);
    //Set interrupt on compare match

    TCCR2B |= (1 << CS21) | (1 << CS22);
    // set prescaler to 64 and starts PWM

    sei();
    // enable interrupts

}
/*
void timerInitColor()
{
	// Set the Timer Mode to CTC
    TCCR0A |= (1 << WGM01);

    // Set the value that you want to count to
    OCR0A = 0xF9;

    TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect

    sei();         //enable interrupts


    TCCR0B |= (1 << CS02);
    // set prescaler to 256 and start the timer
}
*/

void Color_Init()
{
	
	DDRB |= 0b00101000;     /* Make PD2,3,4 ports as output for RGB LED generator and Make PD5,6,7 ports as output for RGB LED output */
	//DDRA &=~(1<<6);			/* Make ADC port as input for give LDR readings */
	ADCSRA = 0x87;			/* Enable ADC, fr/128  */
	ADMUX = 0x46;			/* Vref: Avcc(5V), ADC channel: 6 */
	

}

void buz_Init()
{
	DDRB |= 0b00010000;     /* Make PD2,3,4 ports as output for RGB LED generator and Make PD5,6,7 ports as output for RGB LED output */
	
}


void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	/* UBRR0H contains the 4 most significant bits of the
	baud rate. UBRR0L contains the 8 least significant
	bits.*/  
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
    

	/*Enable transmitter */
	UCSR0B = (1<<RXEN0)  |(1<<TXEN0);
    
	/* Set frame format: 8data */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
    
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_Read()
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<RXC0)) );
    
	/* Put data into buffer, sends the data */
	//UDR0 = data;
	return UDR0;
}

