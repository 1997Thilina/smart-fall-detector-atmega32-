


#define F_CPU 8000000UL									/* Define CPU clock Frequency e.g. here its 8MHz */
#include <avr/io.h>										/* Include AVR std. library file */
#include <util/delay.h>									/* Include delay header file */
#include <inttypes.h>									/* Include integer type header file */
#include <stdlib.h>										/* Include standard library file */
#include <stdio.h>										/* Include standard library file */
#include <math.h>		

#define XG_OFFS_TC 0x00
#define YG_OFFS_TC 0x01
#define ZG_OFFS_TC 0x02
#define X_FINE_GAIN 0x03
#define Y_FINE_GAIN 0x04
#define Z_FINE_GAIN 0x05
#define XA_OFFS_H 0x06 
#define XA_OFFS_L_TC 0x07
#define YA_OFFS_H 0x08 
#define YA_OFFS_L_TC 0x09
#define ZA_OFFS_H 0x0A 
#define ZA_OFFS_L_TC 0x0B
#define XG_OFFS_USRH 0x13
#define XG_OFFS_USRL 0x14
#define YG_OFFS_USRH 0x15
#define YG_OFFS_USRL 0x16
#define ZG_OFFS_USRH 0x17
#define ZG_OFFS_USRL 0x18
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FF_THR 0x1D
#define FF_DUR 0x1E
#define MOT_THR 0x1F
#define MOT_DUR 0x20
#define ZRMOT_THR 0x21
#define ZRMOT_DUR 0x22
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define DMP_INT_STATUS 0x39
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define BANK_SEL 0x6D
#define MEM_START_ADDR 0x6E
#define MEM_R_W 0x6F
#define DMP_CFG_1 0x70
#define DMP_CFG_2 0x71
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75



#define SCL_CLK 100000L							                                           /* Define SCL clock frequency */
#define BITRATE(TWSR)	((F_CPU/SCL_CLK)-16)/(2*pow(4,(TWSR&((1<<TWPS0)|(1<<TWPS1)))))        /* Define bit rate */

#define BAUD_PRESCALE (((F_CPU / (BAUDRATE * 16UL))) - 1)	                              








////////////////////////gyro/ accelerometer /////////////////////////////////////////////////////////////




int fall = 0; //stores if a fall has occurred
int trigger1=0; //stores if first trigger (lower threshold) has occurred
int trigger2=0; //stores if second trigger (upper threshold) has occurred
int trigger3=0; //stores if third trigger (orientation change) has occurred
int trigger1count=0; //stores the counts past since trigger 1 was set true
int trigger2count=0; //stores the counts past since trigger 2 was set true
int trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;


void SendSms(void);

void I2C_Init();								/* I2C initialize function */
uint8_t  I2C_Start(char);						/* I2C start function */
uint8_t  I2C_Repeated_Start(char);				/* I2C repeated start function */
void I2C_Stop();								/* I2C stop function */
void I2C_Start_Wait(char);						/* I2C start wait function */
uint8_t  I2C_Write(char);						/* I2C write function */
char I2C_Read_Ack();							/* I2C read ack function */
char I2C_Read_Nack();							/* I2C read nack function */


float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;

void MPU6050_Init()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}




void I2C_Init()												/* I2C initialize function */
{
	TWBR = BITRATE(TWSR = 0x00);							/* Get bit rate register value by formula */
}	


uint8_t I2C_Start(char slave_write_address)						/* I2C start function */
{
	uint8_t status;											/* Declare variable */
	TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);					/* Enable TWI, generate start condition and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (start condition) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status != 0x08)										/* Check weather start condition transmitted successfully or not? */
	return 0;												/* If not then return 0 to indicate start condition fail */
	TWDR = slave_write_address;								/* If yes then write SLA+W in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */	
	if (status == 0x18)										/* Check weather SLA+W transmitted & ack received or not? */
	return 1;												/* If yes then return 1 to indicate ack received i.e. ready to accept data byte */
	if (status == 0x20)										/* Check weather SLA+W transmitted & nack received or not? */
	return 2;												/* If yes then return 2 to indicate nack received i.e. device is busy */
	else
	return 3;												/* Else return 3 to indicate SLA+W failed */
}

uint8_t I2C_Repeated_Start(char slave_read_address)			/* I2C repeated start function */
{
	uint8_t status;											/* Declare variable */
	TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);					/* Enable TWI, generate start condition and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (start condition) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status != 0x10)										/* Check weather repeated start condition transmitted successfully or not? */
	return 0;												/* If no then return 0 to indicate repeated start condition fail */
	TWDR = slave_read_address;								/* If yes then write SLA+R in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status == 0x40)										/* Check weather SLA+R transmitted & ack received or not? */
	return 1;												/* If yes then return 1 to indicate ack received */ 
	if (status == 0x20)										/* Check weather SLA+R transmitted & nack received or not? */
	return 2;												/* If yes then return 2 to indicate nack received i.e. device is busy */
	else
	return 3;												/* Else return 3 to indicate SLA+W failed */
}

void I2C_Stop()												/* I2C stop function */
{
	TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);					/* Enable TWI, generate stop condition and clear interrupt flag */
	while(TWCR & (1<<TWSTO));								/* Wait until stop condition execution */ 
}

void I2C_Start_Wait(char slave_write_address)				/* I2C start wait function */
{
	uint8_t status;											/* Declare variable */
	while (1)
	{
		TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);				/* Enable TWI, generate start condition and clear interrupt flag */
		while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (start condition) */
		status = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
		if (status != 0x08)									/* Check weather start condition transmitted successfully or not? */
		continue;											/* If no then continue with start loop again */
		TWDR = slave_write_address;							/* If yes then write SLA+W in TWI data register */
		TWCR = (1<<TWEN)|(1<<TWINT);						/* Enable TWI and clear interrupt flag */
		while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (Write operation) */
		status = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
		if (status != 0x18 )								/* Check weather SLA+W transmitted & ack received or not? */
		{
			I2C_Stop();										/* If not then generate stop condition */
			continue;										/* continue with start loop again */
		}
		break;												/* If yes then break loop */
	}
}

uint8_t I2C_Write(char data)								/* I2C write function */
{
	uint8_t status;											/* Declare variable */
	TWDR = data;											/* Copy data in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status == 0x28)										/* Check weather data transmitted & ack received or not? */
	return 0;												/* If yes then return 0 to indicate ack received */
	if (status == 0x30)										/* Check weather data transmitted & nack received or not? */
	return 1;												/* If yes then return 1 to indicate nack received */
	else
	return 2;												/* Else return 2 to indicate data transmission failed */
}

char I2C_Read_Ack()										/* I2C read ack function */
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);					/* Enable TWI, generation of ack and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (read operation) */
	return TWDR;											/* Return received data */
}	

char I2C_Read_Nack()										/* I2C read nack function */
{
	TWCR=(1<<TWEN)|(1<<TWINT);								/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (read operation) */
	return TWDR;											/* Return received data */
}	


/*float get_g(float x, float y, float z){
	float acc1 = sqrt (x*x + y*y + z*z);
	return acc1;
}
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////GSM //////////////////////////////////////////////////////////////



void USARTInit(uint16_t ubrr_value)
{

      
   UBRRL = ubrr_value;
   UBRRH = (ubrr_value>>8);
   
   //UCSRC|=(1<<URSEL);
   UCSRC|=(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);


   UCSRB=(1<<RXEN)|(1<<TXEN);


}
unsigned char USARTReadChar()
{


   while(!(UCSRA & (1<<RXC)));



   return UDR;
}

void USARTWriteChar(char data)
{

   while(!(UCSRA & (1<<UDRE)));
   UDR=data;
}

void USART_StringTransmit(unsigned char s[])
{
int i=0;
while(s[i]!='\0')
{
USARTWriteChar(s[i]);
i++;
}
}

void SendSms(){     // sending sms


	USARTWriteChar('A');
		USARTWriteChar('T');
		USARTWriteChar('+');
		USARTWriteChar('C');
		USARTWriteChar('M');
		USARTWriteChar('G');
		USARTWriteChar('F');
		USARTWriteChar('=');
		USARTWriteChar('1');
		USARTWriteChar('\n\r');


		_delay_ms(2000);
	
	USARTWriteChar('A');
	USARTWriteChar('T');
	USARTWriteChar('+');
	USARTWriteChar('C');
	USARTWriteChar('M');
	USARTWriteChar('G');
			USARTWriteChar('S');
	USARTWriteChar('=');
	USARTWriteChar('"');
	USARTWriteChar('+');
	USARTWriteChar('9');
	USARTWriteChar('4');
	USARTWriteChar('7');
		USARTWriteChar('1');
	USARTWriteChar('7');
	USARTWriteChar('7');
	USARTWriteChar('5');
	USARTWriteChar('0');
	USARTWriteChar('6');
	USARTWriteChar('9');
		USARTWriteChar('4');
	USARTWriteChar('"');
	USARTWriteChar('\n\r');
	_delay_ms(1000);
	
	
}

////////////////////////////////////////////////////////////////////////////////////////





int main()
{
	DDRB = 0xff;
	DDRA = 0x00;
	
	PORTB=0x08;
	
	_delay_ms(1000);
	USARTInit(51);  
	_delay_ms(6000);

		
	USARTWriteChar('A');
	_delay_ms(100);
	USARTWriteChar('T');
	_delay_ms(500);
	USARTWriteChar('\n\r');
	_delay_ms(1000);
	  
	  
	//char buffer[20], float_[10];
	float Xa,Ya,Za,t;
	float gx=0,gy=0,gz=0;
	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();										/* Initialize MPU6050 */
	
	SendSms();
	USART_StringTransmit(" Device working..   ");
	
	USARTWriteChar(26);
	USARTWriteChar('\n\r');
	


	

	while(1)
	{
		Read_RawValue();

		Xa = Acc_x/16384.0;								/* Divide raw value by sensitivity scale factor to get real values */
	   	Ya = Acc_y/16384.0;
		Za = Acc_z/16384.0;
		
		gx = Gyro_x/16.4;
		gy = Gyro_y/16.4;
		gz = Gyro_z/16.4;

		//t = (Temperature/340.00)+36.53;	
		
		//get_g(float xa,float  ya,float za);
		
		float AM = pow(pow(Xa,2)+pow(Ya,2)+pow(Za,2),0.5);
		int Amp  = AM*9.8;
		
		if (Amp<=4 && trigger2==0){         //if AM breaks lower threshold (0.4g)
			trigger1=1;                         // "TRIGGER 1 ACTIVATED"
	    
	   }
		if (trigger1==1){
			trigger1count++;
			if (Amp>=20){            //if AM breaks upper threshold (3g)
				trigger2=1;           //"TRIGGER 2 ACTIVATED"
				trigger1=0; 
				trigger1count=0;
		    }
	  }
	  
	 if (trigger2==1){
		trigger3count++;
		if (trigger3count>=10){ 
			fall=1; 
			trigger2=0; 
			trigger2count=0;
		}   
		  
	  }
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  /*if (trigger2==1){
	  
	   trigger2count++;
	   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); 
	   if (angleChange>=30 && angleChange<=400){              //if orientation changes by between 80-100 degrees
		 trigger3=1;
		 trigger2=0; 
		 trigger2count=0;   // "TRIGGER 3 ACTIVATED"
            
		 
		   }
	   }
	 if (trigger3==1){
		trigger3count++;
		if (trigger3count>=10){ 
		   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
		   
		   
		   if (angleChange>=0 && angleChange <= 40){ //if orientation changes remains between 0-40 degrees
			   fall=1; trigger3=0; trigger3count=0;
			   
				 }
				 
		   else{                 //user regained normal orientation
			  trigger3=0;            // "TRIGGER 3 DEACTIVATED"
			  trigger3count=0;
			  
		   }
		 }
	  }
	  
	  
	  */
	  
	  
	  
	  
	  
	 int i = 0;
	  
	 if (fall==1){                   //in event of a fall detection
		while(PINA & (1<<2) ) {            //  PINA2  switch
			PORTB=0xf0;
			i++;
			
			if(i<=11 && i>10){
				SendSms();
				USART_StringTransmit(" Alert! The person who was wearing the device fell down   ");
				USARTWriteChar(26);
				USARTWriteChar('\n\r');
			}
			_delay_ms(1000);
	 
		}
		
		
	    fall=0;
		trigger3count=0;
		PORTB=0x08;
		
	  }
	   
	   
	 if (trigger2count>=8){        //allow 0.5s for orientation change
	   trigger2=0; 
	   trigger2count=0;
	  } 
	   
	   
	 if (trigger1count>=8){      //allow 0.5s for AM to break upper threshold
	   trigger1=0;
		trigger1count=0;       // TRIGGER 1 DECACTIVATED
	  
	   }
	  _delay_ms(100);
	  
	  
	  ///////////// Heartbeat////////////////////////////
	  
	  if (PINA & (1<<3) ){
	  }
	  
	  else {           // PINA3 button
		PORTB = 0x07;
		_delay_ms(2000);
		unsigned char k =0;
		unsigned char m =0;
		while(k<=150){
			//PORTB = 0x03;
			k++;
			PORTB = PORTB & (~(1<<2));      // PINB0 or PINB1 IR
			if(PINA & (1<<4)){       // pinA4 pulse sensor
				m++;
				PORTB = PORTB | (1<<2);   // PINB2 BLUE LED
			}
		_delay_ms(100);
		}
		unsigned char hb = (m/15)*60;
		
		SendSms();
			if (hb>10){
				USART_StringTransmit(" BPM:   " );
				//USART_StringTransmit(hb +"   ");
				//USART_StringTransmit();
				USARTWriteChar(hb);
			}
			else{
				USART_StringTransmit(" Heartbeat measuring error!!!   ");
				
			}
		
		USARTWriteChar(26);
		USARTWriteChar('\n\r');
		PORTB = 0x08;

		}
	  
	   
		  
	}
	
}