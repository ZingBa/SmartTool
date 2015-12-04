#include "functions.h"
#include "stm32f4_discovery.h"
#include "alphaNum.h"

//system delay
static __IO uint32_t TimingDelay;
//spi accelerometer
volatile int8_t xVal = 0;
volatile int8_t yVal = 0;
//distance sensor
int value;
double valueInCm;
int preciseValue;
double precVal[NUMBER_OF_READINGS];
int valPriority[NUMBER_OF_READINGS];

/*      INITIALIZE ALL FUNCTIONS        */
void init()
{
	//accelerator
	init_GPIOA_SPI();
	init_GPIOE_SPI();
	init_accel();
	start_accel();
	
	//distance sensor
	initADC();
	
	//display
	init_GPIOB();
	init_GPIOD();
	init_GPIOE_OUT();
	init_TIM4();
	init_LCD_protocol();
}

/*		SPI		*/
uint8_t SPI1_Read()
{ 
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){};
  
  SPI_I2S_SendData(SPI1, 0x00);
  
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){};
  
  return SPI_I2S_ReceiveData(SPI1);
}

void SPI1_Write(uint8_t data)
{  
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET){};
  
  SPI_I2S_SendData(SPI1, data);
  
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){};
  
  SPI_I2S_ReceiveData(SPI1);
}

void init_GPIOA_SPI()
{
  //enable clock for button
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  //alternate function
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  
  
} 

void init_GPIOE_SPI()
{
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  //GPIOE
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/*		ACCELERATOR		*/
void init_accel()
{
  SPI_InitTypeDef SPI_InitStruct;

  //clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7; 
  SPI_Init(SPI1, &SPI_InitStruct);
    
  SPI_Cmd(SPI1, ENABLE);
}

void start_accel()
{
  //vklopimo senzor
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  SPI1_Write(0x20);
  SPI1_Write(0x47);  
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

int8_t read_accel_x()
{
     //x os
	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
        
	//ker beremo moramo nastavit prvi bit na 1 iz 0x29 dobimo 0xA9
	SPI1_Write(0xA9); 
	
	xVal = SPI1_Read();  
	
	if(xVal > 3)
	{
	  GPIO_SetBits(GPIOD, GPIO_Pin_14);
	  GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	}
	if(xVal >= -3 && xVal <= 3)
	{
	  GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_14);
	}
	if(xVal < -3)
	{
	  GPIO_SetBits(GPIOD, GPIO_Pin_12);
	  GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	}
	
	GPIO_SetBits(GPIOE, GPIO_Pin_3);    
	
        Delay_ms(100);
        
	return xVal;
}
int8_t read_accel_y()
{
  //y os
	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
	 
        //ker beremo moramo nastavit prvi bit na 1 iz 0x2B dobimo 0xAB
	SPI1_Write(0xAB);
	
	yVal = SPI1_Read();
	 
	if(yVal > 3)
	{
	  GPIO_SetBits(GPIOD, GPIO_Pin_13);
	  GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	}
	if(yVal >= -3 && yVal <= 3)
	{
	  GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_15);
	}    
	if(yVal < -3)
	{
	  GPIO_SetBits(GPIOD, GPIO_Pin_15);
	  GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}
	
	GPIO_SetBits(GPIOE, GPIO_Pin_3);
		
        Delay_ms(100);
        
        return yVal;
}

void update_accel(uint8_t ux, uint8_t uy)
{
  int n = 0;
  
  int frst = 0;
  int scnd = 0;
  
        if(ux > 0x02 && ux  < 0x7E)
	{
	  n++;
          frst = LEFT;
	}
	else if(ux > 0x80 && ux < 0xFD)
	{
	  n++;
          frst = RIGHT;
	}
        
	//y os	 
	if(uy > 0x02 && uy  < 0x7E)
	{
          if(n == 0)
          {
              n++;
              frst = DOWN;
          }
          else
          {
              n++;
              scnd = DOWN;
          }
	}    
	else if(uy > 0x80 && uy < 0xFD)
	{
	  if(n == 0)
          {
              n++;
              frst = UP;
          }
          else
          {
              n++;
              scnd = UP;
          }
	}
  
        //react to measurement
        
        if(n == 0)
        {
          //balanced
          printBalanced();
        }
        else if(n == 1)
        {
          //correct one
          printCorrectOne(frst);
        }
        else if(n == 2)
        {
          //correct two
          printCorrectTwo(frst, scnd);
        }     
        
}
/*		DISTANCE SENSOR		*/
void initADC()
{
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
   
   // PORT C - ANALOG INPUT
  GPIO_InitTypeDef GPIOC_InitStruct;
  GPIOC_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
   
  // ADC Common Initi Type
  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
  ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
  
  // ADC 1
  ADC_InitTypeDef ADC1_InitStruct;
  ADC1_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC1_InitStruct.ADC_ScanConvMode = DISABLE;
  ADC1_InitStruct.ADC_ContinuousConvMode = DISABLE;
  ADC1_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC1_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC1_InitStruct.ADC_NbrOfConversion = 1;
  
  // Init
  GPIO_Init(GPIOC, &GPIOC_InitStruct);
  ADC_CommonInit(&ADC_CommonInitStruct);
  ADC_Init(ADC1, &ADC1_InitStruct);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
  ADC_Cmd(ADC1, ENABLE);
 
}


int getMeasurement()
{
  //start adc conversion
    ADC_SoftwareStartConv(ADC1);
  //wait until it's done
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  //return read value      
   return ADC_GetConversionValue(ADC1);
}

double measurementToCm(int val)
{
  int values[NUMBER_OF_VALUES] = {3450, 2750, 2140, 1740, 1430, 1210, 1070, 940, 860, 780, 690, 640, 590, 500};
  int m = -1, a = 0, min = 0, max = 0;
  
  for(int i = 0; i < NUMBER_OF_VALUES; i++)
  {
    if(val < values[i])
    {   
      max = values[i];      
    }
    else
    {
      //stops when val is higher then values[i]
      m = ((i-1)*10);
      min = values[i];
      a = (max - min)/10;
      
      //make a loop to determine number of cm in 10ths (0.1 dm precision)
      for(int i = 0; i < 10; i++)
      {
        if(max > val)
        {
          max -= a;
        }
        else
        {
          m += i + 20;
          break;
        }
      }      
      break;
    }
  }
  
  return m;  
}

int preciseMeasurement()
{
  double precVal[NUMBER_OF_READINGS];
  double numValid = NUMBER_OF_READINGS;
  
  //capture data
  for(int i = 0; i < NUMBER_OF_READINGS; i++)
  {
        value = getMeasurement();
        valueInCm = measurementToCm(value);
        precVal[i] = valueInCm;
        Delay_ms(5);
  }

  double max = precVal[0]; double min = precVal[0];
  double avrg = 0;
  
  //calculate average value
  for(int i = 0; i < NUMBER_OF_READINGS; i++)
  {
      if(precVal[i] > 15 && precVal[i] < 155)
      {
        avrg += precVal[i];
        if(precVal[i] < min)      min = precVal[i];
        if(precVal[i] > max)      max = precVal[i];
      }
      else
      {
        numValid--;
      }
  }
  //calculate average value, removing min and max
  avrg = (avrg - max - min) / (numValid - 2);
  
  int returnVal = (int) avrg;
  
  if( !((returnVal > 15) && (returnVal < 155)) )
      returnVal = -1;
  
  return returnVal;
}


/*      LED LIGHTS      	*/
void init_GPIOD()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  //LEDS: P_12: GREEN;   P_13: ORANGE;   P_14: RED;   P_15: BLUE;   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*      LCD INIT       */
void init_GPIOB()       //RS, RW, EN
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;  
  GPIO_InitStructure.GPIO_Pin = RS | RW | EN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void init_GPIOE_OUT()   //DATA == WRITE
{
  //enable clock for button
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  //BUTTON:  USER: P_0;
  GPIO_InitStructure.GPIO_Pin = DB0 | DB1 | DB2 | DB3 | DB4 | DB5 | DB6 | DB7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void init_GPIOE_IN()   //ADDRESS == READ
{
  //enable clock for button
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  //BUTTON:  USER: P_0;
  GPIO_InitStructure.GPIO_Pin = DB0 | DB1 | DB2 | DB3 | DB4 | DB5 | DB6 | DB7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void init_LCD_protocol()
{
  //initialize via LCD protocol
  Delay_ms(20);
  
  //function set
  setRS_RW(0x0);
  setDATA(0x38); 
  enable_ms();
  Delay_ms(10);
  
  
   //function set
  setRS_RW(0x0);
  setDATA(0x38); 
  enable_ms();
  Delay_ms(5);
  
  //function set
  setRS_RW(0x0);
  setDATA(0x38); 
  enable_ms();
  Delay_ms(5);
   
  /*    CLEAR SCREEN        */
  setRS_RW(0x0);
  setDATA(0x01); 
  enable_ms();
  
  //from this step on, busy may be checked at any time
  dcb();        //display, cursor, blinking
  
  //create custom patterns
  createPatterns();
}

/*      LCD COMMS       */
//expected: hexa 0x3
void setRS_RW(int n)
{
  int s = 0x1;
  
  GPIO_ResetBits(GPIOB, RS | RW);
  
  if(n & s)             GPIO_SetBits(GPIOB, RW);
  if(n & s << 1)        GPIO_SetBits(GPIOB, RS);
}

void enable()
{
  GPIO_SetBits(GPIOB, EN);
  Delay_ms(5);
  GPIO_ResetBits(GPIOB, EN);
  Delay_ms(5);
}

void enable_ms()
{
  GPIO_SetBits(GPIOB, EN);
  Delay_ms(15);
  GPIO_ResetBits(GPIOB, EN);
  Delay_ms(15);
}

//expected: hexa 0x3F
void setDATA(int n)
{
  int s = 0x1;
  
  GPIO_ResetBits(GPIOE, DB0 | DB1 | DB2 | DB3 | DB4 | DB5 | DB6 | DB7); 
  
  if(n & s)             GPIO_SetBits(GPIOE, DB0);
  if(n & s << 1)        GPIO_SetBits(GPIOE, DB1);
  if(n & s << 2)        GPIO_SetBits(GPIOE, DB2);
  if(n & s << 3)        GPIO_SetBits(GPIOE, DB3);
  if(n & s << 4)        GPIO_SetBits(GPIOE, DB4);
  if(n & s << 5)        GPIO_SetBits(GPIOE, DB5);
  if(n & s << 6)        GPIO_SetBits(GPIOE, DB6);
  if(n & s << 7)        GPIO_SetBits(GPIOE, DB7);
}

void waitBusy()
{
  //change pin mode to READ
  init_GPIOE_IN();
  
  //wait for BUSY pin to clear
  setRS_RW(0x1);
  while( GPIO_ReadInputDataBit(GPIOE, DB7) );
  
  //chamge pin mode back to WRITE
  init_GPIOE_OUT();
}

//lcd printing
void clearDisplay()
{
  setRS_RW(0x0);
  setDATA(0x01); 
  enable_ms();
}
void dcb()
{
  setRS_RW(0x0);
  setDATA(0x0E); // 1|D|C|B => display on | cursor on | blinking on 
  enable();
}
void cursorLeft()
{
  setRS_RW(0x0);
  setDATA(0x10); 
  enable_ms();
}
void cursorRight()
{
  setRS_RW(0x0);
  setDATA(0x14); 
  enable_ms();
}
void displayLeft()
{
  setRS_RW(0x0);
  setDATA(0x18); 
  enable_ms();
}
void displayRight()
{
  setRS_RW(0x0);
  setDATA(0x1C); 
  enable_ms();
}
void lineOne()
{
  //range from 0x00 -> 0x27
  setRS_RW(0x0);
  setDATA(0x80); 
  enable_ms();
}
void lineTwo()
{
  //range from 0x40 -> 0x67
  setRS_RW(0x0);
  setDATA(0xC0); 
  enable_ms();
}
void printChar(int num)
{
  setRS_RW(0x2);
  setDATA(num); 
  enable();
}
void printCm(int num)
{
  int n1 = 0; int n0 = 0;
  if(num > 100)
  {
    printChar(ONE);
    num -= 100;
  }
  else
  {
    printChar(EMPTY);
  }
  
    n1 = (num / 10) + 0x30;
    n0 = (num % 10) + 0x30;
    
    printChar(n1);
    printChar(n0);
    printChar(EMPTY);
  
}
void printDm(int num)
{
  int n1 = 0; int n0 = 0;
  if(num > 100)
  {
      printChar(ONE);
    num -= 100;
    n1 = (num / 10) + 0x30;
      printChar(n1);
      printChar(COMMA);
    n0 = (num % 10) + 0x30;
      printChar(n0);
    
  }
  else
  { 
    n1 = (num / 10) + 0x30;
    printChar(n1);
    printChar(COMMA);
    n0 = (num % 10) + 0x30;   
    printChar(n0);
    printChar(EMPTY);
  } 
  
}
void printM(int num)
{
  int n1 = 0; int n0 = 0;
  if(num > 100)
  {
      printChar(ONE);
      printChar(COMMA);
    num -= 100;
    n1 = (num / 10) + 0x30;
      printChar(n1);
    n0 = (num % 10) + 0x30;
      printChar(n0);
    
  }
  else
  { 
      printChar(ZERO);
      printChar(COMMA);
    n1 = (num / 10) + 0x30;
      printChar(n1);
    n0 = (num % 10) + 0x30;
      printChar(n0);
  } 
  
}
void printUnit(int u)
{
  if(u == 1)
  {
    printChar(EMPTY);
    printChar(c_m);
  }
  else if(u == 2)
  {
    printChar(c_d);
    printChar(c_m);
  }
  else
  {
    printChar(c_c);
    printChar(c_m);
  }
}
//SPI accelerator
void printBalanced()
{
  //Delay_ms(100);
  lineTwo();
  //Delay_ms(100);
    printChar(EMPTY);printChar(EMPTY);printChar(c_V);printChar(EMPTY);
    printChar(c_r);printChar(c_a);printChar(c_v);printChar(c_n);printChar(c_o);
    printChar(c_v);printChar(c_e);printChar(c_s);printChar(c_j);printChar(c_u);
    printChar(EMPTY);printChar(EMPTY);
}
void printCorrectOne(int c)
{
  //Delay_ms(100);
  lineTwo();
  //Delay_ms(100);
    printChar(c_N);printChar(c_a);printChar(c_g);printChar(c_n);
    printChar(c_i);printChar(c_t);printChar(c_e);printChar(COLON);
      printChar(EMPTY);
    printChar(EMPTY);printChar(EMPTY);printChar(EMPTY);printChar(c);
    printChar(EMPTY);printChar(EMPTY);printChar(EMPTY);
}
void printCorrectTwo(int c1, int c2)
{
  //Delay_ms(100);
  lineTwo();
  //Delay_ms(100);
    printChar(c_N);printChar(c_a);printChar(c_g);printChar(c_n);
    printChar(c_i);printChar(c_t);printChar(c_e);printChar(COLON);
      printChar(EMPTY);
    printChar(EMPTY);printChar(c2);printChar(EMPTY);printChar(AND);
    printChar(EMPTY);printChar(c1);printChar(EMPTY);
}
//prepared Printing
void startUp()
{
  clearDisplay();
  //PRVI ZASLON
  lineOne();
  for(int i = 0; i < 16; i++)
  {
    printChar(FULL);
    Delay_ms(25);
  }
  lineOne();
  printChar(c_V);printChar(c_G);printChar(c_R);printChar(c_A);printChar(c_J);
  printChar(c_E);printChar(c_N);printChar(c_I);
    printChar(EMPTY);
  printChar(c_S);printChar(c_I);printChar(c_S);printChar(c_T);printChar(c_E);
  printChar(c_M);printChar(c_I);
  
  lineTwo();
  for(int i = 0; i < 16; i++)
  {
    printChar(FULL);
    Delay_ms(25);
  }
  lineTwo();
  printChar(c_B);printChar(c_u);printChar(c_l);printChar(c_i);printChar(c_c);
    printChar(EMPTY);printChar(AND);printChar(EMPTY);
  printChar(c_C);printChar(c_e);printChar(c_s);printChar(c_n);printChar(c_o);
  printChar(c_v);printChar(c_a);printChar(c_r);
  
  Delay_s(2);
  clearDisplay();
  
  //DRUGI ZASLON
  lineOne();
  printChar(EMPTY);printChar(c_P);printChar(c_a);printChar(c_m);printChar(c_e);
  printChar(c_t);printChar(c_n);printChar(c_o);
    printChar(EMPTY);
  printChar(c_o);printChar(c_r);printChar(c_o);printChar(c_d);printChar(c_j);
  printChar(c_e);printChar(EMPTY);
  
  lineTwo();
  printChar(AT);printChar(c_T);printChar(c_o);printChar(c_v);printChar(c_o);printChar(c_r);
  printChar(c_n);printChar(c_i);printChar(c_k);
    printChar(EMPTY);
  printChar(c_R);printChar(c_o);printChar(c_b);printChar(c_e);
  printChar(c_r);printChar(c_t);printChar(EMPTY);
  
  
}
void workScreen()
{
  
  for(int i = 0; i < 20; i++)
    displayRight();
  
  clearDisplay();
  
  lineOne();
  printChar(c_R);printChar(c_a);printChar(c_z);printChar(c_d);printChar(c_a);
  printChar(c_l);printChar(c_j);printChar(c_a);printChar(COLON);
    printChar(EMPTY);
  printChar(EMPTY);printChar(EMPTY);printChar(ZERO);printChar(EMPTY);printChar(c_c);
  printChar(c_m);
  
  printBalanced();
  
}

void dTemplate()
{
  //Delay_ms(100);
  lineOne();
  //Delay_ms(100);
  printChar(c_R);printChar(c_a);printChar(c_z);printChar(c_d);printChar(c_a);
  printChar(c_l);printChar(c_j);printChar(c_a);printChar(COLON);
    printChar(EMPTY);
  printChar(EMPTY);printChar(EMPTY);printChar(ZERO);printChar(EMPTY);printChar(c_c);
  printChar(c_m);
}

void updateScreen(int u, int m)
{
  //dTemplate();  

  //update temperature
  if(m > 155 || m < 15)
  {
    //error in meter calculations
      meterError();
  }
  else
  {
    //meters
    if(u == 1)
    {
      setRS_RW(0x0);
      setDATA(0x8A); 
      enable_ms();
      
      printM(m);
      printUnit(u);
    }
    //decimeters
    else if(u == 2)
    {
      setRS_RW(0x0);
      setDATA(0x8A); 
      enable_ms();
      
      printDm(m);
      printUnit(u);
    }
    //centimeters = also default
    else
    {
      setRS_RW(0x0);
      setDATA(0x8A); 
      enable_ms();
      
      printCm(m);
      printUnit(u);
    }
      
  }
  
}

void meterError()
{
    setRS_RW(0x0);
    setDATA(0x8A); 
    enable_ms();
    printChar(c_N);
    printChar(c_A);
    printChar(c_P);
    printChar(c_A);
    printChar(c_K);
    printChar(c_A);
}
//custom pattern writing
void createPatterns()
{
  createUP();
  Delay_ms(100);
  createDOWN();
}
void createUP()
{
  //set CGRAM address
  setRS_RW(0x0);
  setDATA(0x40); 
  enable();
  
  //write data into address
  setRS_RW(0x2);
  setDATA(0x00);
  enable();
  setDATA(0x04);
  enable();
  setDATA(0x0E);
  enable();
  setDATA(0x15);
  enable();
  setDATA(0x04);
  enable();
  setDATA(0x04);
  enable();
  setDATA(0x00);
  enable();
}
void createDOWN()
{
  //set CGRAM address
  setRS_RW(0x0);
  setDATA(0x48); 
  enable();
  
  //write data into address
  setRS_RW(0x2);
  setDATA(0x00);
  enable();
  setDATA(0x04);
  enable();
  setDATA(0x04);
  enable();
  setDATA(0x15);
  enable();
  setDATA(0x0E);
  enable();
  setDATA(0x04);
  enable();
  setDATA(0x00);
  enable();
}


/*      TIMING DELAY    */
void init_TIM4(){
  
  //clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 10; // TIM4_ARR .. 10000 = 1s
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;   //8400
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
  TIM_Cmd(TIM4,ENABLE);
}

void Delay_ms(int a)
{
  while( a > 0 )
  {
    if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update))
    {
          TIM_ClearFlag(TIM4,TIM_FLAG_Update);
          a--;
    }
  }
}

void Delay_s(int s)
{
  for(int i = 0; i < s; i++)
  {
	  Delay_ms(1000);
  }
}
//keypad
void set_GPIOB_Keypad_IN()
{
     /* GPIOB Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void set_GPIOD_Keypad_IN()
{
    /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void set_GPIO_Keypad_High()
{
  //gpiob
  GPIO_SetBits(GPIOB, GPIO_Pin_3);
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
  GPIO_SetBits(GPIOB, GPIO_Pin_7);
  
  //gpiod
  GPIO_SetBits(GPIOD, GPIO_Pin_0);
  GPIO_SetBits(GPIOD, GPIO_Pin_2);
  GPIO_SetBits(GPIOD, GPIO_Pin_3);
  GPIO_SetBits(GPIOD, GPIO_Pin_6); 
}

void set_Row_In(uint32_t row)
{
    /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = row;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void set_Row_Out(uint32_t row)
{
    /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = row;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

int buttonPressed()
{
  int keypad[4][3] = { {1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 0, 11} };
  
  //keypad init
  set_GPIOB_Keypad_IN();        //with pullup
  set_GPIOD_Keypad_IN();
  
  //set high
  set_GPIO_Keypad_High();
  uint32_t rows[] = {GPIO_Pin_0, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_6};
  uint32_t colmns[] = {GPIO_Pin_3, GPIO_Pin_5, GPIO_Pin_7};
  
  for(int r = 0; r < 4; r++)
  {
    //drive row to low
    set_Row_Out(rows[r]);
    GPIO_ResetBits(GPIOD, rows[r]);
    
    //scan columns for low
    for(int c = 0; c < 3; c++)
    {
      if( GPIO_ReadInputDataBit(GPIOB, colmns[c]) == 0  )
        return  keypad[r][c];
    }
    //this row was not active.. set back to INPUT
    set_Row_In(rows[r]);
  }
  
  return -1;
  
}



/*      END OF USER CODE        */