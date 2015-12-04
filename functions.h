#ifndef FUNCTIONS_H_INCLUDED
#define FUNCTIONS_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "alphaNum.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//GPIO  B
#define RS      GPIO_Pin_13 // RS is named as Port 13
#define RW      GPIO_Pin_14 // RW is named as Port 14
#define EN      GPIO_Pin_15 // EN is named as Port 15
//GPIO  E
#define DB0     GPIO_Pin_5
#define DB1     GPIO_Pin_6
#define DB2     GPIO_Pin_7
#define DB3     GPIO_Pin_8
#define DB4     GPIO_Pin_9
#define DB5     GPIO_Pin_10
#define DB6     GPIO_Pin_11
#define DB7     GPIO_Pin_12
//DISTANCE SENSOR
/*	CABLE CONNECTIONS:
PURPLE-black:   GND     ground          GND
GREEN-white:    Vout    reading         PC2
BLUE-red:       Vcc     supply          5V
*/
#define NUMBER_OF_VALUES        14
#define NUMBER_OF_READINGS      20

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void init();
//lights
void init_GPIOD();
//SPI 
void SPI1_Write(uint8_t data);
uint8_t SPI1_Read();
void init_GPIOA_SPI();
void init_GPIOE_SPI();
//accelerator
void init_accel();
void start_accel();
int8_t read_accel_x();
int8_t read_accel_y();
void update_accel(uint8_t ux, uint8_t uy);
//distance sensor
void initADC(void);
int getMeasurement();
double measurementToCm(int val);
int preciseMeasurement();
//lcd init
void init_GPIOB();
void init_GPIOE_OUT();
void init_GPIOE_IN();
void init_LCD_protocol();
//lcd comms
void setRS_RW(int n);
void enable();
void enable_ms();
void setDATA(int n);
void waitBusy();
//lcd printing
void clearDisplay();
void dcb();
void cursorLeft();
void cursorRight();
void displayLeft();
void displayRight();
void lineOne();
void lineTwo();
void printChar(int num);
void printCm(int num);
void printDm(int num);
void printM(int num);
void printUnit(int u);
void printBalanced();
void printCorrectOne(int c);
void printCorrectTwo(int c1, int c2);
//prepared Printing
void startUp();
void workScreen();
void dTemplate();
void updateScreen(int u, int m);
void meterError();
//custom pattern writing
void createPatterns();
void createUP();
void createDOWN();
//timing delay
void init_TIM4();
void Delay_ms(int a);
void Delay_s(int s);
//keypad
void set_GPIOB_Keypad_IN();
void set_GPIOD_Keypad_IN();
void set_GPIO_Keypad_High();
void set_Row_Out(uint32_t row);
void set_Row_In(uint32_t row);
int buttonPressed();
#endif
