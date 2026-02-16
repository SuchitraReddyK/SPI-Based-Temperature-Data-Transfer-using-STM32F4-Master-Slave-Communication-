#include<stdint.h>
#include"stm32f4xx.h"
#include "SPI.h"

//Function Prototyping
void read_temp();
void GPIO_init();//Initialize GPIO pins
void ADC_init();//Initialize ADC for Temperature reading
void ADC_read();//Read the ADC(Temperature sensor value)
void SPI_send(uint8_t data);

//Global declarations
uint32_t ADC_val;
char rx[8];
char msg[8] = {0};  // Initialize the message array
float temp_data;

int main(void)
{
    uint32_t count = 0, lp;
    GPIO_init();// Initialize GPIO pins
    SPI1_master();// Initialize SPI1 as Master
    SPI3_slave();// Initialize SPI3 as Slave
    ADC_init();// Initialize ADC for temperature reading

    while (1)
    {

        ADC_read();//Read the ADC (temperature sensor value)
        read_temp();//Calculate the temperature

        //Prepare the temperature data for SPI transmission
        msg[0] = temp_data;// Store the temperature value in float
        for (count = 0; count < 8; count++)//Send the temperature data via SPI
        {
            SPI_send(msg[count]);// Send the temperature data byte by byte
            while (!(SPI1->SR & (1 << 1))) {} // Wait until TX buffer is empty (for SPI1)

            if (SPI3->SR & (1 << 0))// Check if data is received from SPI3 (Slave)
            {
                rx[count] = SPI3->DR; // Store received data from SPI3 (Slave)
            }
        }


        for (lp = 0; lp < 50000; lp++);  // Add a delay to simulate a slower sampling rate
    }
}

//GPIO initialization
void GPIO_init()
{
	    RCC->AHB1ENR |= (1<<0);//Enable clock for port A
		GPIOA->MODER &= ~((3<<10)|(3<<12)|(3<<14));//clear PA5 - PA7
		GPIOA->MODER |= ((2<<10)|(2<<12)|(2<<14));// Alternate mode for PA5 - PA7
		GPIOA->OTYPER &= ~(7<<5);//push-pull for PA5-PA7
		GPIOA->OSPEEDR &= ~((3<<10)|(3<<12)|(3<<14));//clear the speed select
		GPIOA->OSPEEDR |= ((2<<10)|(2<<12)|(2<<14));//high speed for PA5-PA7
		GPIOA->AFR[0] &= ~(0xFFF << 20);// Clear alternate function settings
		GPIOA->AFR[0] |= (0x555 << 20);//SPI alternate function  for PA5-PA7

		RCC->AHB1ENR |= (1<<2);//Enable clock for port C
		GPIOC->MODER &= ~((3<<20)|(3<<22)|(3<<24));//clear PC10 - PC12
		GPIOC->MODER |= ((2<<20)|(2<<22)|(2<<24));//Alternate mode for PC10 - PC12
		GPIOC->OTYPER &= ~(7<<10);//push-pull for PC10-PC12
		GPIOC->OSPEEDR &= ~((3<<20)|(3<<22)|(3<<24));//clear the speed select
		GPIOC->OSPEEDR |= ((2<<20)|(2<<22)|(2<<24));//high speed for PC10-PC12
		GPIOC->AFR[1] &= ~(0xFFF << 8);// Clear alternate function settings
		GPIOC->AFR[1] |= (0x666 << 8);//SPI alternate functions for PC10-PC12
}

void ADC_init()
{
	RCC->APB2ENR |= (1<<8);//Enable ADC1 clock
	ADC1->SQR3 |=(0x10<<0);//Select channel 16(Temperature Sensor) for conversion
	ADC->CCR |=(1<<23);//Enable ADC temperature sensor
	ADC1->CR1 = 0;//Clear any previous settings
	ADC1->SMPR1 |=(1<<18);//Set sampling time for channel 16 (Temperature Sensor)
	ADC1->SMPR1 |=(1<<19);
	ADC1->SMPR1 |=(1<<20);
	ADC1->CR2 |= (1<<0);//Enable ADC1
}

//
void ADC_read()
{
	ADC1->CR2 |=(1<<30);//Start conversion
	while(!(ADC1->SR & (1<<1)));//Wait for conversion complete
	ADC_val = ADC1->DR;//Read the ADC result
}
void read_temp()
{
	float Vsense, volt_25, slope;
	Vsense = ADC_val*(3.3/4095);//Convert ADC value to voltage
	volt_25 = 0.76;//Voltage at 25degree C
	slope = 0.0025;//voltage at 25degree C
	temp_data = (Vsense-volt_25)/slope;//Calculate the temperature
}

//
void SPI_send(uint8_t data)
{
	SPI1->DR = data;//Data is sending via SPI1, write data to SPI1 data register
}
