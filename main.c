
#include "spi.h"
#include "stm32f4xx.h"

uint32_t adc_val = 0;
float temperature_celsius = 0.0f;
uint8_t spi_data = 0, rx_data = 0;
uint8_t loopcount=0;

//Function Prototyping
void ADC_Init(void);
void ADC_Read(void);
void Calculate_Temperature(void);

K
int main(void) {
    GPIO_Init();   // Step 1: Initialize SPI pins
    SPI1_Master_Init();  // Step 2: Configure SPI1 as Master
    SPI3_Slave_Init();   // Step 3: Configure SPI3 as Slave
    ADC_Init();          // Step 4: Configure ADC

    while (1) {
        ADC_Read();              // Step 5: Read ADC value
        Calculate_Temperature(); // Step 6: Convert to temperature
        spi_data = (uint8_t)temperature_celsius; // Convert float to 8-bit

        SPI1_Transmit(spi_data);        // Step 7: Transmit via SPI1
       // rx_data = SPI3_Receive();       // Step 8: Receive via SPI3
        rx_data = SPI1_Transmit(spi_data);
        // Debug: watch adc_val, temperature_celsius, spi_data, rx_data
        for ( loopcount = 0; loopcount < 100000; loopcount++);
    }
}

// ADC1 configuration for temperature sensor (channel 16)
void ADC_Init(void) {
    RCC->APB2ENR |=(1<<8);
    GPIOA->MODER |= (3 << 0); // PA0 as analog

    ADC->CCR |= (1 << 23); // Enable temp sensor
    ADC1->SQR3 = 16; // Channel 16
    ADC1->SMPR1 |= (7 << 18); // Sampling time
    ADC1->CR2 |= (1 << 0); // Enable ADC
}

// Read ADC1 data
void ADC_Read(void) {
    ADC1->CR2 |= (1 << 30); // Start conversion
    while (!(ADC1->SR & (1 << 1))); // Wait for EOC
    adc_val = ADC1->DR;
}

// Convert ADC value to temperature in Celsius
void Calculate_Temperature(void) {
    float Vsense = (adc_val * 3.3f) / 4095.0f;
    temperature_celsius = (Vsense - 0.76f) / 0.0025f;
}
