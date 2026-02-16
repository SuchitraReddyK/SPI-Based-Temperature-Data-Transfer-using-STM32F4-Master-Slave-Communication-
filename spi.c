#include "spi.h"
#include "stm32f4xx.h"

//Function Prototyping
void GPIO_Init(void);
void SPI1_Master_Init(void);
void SPI3_Slave_Init(void);
uint8_t SPI1_Transmit(uint8_t data);
uint8_t SPI3_Receive(void);

// Initialize GPIOA and GPIOC for SPI1 and SPI3
void GPIO_Init(void) {
    RCC->AHB1ENR |=(1<<0) |  (1<<2);

    // SPI1 - PA5 (SCK), PA6 (MISO), PA7 (MOSI)
    GPIOA->MODER |= (2 << 10) | (2 << 12) | (2 << 14);
    GPIOA->AFR[0] |= (5 << 20) | (5 << 24) | (5 << 28);

    // SPI3 - PC10 (SCK), PC11 (MISO), PC12 (MOSI)
    GPIOC->MODER |= (2 << 20) | (2 << 22) | (2 << 24);
    GPIOC->AFR[1] |= (6 << 8) | (6 << 12) | (6 << 16);
}

// SPI1 Master configuration
void SPI1_Master_Init(void) {
    RCC->APB2ENR |= (1<<12);
    SPI1->CR1 = 0;
    SPI1->CR1 |= (1 << 2); // Master mode
    SPI1->CR1 |= (1 << 1) | (1 << 0); // CPOL and CPHA
    SPI1->CR1 |= (3 << 3); // Baud rate fPCLK/16
    SPI1->CR1 |= (1 << 8) | (1 << 9); // SSI and SSM
    SPI1->CR1 &= ~(1 << 10); // Full duplex
    SPI1->CR1 |= (1 << 6); // Enable SPI1
}

// SPI3 Slave configuration
void SPI3_Slave_Init(void) {
    RCC->APB1ENR |= (1<<15);
    SPI3->CR1 = 0;
    SPI3->CR1 |= (1 << 1) | (1 << 0); // CPOL and CPHA
    SPI3->CR1 |= (1 << 9); // SSM
    SPI3->CR1 &= ~(1 << 2); // Slave mode
    SPI3->CR1 |= (1 << 6); // Enable SPI3
}

// Transmit 8-bit data via SPI1
uint8_t SPI1_Transmit(uint8_t data) {
    while (!(SPI1->SR & (1 << 1)));   // Wait until transmit buffer is empty
    SPI1->DR = data;                    // Send data
    while (!(SPI1->SR & (1 << 0))); // Wait until receive buffer is full
    return SPI1->DR;                    // Return received byte
}


// Receive 8-bit data via SPI3
uint8_t SPI3_Receive(void) {
    while (!(SPI3->SR & (1 << 0)));
    return SPI3->DR;
}
