#ifndef SPI_H
#define SPI_H

#include <stdint.h>

// SPI functions
void GPIO_Init(void);
void SPI1_Master_Init(void);
void SPI3_Slave_Init(void);
uint8_t SPI1_Transmit(uint8_t data);
// void SPI1_Transmit(uint8_t data);
uint8_t SPI3_Receive(void);

#endif // SPI_H
