/**
 * @file sun.c
 * @brief Read the ADC values for the Sun sensors
 */

#include "common.h"
#include "sun.h"
#include "spi.h"
#include "peripherals.h"

void configure_SS(void)
{
  ///Choose the ADC IC on the SPI bus
  init_SPI_trans(ADC_S);
  
  ///Set PD and range pins as input
  cbi(DDRB, PB6);
  cbi(DDRB, PB7);
  
  ///Set PD high: power up the ADC IC
  sbi(PORTB, PB6);
  
  ///Configure GPIO pins - GPIO3 as PD and GPIO2 as range input
  SPI_transfer(0b01000001);
  
  SPI_transfer(0b10000000);
  
  ///Configure Auto-2 Mode
  
  ///Program the Program register of Auto-2 Mode:
  
  SPI_transfer(0b10010001);
  
  SPI_transfer(0b01000000);
  
  ///Set Mode to Auto-2
  
  SPI_transfer(0b00111100);
  
  SPI_transfer(0b00000000);
}

void poll_SS(void)
{
  uint8_t channel = 0;
  //uint8_t c= 100;
  ///Loop for reading the 6 sun sensor values
  //transmit_UART0(c);
 
  while(channel <= 5)
  {
    ///* Put the ADC reading in the appropriate variable
    Current_state.ss.reading[channel] = (uint16_t)receive_UART0();
    Current_state.ss.reading[channel] = Current_state.ss.reading[channel] << 8;
    Current_state.ss.reading[channel] &= 0xFF00;
    Current_state.ss.reading[channel] |= (uint16_t)receive_UART0();
    channel++;
	
  }
  for (int i=0;i<6;i=i+1)
  {
	  Current_state.ss.read[i] = (float)(Current_state.ss.reading[i]*(3.3/1024));
  }
  /*uint8_t sen;
  for(int i=0;i<6;i=i+1)
  {
	  sen = (Current_state.ss.read[i]*255/3.3);
	  transmit_UART0(sen);
  }*/
  ///Power Down PD low
  //cbi(PORTB, PB6);
return;  
}
