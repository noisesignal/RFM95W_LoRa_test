// RM0394 https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

#include "main.h"

// Default system clock frequency.
uint32_t core_clock_hz = 4000000;
// SysTick delay counter.
volatile uint32_t systick = 0;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// Delay for a specified number of milliseconds.
// TODO: Prevent rollover bug on the 'systick' value.
void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __asm__( "WFI" ); }
}

// Override the 'write' clib method to implement 'printf' over UART.
int _write( int handle, char* data, int size ) {
  int count = size;
  while( count-- ) {
    while( !( USART2->ISR & USART_ISR_TXE ) ) {};
    USART2->TDR = *data++;
  }
  return size;
}

// Send a byte of data over SPI (blocking)
void spi_w8( SPI_TypeDef *SPIx, uint8_t dat ) {
  // Wait for 'transmit buffer empty' bit to be set, then send data.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  *( uint8_t* )&( SPIx->DR ) = dat;
  // Dummy receive.
  while( !( SPIx->SR & SPI_SR_RXNE ) ) {};
  uint8_t dr = *( uint8_t* )&( SPIx->DR );
}

// Receive a byte of data over SPI (blocking)
uint8_t spi_r8( SPI_TypeDef *SPIx ) {
  // Transmit a dummy byte once the peripheral is ready.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  *( uint8_t* )&( SPIx->DR ) = 0x00;
  // Wait to receive a byte of data, then return it.
  while( !( SPIx->SR & SPI_SR_RXNE ) ) {};
  return *( uint8_t* )&( SPIx->DR );
}

// Logic to read a register from the RF-LORA-868 module.
uint8_t read_rf_reg( uint8_t addr ) {
  // Assert CS signal.
  GPIOA->ODR &= ~( 0x1 << 11 );
  // Write the address byte with the read/write bit set to 0.
  spi_w8( SPI1, addr & 0x7F );
  // Receive the current register contents from the module.
  uint8_t rx = spi_r8( SPI1 );
  // Release CS signal, then return the received value.
  GPIOA->ODR |=  ( 0x1 << 11 );
  return rx;
}

// Logic to write a register in the RF-LORA-868 module.
void write_rf_reg( uint8_t addr, uint8_t data ) {
  // Assert CS signal.
  GPIOA->ODR &= ~( 0x1 << 11 );
  // Write the address with the read/write bit set to 1.
  spi_w8( SPI1, addr | 0x80 );
  // Write the byte of data.
  spi_w8( SPI1, data );
  // Release CS signal.
  GPIOA->ODR |=  ( 0x1 << 11 );
}

/**
 * Main program.
 */
int main(void) {
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit. (Required for 'printf' formatting)
  SCB->CPACR    |=  ( 0xF << 20 );

  // Use the 16MHz HSI oscillator for the system core clock.
  RCC->CR |=  ( RCC_CR_HSION );
  while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
  RCC->CFGR &= ~( RCC_CFGR_SW );
  RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
  while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
  core_clock_hz = 16000000;

  // Setup the SysTick peripheral to generate 1ms ticks.
  SysTick_Config( core_clock_hz / 1000 );

  // Enable peripheral clocks: GPIOA, GPIOB, SPI1, USART2.
  RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
  RCC->APB2ENR  |= ( RCC_APB2ENR_SPI1EN );
  RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN |
                     RCC_AHB2ENR_GPIOBEN );
  //
  // UART TX pin setup (AF7).
  GPIOA->MODER    &= ~( 0x3 << ( 2 * 2 ) );
  GPIOA->MODER    |=  ( 0x2 << ( 2 * 2 ) );
  GPIOA->OTYPER   &= ~( 0x1 << 2 );
  GPIOA->OSPEEDR  &= ~( 0x3 << ( 2 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 0x2 << ( 2 * 2 ) );
  GPIOA->AFR[ 0 ] &= ~( 0xF << ( 2 * 4 ) );
  GPIOA->AFR[ 0 ] |=  ( 0x7 << ( 2 * 4 ) );


  //send reset signal from PB1 
  GPIOB->MODER    &= ~( 0x3 << ( 1 * 2 ) );
  GPIOB->MODER    |=  ( 0x1 << ( 1 * 2 ) );
  GPIOB->OTYPER    &= ~( 0x1 <<  1 );
//  GPIOB->OTYPER   |=  ( 0x1 << 1 );
  GPIOB->PUPDR    &= ~( 0x3 << ( 1 * 2 ) );
  GPIOB->PUPDR   |=  ( 0x1 << ( 1 * 2 ) );
  GPIOB->BSRR    |= ( 0x1 <<  17 );


  // SPI pins setup.
  // PA11: software-controlled CS pin.
  GPIOA->MODER    &= ~( 0x3 << ( 11 * 2 ) );
  GPIOA->MODER    |=  ( 0x1 << ( 11 * 2 ) );
  GPIOA->OTYPER   &= ~( 0x1 << 11 );
  GPIOA->OSPEEDR  &= ~( 0x3 << ( 11 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 0x1 << ( 11 * 2 ) );
  GPIOA->ODR      |=  ( 0x1 << 11 );

  // PA5, PA6, PA7: hardware-controlled SPI SCK/MISO/MOSI pins (AF5) respectively.
  // p55 https://www.st.com/resource/en/datasheet/stm32l432kc.pdf
  GPIOA->MODER    &= ~( ( 0x3 << ( 5 * 2 ) ) |
                        ( 0x3 << ( 6 * 2 ) ) |
                        ( 0x3 << ( 7 * 2 ) ) );
  GPIOA->MODER    |=  ( ( 0x2 << ( 5 * 2 ) ) |
                        ( 0x2 << ( 6 * 2 ) ) |
                        ( 0x2 << ( 7 * 2 ) ) );
  GPIOA->OSPEEDR  &= ~( ( 0x3 << ( 5 * 2 ) ) |
                        ( 0x3 << ( 6 * 2 ) ) |
                        ( 0x3 << ( 7 * 2 ) ) );
  GPIOA->OSPEEDR  |=  ( ( 0x2 << ( 5 * 2 ) ) |
                        ( 0x2 << ( 6 * 2 ) ) |
                        ( 0x2 << ( 7 * 2 ) ) );
 GPIOA->PUPDR  &= ~( ( 0x3 << ( 5 * 2 ) ) |
                        ( 0x3 << ( 6 * 2 ) ) |
                        ( 0x3 << ( 7 * 2 ) ) );
  GPIOA->PUPDR  |=  ( ( 0x2 << ( 5 * 2 ) ) |
                        ( 0x2 << ( 6 * 2 ) ) |
                        ( 0x2 << ( 7 * 2 ) ) );
  GPIOA->AFR[ 0 ] &= ~( ( 0xF << ( 5 * 4 ) ) |
                        ( 0xF << ( 6 * 4 ) ) |
                        ( 0xF << ( 7 * 4 ) ) );
  GPIOA->AFR[ 0 ] |=  ( ( 0x5 << ( 5 * 4 ) ) |
                        ( 0x5 << ( 6 * 4 ) ) |
                        ( 0x5 << ( 7 * 4 ) ) );

  // UART setup: 115200 baud, transmit only.
  USART2->BRR  = ( core_clock_hz / 115200 );
  USART2->CR1 |= ( USART_CR1_TE | USART_CR1_UE );

  // SPI setup: standard host mode.
  SPI1->CR1 |= ( SPI_CR1_SSM |
                 SPI_CR1_SSI |
                 SPI_CR1_MSTR |
                 SPI_CR1_CPOL |
                 SPI_CR1_CPHA );
  // Set RX FIFO threshold to one byte instead of two.
  // Without this, the RXNE flag will not get set for single bytes.
  SPI1->CR2 |= ( SPI_CR2_FRXTH );
  SPI1->CR1 |= ( SPI_CR1_SPE );
 

//  https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/440000001NCE/v_VBhk1IolDgxwwnOpcS_vTFxPfSEPQbuneK3mWsXlU -> p 115
//  Send reset signal from PB1

  GPIOB->BSRR    |= ( 0x1 <<  17 );
  delay_ms( 200 );
  GPIOB->BSRR    |=  ( 0x1 <<  1 );
  delay_ms( 200 );
  GPIOB->BSRR    |=  ( 0x1 <<  17 );
   // GPIOB->OTYPER    |= ( 0x1 <<  1 );
  delay_ms( 100 );

  uint8_t reg;
  printf( "Configure RF-LORA-868 for LoRa communication...\r\n" );
  // Set 'RegOpMode' to sleep mode, and clear all other flags.

  write_rf_reg( RF_OPMODE, RF_OP_MD_SLEEP );
  // Wait for the module to go to sleep.
  reg = read_rf_reg( RF_OPMODE );
  while ( ( reg & RF_OP_MD ) != RF_OP_MD_SLEEP ) {
    reg = read_rf_reg( RF_OPMODE );
  }

  reg = read_rf_reg( RF_ID );
  printf( "Revision Id: 0x%02X\r\n", reg );

  // These end blocks should never be reached.
  while ( 1 ) {};
  return 0;
}

// SysTick interrupt handler: increment the global 'systick' value.
void SysTick_IRQn_handler( void ) {
  ++systick;
}



