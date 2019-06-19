/* Simple "Hello World" UART output  */
#include <string.h>
#include <stdint.h>
#include "stm8.h"

/* Simple busy loop delay */
void delay(unsigned long count) {
    while (count--)
        nop();
}

void init_adc() {
    ADC_CSR = 0;
    ADC_CR1 = 0;
    ADC_CR2 = 0;
    ADC_CR3 = 0;

/*
    ADC_DRH = 0;
    ADC_DRL = 0;
    ADC_TDRH = 0;
    ADC_TDRL = 0;
    ADC_HTRH = 0;
    ADC_HTRL = 0;
    ADC_LTRH = 0;
    ADC_LTRL = 0;
    ADC_AWSRH = 0;
    ADC_AWSRL = 0;
    ADC_AWCRH = 0;
    ADC_AWCRL = 0;
*/
    ADC_CSR = 2; // Select channel 2 (AIN2=PC4)

    ADC_CR1 |= ADC_CR1_ADON; // ADON
    ADC_CR2 &= ~ADC_CR2_ALIGN; // Align left

    delay(1000); // Give little time to be ready for first conversion
}

uint16_t analog_read() {
    ADC_CR1 &= ~ADC_CR1_CONT; // Single conversion mode
    ADC_CR1 |= ADC_CR1_ADON; // Start conversion
    do { nop(); } while ((ADC_CSR >> 7) == 0);
    ADC_CSR &= ~ADC_CSR_EOC; // Clear "End of conversion"-flag
    return (ADC_DRH << 2) | (ADC_DRL >> 6);  // Left aligned
}

int uart_write(const char *str) {
    char i;
    for(i = 0; i < strlen(str); i++) {
        while(!(UART1_SR & UART_SR_TXE)); // !Transmit data register empty
        UART1_DR = str[i];
    }
    return(i); // Bytes sent
}

void init_uart(void)
{
    /* Set clock to full speed (16 Mhz) */
    CLK_CKDIVR = 0;

    // Setup UART1 (TX=D5)
    UART1_CR2 |= UART_CR2_TEN; // Transmitter enable
    // UART1_CR2 |= UART_CR2_REN; // Receiver enable
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    // 9600 baud: UART_DIV = 16000000/9600 ~ 1667 = 0x0683
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 0x0683 coded funky way (see ref manual)
}

int main(void)
{
    init_uart();
    init_adc();
    while(1) {
        uart_write("Hello World uart adc \r\n");
        delay(400000L);
    }
}