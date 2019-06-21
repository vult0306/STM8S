/* Simple "Hello World" UART output  */
/*
 * Analog input to port D4
 * ds18b20 input to port A3
 */
#include <string.h>
#include <stdint.h>
#include "stm8.h"

/* 1-Wire (DS18B20 data) pin */
#define OW_PORT PA
#define OW_PIN  PIN3

/* Simple busy loop delay */
void delay(unsigned long count) {
    while (count--)
        nop();
}

int uart_write(const char *str, char len) {
    char i;
    for(i = 0; i < len; i++) {
        while(!(UART1_SR & UART_SR_TXE)); // !Transmit data register empty
        UART1_DR = str[i];
    }
    return(i); // Bytes sent
}

/********************** OneWire/DS18B20 routines ***************************/
void delay_us(uint16_t i) {
    if (i < 9) { // FIXME: Really ugly
        nop();
        return;
    }
    TIM2_CNTRH = 0;
    TIM2_CNTRL = 0;
    TIM2_EGR = 0x01; // Update Generation
    while(1) {
        volatile uint16_t counter = (((TIM2_CNTRH) << 8) | TIM2_CNTRL);
        if (i-6 < counter)
            return;
    }
}

#define OW_INPUT_MODE()     PORT(OW_PORT,DDR) &= ~OW_PIN
#define OW_OUTPUT_MODE()    PORT(OW_PORT,DDR) |= OW_PIN
#define OW_LOW()            PORT(OW_PORT,ODR) &= ~OW_PIN
#define OW_HIGH()           PORT(OW_PORT,ODR) |= OW_PIN
#define OW_READ()           (PORT(OW_PORT,IDR) & OW_PIN)

void ow_pull_low(unsigned int us) {
    OW_OUTPUT_MODE();
    OW_LOW();
    delay_us(us);
    OW_INPUT_MODE();
}

void ow_write_byte(uint8_t out) {
    uint8_t i;
    for (i=0; i < 8; i++) {
        if ( out & ((uint8_t)1<<i) ) {
            // write 1
            ow_pull_low(1);
            delay_us(60);
        } else {
            // write 0
            ow_pull_low(60);
            delay_us(1);
        }
    }
}

uint8_t ow_read_byte() {
    uint8_t val = 0;
    uint8_t i;
    for (i=0; i < 8; i++) {
        ow_pull_low(1);
        delay_us(5);
        if (OW_READ()) {
            val |= ((uint8_t)1<<i);
        }
        delay_us(55);
    }
    return val;
}

unsigned int ow_init() {

    uint8_t input;

    ow_pull_low(480);
    delay_us(60);

    input = !OW_READ();
    delay_us(420);

    return input;
}

unsigned int ow_convert_temperature() {
    int cycles = 1; // For debugging purposes

    ow_write_byte(0x44); // Convert Temperature

    while (1) {
        ow_pull_low(1);
        delay_us(5);
        if (OW_READ()) {
            return cycles;
        }
        delay_us(55);
        cycles++;
    }
}

void display_ds_temperature(uint8_t high, uint8_t low) {
    uint8_t is_negative = 0;
    uint16_t decimals = 0; // 4 decimals (e.g. decimals 625 means 0.0625)
    uint16_t i;

    uint16_t temp = ((int16_t)high << 8) | low;
    if (temp & 0x8000) {
        is_negative = 1;
        temp = (~temp) + 1;
    }
    low = temp & 0x0f;
    temp = temp >> 4;

    // low[3:0] mean values 0.5,0.25,0.125 and 0.0625
    for (i=625; i <= 5000; i=i*2) {
        if (low & 0x01) {
            decimals += i;
        }
        low = low >> 1;
    }

    // Display temperature rounded to one decimal
    // vle send to uart
    // display_number_dot((temp*1000 + ((decimals+5)/10) + 50)/100, 2, is_negative);
}

float read_ds18b20() {
    // uint8_t i;
    // uint8_t scratchpad[9];
    unsigned char temp=0;
    float t=0;
    if (ow_init()) {
        ow_write_byte(0xcc); // Skip ROM
        ow_convert_temperature();

        ow_init();
        ow_write_byte(0xcc); // Skip ROM
        ow_write_byte(0xbe); // Read Scratchpad
        // for (i=0; i<9; i++) {
            // scratchpad[i] = ow_read_byte();
        // }
        temp = ow_read_byte();
        t = (((temp & 0xf0) >> 4) + (temp & 0x07) * 0.125); 
        temp = ow_read_byte();
        t += ((temp & 0x0f) << 4);
        return t;
        //display_ds_temperature(scratchpad[1], scratchpad[0]);
        //vle send to uart

    } else {
        /* DS18B20 was not detected */
        // vle send to uart
        uart_write("can not read temperature",strlen("can not read temperature"));
        return 0;
        // output_max(0x8, 0xa);
    }
}

/***************************************************************************/

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
    uint16_t temp;
    char temperature[6];
    init_uart();
    init_adc();
    // Timer setup (for delay_us)
    TIM2_PSCR = 0x4; // Prescaler: to 1MHz
    TIM2_CR1 |= TIM_CR1_CEN; // Start timer
    uart_write("before read ds18b20 \r\n",22);
    temperature[4]='\r';
    temperature[5]='\n';
    while(1) {
        temp = (uint16_t)(100*read_ds18b20());
        temperature[0]=(temp/1000) +0x30;
        temperature[1]=(temp/100)%10 +0x30;
        temperature[2]=(temp/10)%10 +0x30;
        temperature[3]=(temp%10) +0x30;
        uart_write(temperature,sizeof(temperature));
        delay(400000L);
    }
}