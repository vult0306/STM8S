/* Simple "Hello World" UART output  */
/*
 * Analog input to port D4
 * ds18b20 input to port A3
 */
#include <string.h>
#include <stdint.h>
#include <float.h>
#include "stm8.h"

/* 1-Wire (DS18B20 data) pin */

#define OW_INPUT_MODE()     PORT(OW_PORT,DDR) &= ~OW_PIN
#define OW_OUTPUT_MODE()    PORT(OW_PORT,DDR) |= OW_PIN
#define OW_LOW()            PORT(OW_PORT,ODR) &= ~OW_PIN
#define OW_HIGH()           PORT(OW_PORT,ODR) |= OW_PIN
#define OW_READ()           (PORT(OW_PORT,IDR) & OW_PIN)

#define OW_PORT PA
#define OW_PIN  PIN3
#define DEBUG
#define VREF 3.3
#define SCOUNT 200
/* Simple busy loop delay */

#define F_CPU 16000000

static inline void _delay_cycl( unsigned short __ticks )
{
#define T_COUNT(x) (( F_CPU * x / 1000000UL )-5)/5)
__asm__("nop\n nop\n"); 
do { 		// ASM: ldw X, #tick; lab$: decw X; tnzw X; jrne lab$
            __ticks--;//      2c;                 1c;     2c    ; 1/2c   
    } while ( __ticks );
__asm__("nop\n");
}

static inline void _delay_us( const unsigned short __us )
{
	_delay_cycl( (unsigned short)( T_COUNT(__us) );
}

void _delay_ms( unsigned short __ms )
{
	while ( __ms-- )
	{
		_delay_us( 1000 );
	}
}

/********************** uart routines ***************************/
#if defined DEBUG
int uart_write(const char *str, char len) {
    char i;
    for(i = 0; i < len; i++) {
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
#endif
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

/*
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
*/
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
        // display_ds_temperature(scratchpad[1], scratchpad[0]);
        //vle send to uart

    } else {
        /* DS18B20 was not detected */
        // vle send to uart
#if defined DEBUG
        uart_write("can not read temperature",strlen("can not read temperature"));
#endif
        return 0;
        // output_max(0x8, 0xa);
    }
}

/***************************************************************************/

void init_adc() {
     PC_DDR &= ~(0x1 << 4);
     PC_CR1 &= !(0x1 << 4);
     ADC_CSR |= ((0x0F)&2); // select channel
     ADC_CR2 |= (1<<3); // Right Aligned Data
     ADC_CR1 |= (1<<0); // ADC ON 

    _delay_ms(1000); // Give little time to be ready for first conversion
}

unsigned int analog_read(){
     unsigned int val=0;
     ADC_CR1 |= (1<<0); // ADC Start Conversion
     while(((ADC_CSR)&(1<<7))== 0); // Wait till EOC
     val |= (unsigned int)ADC_DRL;
     val |= (unsigned int)ADC_DRH<<8;
     val &= 0x03ff;
     ADC_CSR &= ~ADC_CSR_EOC; //clear EOC
     return (val);
}

void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637ReadResult(void);
void _tm1637WriteByte(unsigned char b);

void _tm1637ClkHigh(void);
void _tm1637ClkLow(void);
void _tm1637DioHigh(void);
void _tm1637DioLow(void);
void tm1637SetBrightness(char brightness);

const char segmentMap[] = {
	0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
	0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
	0x00
};


void tm1637Init(void)
{
	tm1637SetBrightness(8);
}



void tm1637DisplayDecimal(long TT,unsigned int displaySeparator)
{ unsigned int ii;
	unsigned int v = TT & 0x0000FFFF;
	unsigned char digitArr[4];



	//  unsigned char digitArr[4];
	for (ii = 0; ii < 4; ++ii) {
		digitArr[ii] = segmentMap[v % 10];
		if (ii == 2 && displaySeparator) {
			digitArr[ii] |= 1 << 7;
		}
		v /= 10;
	}

	_tm1637Start();
	_tm1637WriteByte(0x40);
	_tm1637ReadResult();
	_tm1637Stop();

	_tm1637Start();
	_tm1637WriteByte(0xc0);
	_tm1637ReadResult();

	for (ii = 0; ii < 4; ++ii) {
		_tm1637WriteByte(digitArr[3 - ii]);
		_tm1637ReadResult();
	}

	_tm1637Stop();
}

// Valid brightness values: 0 - 8.
// 0 = display off.
void tm1637SetBrightness(char brightness)
{
	// Brightness command:
	// 1000 0XXX = display off
	// 1000 1BBB = display on, brightness 0-7
	// X = don't care
	// B = brightness
	_tm1637Start();
	_tm1637WriteByte(0x87 + brightness);
	_tm1637ReadResult();
	_tm1637Stop();
}

void _tm1637Start(void)
{
	_tm1637ClkHigh();
	_tm1637DioHigh();
	_delay_ms(5);
	_tm1637DioLow();
}

void _tm1637Stop(void)
{
	_tm1637ClkLow();
	_delay_ms(5);
	_tm1637DioLow();
	_delay_ms(5);
	_tm1637ClkHigh();
	_delay_ms(5);
	_tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
	_tm1637ClkLow();
	_delay_ms(5);
	// while (dio); // We're cheating here and not actually reading back the response.
	_tm1637ClkHigh();
	_delay_ms(5);
	_tm1637ClkLow();
}

void _tm1637WriteByte(unsigned char b)
{int ii;
	for (ii = 0; ii < 8; ++ii) {
		_tm1637ClkLow();
		if (b & 0x01) {
			_tm1637DioHigh();
		}
		else {
			_tm1637DioLow();
		}
		_delay_ms(15);
		b >>= 1;
		_tm1637ClkHigh();
		_delay_ms(15);
	}
}



void _tm1637ClkHigh(void)
{ 
	//PB_ODR_bit.ODR5 = 1; //      _tm1637ClkHigh(); 

	//  GPIO_WriteHigh(GPIOD,GPIO_PIN_2);
	PD_ODR |= 1 << 2;
}

void _tm1637ClkLow(void)
{ 
	// GPIO_WriteLow(GPIOD,GPIO_PIN_2);

	PD_ODR &= ~(1 << 2);

	//    PB_ODR_bit.ODR5 = 0; //      _tm1637ClkHigh(); 

}

void _tm1637DioHigh(void)
{
	//PB_ODR_bit.ODR4 = 1; //  _tm1637DioHigh(); 
	// GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
	PD_ODR |= 1 << 3;

}

void _tm1637DioLow(void)
{
	PD_ODR &= ~(1 << 3);

	//GPIO_WriteLow(GPIOD,GPIO_PIN_3);
	//PB_ODR_bit.ODR4 = 0; //  _tm1637DioHigh(); 

}

int getMedianNum(int* bArray, uint8_t count) 
{
    int i, j, bTemp;
    int bTab[SCOUNT];

      for (i = 0; i<SCOUNT; i++)
	  bTab[i] = bArray[i];
      for (j = 0; j < SCOUNT - 1; j++) 
      {
	  for (i = 0; i < SCOUNT - j - 1; i++) 
          {
	    if (bTab[i] > bTab[i + 1]) 
            {
		bTemp = bTab[i];
	        bTab[i] = bTab[i + 1];
		bTab[i + 1] = bTemp;
	     }
	  }
      }
      if ((count & 1) > 0)
	bTemp = bTab[(SCOUNT - 1) / 2];
      else
	bTemp = (bTab[SCOUNT / 2] + bTab[SCOUNT / 2 - 1]) / 2;
      return bTemp;
}

int main(void)
{

    float temp;
    uint16_t ppm_value;
    uint8_t i;
#if defined DEBUG
    // char text[6];
#endif
    float Voltage;
#if defined DEBUG
    init_uart();
#endif
    init_adc();

	//display on PD2/PD3-CLK/DIO
	PD_DDR = (1 << 3) | (1 << 2); // output mode
	PD_CR1 = (1 << 3) | (1 << 2); // push-pull
	PD_CR2 = (1 << 3) | (1 << 2); // up to 10MHz speed
	tm1637Init();

    // Timer setup (for delay_us)
    TIM2_PSCR = 0x4; // Prescaler: to 1MHz
    TIM2_CR1 |= TIM_CR1_CEN; // Start timer

    while(1) {
        // read temperature value
        temp = read_ds18b20(); 
        // tm1637DisplayDecimal(temp, 0); // eg 37:12
        Voltage = 0;
        for(i=0;i<SCOUNT;i++)
        {
            Voltage += analog_read();
            _delay_ms(5);
        }
        Voltage = (float)(Voltage/SCOUNT) * (float)VREF / 1024.0;
        Voltage = (float)(Voltage/(1.0+0.02*(temp-25.0)));
        ppm_value = (uint16_t)(143.82*Voltage*Voltage*Voltage*Voltage - 418.29*Voltage*Voltage*Voltage + 458.42*Voltage*Voltage + 183.44*Voltage - 15.603);
#if defined DEBUG
        // uart_write(text,sizeof(text));
#endif
        // _delay_ms(350);
        tm1637DisplayDecimal(ppm_value, 0); // eg 37:12
    }
}