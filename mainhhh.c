/*
 * nrf1.c
 *
 * Created: 6.03.2024 15:15:21
 * Author : Merhaba
 */ 
#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<stdio.h>
#include<stdlib.h>
 
#define HIGH 1
#define LOW 0
 
#define SS 4
#define MOSI 5
#define SCK 7
#define MISO 6
#define SPIF 7

#define 	CMD_R_REGISTER                         	     0x00        /**< read command */
#define 	CMD_W_REGISTER                	             0x20        /**< write command */
#define 	CMD_R_RX_PAYLOAD                            0x61        /**< read payload command */
#define 	CMD_W_TX_PAYLOAD                          0xA0        /**< write payload command */
#define 	CMD_FLUSH_TX                                    0xE1        /**< flush tx command */
#define 	CMD_FLUSH_RX                                    0xE2        /**< flush rx command */
#define 	CMD_REUSE_TX_PL                              0xE3        /**< reuse tx payload command */
#define   CMD_R_RX_PL_WID                             0x60        /**< read rx payload width for the top command */
#define 	CMD_W_ACK_PAYLOAD                        0xA8        /**< write ack payload command */
#define   CMD_W_TX_PAYLOAD_NO_ACK         0xB0        /**< write ack payload with ack command */
#define   CMD_NOP                                               0xFF        /**< nop command */


#define  REG_CONFIG                   0x00        /**< config register */
#define  REG_EN_AA                     0x01        /**< enable auto acknowledgment register */
#define  REG_EN_RXADDR            0x02        /**< enabled rx addresses register */
#define  REG_SETUP_AW              0x03        /**< setup of address widths register */
#define  REG_SETUP_RETR           0x04        /**< setup of automatic retransmission register */
#define  REG_RF_CH                      0x05        /**< rf channel register */
#define  REG_RF_SETUP               0x06        /**< rf setup register register */
#define  REG_STATUS                  0x07        /**< status register */
#define  REG_OBSERVE_TX         0x08        /**< transmit observe register */
#define  REG_RPD                          0x09        /**< received power detector register */
#define  REG_RX_ADDR_P0          0x0A        /**< receive address data pipe 0 register */
#define  REG_RX_ADDR_P1           0x0B        /**< receive address data pipe 1 register */
#define  REG_RX_ADDR_P2          0x0C        /**< receive address data pipe 2 register */
#define  REG_RX_ADDR_P3          0x0D        /**< receive address data pipe 3 register */
#define  REG_RX_ADDR_P4          0x0E        /**< receive address data pipe 4 register */
#define  REG_RX_ADDR_P5          0x0F        /**< receive address data pipe 5 register */
#define  REG_TX_ADDR                0x10        /**< transmit address register */
#define  REG_RX_PW_P0              0x11        /**< number of bytes in rx payload in data pipe 0 register */
#define  REG_RX_PW_P1              0x12        /**< number of bytes in rx payload in data pipe 1 register */
#define  REG_RX_PW_P2              0x13        /**< number of bytes in rx payload in data pipe 2 register */
#define  REG_RX_PW_P3              0x14        /**< number of bytes in rx payload in data pipe 3 register */
#define  REG_RX_PW_P4              0x15        /**< number of bytes in rx payload in data pipe 4 register */
#define  REG_RX_PW_P5              0x16        /**< number of bytes in rx payload in data pipe 5 register */
#define  REG_FIFO_STATUS       0x17        /**< fifo status register */
#define  REG_DYNPD                    0x1C        /**< enable dynamic payload length register */
#define  REG_FEATURE                0x1D        /**< feature register */

#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define CONT_WAVE   7
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0


#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

typedef enum
{
	yes=1,
	no=0
}is_available;

uint8_t tx_adress[5] = { 0x78, 0x78, 0x78, 0x78, 0x78 };
uint8_t rx_adress_p0[5] = { 0x78, 0x78, 0x78, 0x78, 0x78 };

/*
enum config
{
	MASK_RX_DR = 6,
	MASK_TX_DX = 5,
	MASK_MAX_RT = 4,
	EN_CRC = 3,
	CRCO = 2,
	PWR_UP = 1,
	PRIM_RX = 0
};

enum status 
{
	RX_DR = 6,
	TX_DS = 5,
	MAX_RT = 4,
	TX_FULL = 0
};

enum fifo_status
{
	TX_REUSE = 6,
	TX0_FULL = 5,
	TX_EMPTY = 4,
	RX_FULL = 1,
	RX_EMPTY = 0
};
*/
void USART_init(void){
	UBRRH = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRRL = (uint8_t)(BAUD_PRESCALLER);
	UCSRB = (1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<UCSZ0)|(1<<UCSZ1)|(1<<URSEL);
}
/* Function to receive byte/char */
unsigned char USART_receive(void){
	while(!(UCSRA & (1<<RXC)));
	return UDR;
}
/* Function to send byte/char */
void USART_ptr_send(char* data){
	while(!(UCSRA & (1<<UDRE)));
	UDR = *data;
}

void USART_send(char data){
	while(!(UCSRA & (1<<UDRE)));
	UDR = data;
}

/* Send string */
void USART_putstring(char* StringPtr){
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	    StringPtr++;}
}

void USART_print_bcd(uint8_t k)
{
	for(int i=7; i>=0; i--)
	{
		if(((k>>i) & 0x01) == 0){ USART_send(0x30); }
		else{ USART_send(0x31); }
	}
}



char bir []= "rx fifodan data alýndý";
char iki []= "gelen data uyuþtu \n\r\n ";
char uc []= "uygulama baþladý \n\r\n";
char dort []= "servo -90 da \n\r\n";
char bes []= "servo +90 da \n\r\n";
char alti []= "servo 0 da \n\r\n";
char yedi []= "display mode ayarlandi \n\r\n";
char sekiz []= "rectangle cizildi \n\r\n";
char dokuz []= "display mode ayarlandi \n\r\n";
char yeni_satir []= "\n\r\n";

/*
NRFÝ TX ETTÝRMEK ÝÇÝN GALÝBA MUTLAKA ENHANCED SCHOCK BURST KULLANMAK ZORUNDAYIZ GALÝBA


 spi operations 
 registers
 SPDR
 SPSR
 SPCR = SPIE, SPE, DORD, MSTR, CPOL , CPHA, SPR1, SPR0
 SPSR = SPIF, WCOL, , , , , , SPI2X
*/
void pin_init()
{
	DDRA |= (1<<0); // PORTA PA0 CE CHÝP EBABLE PORT OLARAK KULLANICAM7
	DDRA |= (1<<2);
	DDRA &= ~(1<<1); // nrf irq pin olarak kullanýlacak
	PORTA &= ~(1<<2);
}

void spi_init()
{
	DDRB |= (1<<MOSI) | (1<<SCK) | (1<<SS) ;
	DDRB &= ~(1<<MISO);
	PORTB |= (1<<SS) ;
	
	SPCR = (1<<SPE) | (1<<MSTR) ;	
}

void spi_write (uint8_t data)
{
    uint8_t flush_buf;
    SPDR =  data;
    while( ! ( SPSR & (1<<SPIF)));
    flush_buf = SPDR;
}

uint8_t spi_write_with_read_status(uint8_t data)
{
	uint8_t flush_buf;
	SPDR =  data;
	while( ! ( SPSR & (1<<SPIF)));
	flush_buf = SPDR;
	return flush_buf;
}

void spi_ptr_write (uint8_t* data)
{
	uint8_t flush_buf;
	SPDR =  *data;
	while( ! ( SPSR & (1<<SPIF)));
	flush_buf = SPDR;
}

uint8_t spi_read()
{
	SPDR = 0xFF;
	while( ! ( SPSR & (1<<SPIF)));
	return (SPDR);
}

uint8_t spi_direct_read()
{
	//SPDR = 0xFF;
	//while( ! ( SPSR & (1<<SPIF)));
	return (SPDR);
}

void spi_alotof_write ( uint8_t *data, size_t length)
{
	for( int i=0; i<length; i++)
	{
		spi_write( *(data+i));
	}
}	

uint8_t nrf_register_read ( uint8_t reg_name )
{
	
	spi_write ( CMD_R_REGISTER | reg_name );
	while( ! ( SPSR & (1<<SPIF)));
    return (SPDR);
}

void nrf_chip_enable()  { PORTA |= (1<<0); }

void nrf_chip_not_enable()  { PORTA &= ~(1<<0); }

void nrf_chip_select()  { PORTB &= ~(1<<SS); }

void nrf_chip_not_select()  { PORTB |= (1<<SS); }

void nrf_pwr_up()
{
	nrf_chip_not_enable();
	spi_write( CMD_W_REGISTER | REG_CONFIG );
	//config register mask_rx_dr, mask_tx_ds, mask_max_rt, en_crc, crco, pwr_up, prim_rx
	spi_write(0b00001110);
}

void nrf_enter_tx_mode()
{
	spi_write( CMD_W_REGISTER | REG_CONFIG );
	//config register mask_rx_dr, mask_tx_ds, mask_max_rt, en_crc, crco, pwr_up, prim_rx
	spi_write(0b00000010);
	nrf_chip_enable();
}

void nrf_enter_rx_mode ()
{
	//bu command ile birþey yazabilmek için power down veya standby modlarýnda olmalý
	spi_write( CMD_W_REGISTER | REG_CONFIG );
	//config register mask_rx_dr, mask_tx_ds, mask_max_rt, en_crc, crco, pwr_up, prim_rx
	spi_write(0b00001111);
	nrf_chip_enable();
}

void nrf_write_tx_fifo (uint8_t *data, size_t length)
{
	spi_write( CMD_W_TX_PAYLOAD);
	
	for( int i=0; i<<length; i++)
	{
		spi_write( *(data+i));
	}
}

uint8_t nrf_read_rx_fifo () 
{
	spi_write ( CMD_R_RX_PAYLOAD );
	spi_write(0xFF);
	while ( ! ( SPSR & (1<<SPIF)));
	return ( SPDR );
}

void nrf_listen_irq ()
{
	while ( PINA & (1<<1) ) ;
}


void nrf_enabling_or_disabling_auto_acknowledgement( uint8_t data_pipe_5, uint8_t data_pipe_4, uint8_t data_pipe_3,
																 uint8_t data_pipe_2, uint8_t data_pipe_1,
																 uint8_t data_pipe_0)
{
	uint8_t a= 0b00000000;
	a= (data_pipe_5<<5) | (data_pipe_4<<4) | (data_pipe_3<<3) | (data_pipe_2<<2) | (data_pipe_1<<1) | (data_pipe_0<<0);
	spi_write( CMD_W_REGISTER | REG_EN_AA );
	spi_write( a );
}

void nrf_setting_rx_adress ( uint8_t which_data_pipe, uint8_t * adress )
{
	uint8_t a = 0;
	switch (which_data_pipe)
	{
		
		case 0 : a= REG_RX_ADDR_P0; break;
		case 1 : a= REG_RX_ADDR_P1; break;
		case 2 : a= REG_RX_ADDR_P2; break;
		case 3 : a= REG_RX_ADDR_P3; break;
		case 4 : a= REG_RX_ADDR_P4; break;
		case 5 : a= REG_RX_ADDR_P5; break;
		
	}
	spi_write( CMD_W_REGISTER | a );
	spi_alotof_write ( adress, 5);
}

void nrf_setting_tx_adress (  uint8_t * adress )
{
	spi_write( CMD_W_REGISTER | REG_TX_ADDR );
	spi_alotof_write ( adress, 5);
}

void flush_tx_fifo() { spi_write( CMD_FLUSH_TX ); }
void flush_rx_fifo() { spi_write( CMD_FLUSH_RX ); }

void nrf_enabling_or_disabling_rx_adress ( uint8_t data_pipe_5, uint8_t data_pipe_4, uint8_t data_pipe_3,
							              uint8_t data_pipe_2, uint8_t data_pipe_1,
																 uint8_t data_pipe_0)
{
	uint8_t a= 0b00000000;
	a= (data_pipe_5<<5) | (data_pipe_4<<4) | (data_pipe_3<<3) | (data_pipe_2<<2) | (data_pipe_1<<1) | (data_pipe_0<<0);
	spi_write( CMD_W_REGISTER | REG_EN_RXADDR );
	spi_write( a );
}

uint8_t nrf_read_status_register()
{
	uint8_t flush_buf;
	flush_buf = SPDR;
	spi_write ( CMD_NOP);
		while ( ! ( SPSR & (1<<SPIF)));
		return ( SPDR );
}

void servo_pin_init()
{
	DDRD |= (1<<PD5);
}

void motor_pin_init()
{
	DDRD |= (1<<PD4);
}

void servo_and_motor_pwm_init()
{
	//PWM Ý timer1 de oluþturucaz 14 üncü wgm mod= tcnt1 in top deðeri ýcr1 olacak, 
	//ocr1a daki deðer ile kesiþene kadar set kesiþtikten sonra clear olacak 
	//cs yi 64 prescaler olarak ayarlayacaðýz. 16 mhz kristal ile 64 prescalerde
	//her tcnt1 bir artýþta 4us geçecek. 5000 e(0 dan saymaya baþladýðý için 4999 yazacaz) 
	//geldiðinde 20ms geçmiþ olacak ve böylece frekans 50 olmuþ olacak.
	//servo motor -90 için 20ms nin 0,52 mssi set olmasý lazým ocr1a = 130 yazýcaz (520/4=130)
	//servo motor 0 derece için 1,4ms set olmasý lazým ocr1a = 350 yazýcaz (1400/4=350)
	//servo motor +90 derece için 2,4 ms set olmasý lazým ocr1a =  600 yazýcaz (2400/4=600)
	TCNT1 = 0;
	ICR1 = 4999;
	/* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
	TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
}

void servo_go_minus_ninty_degree()
{
	OCR1A = 130;
	//_delay_ms(20);
	USART_putstring(dort);
}
void servo_go_plus_ninty_degree()
{
	OCR1A = 600;
	//_delay_ms(20);
	USART_putstring(bes);
}
void servo_go_zero_degree()
{
	OCR1A = 350;
	//_delay_ms(20);
	USART_putstring(alti);
}

void adc_init()
{
	DDRA &= ~(1<<PA3) & ~(1<<PA4);
	// ADMUX = REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
	//ADMUX |= (1<<REFS0) | (1<<ADLAR); // AVCC yi referans olarak ayarladým adc okumasýnýda sola yerleþimli olarak ayarladým sadece 8 bitlik msb kýsmýný okuyacam
    ADMUX |= (1<<REFS0) ;
	// ADCSRA = ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0); // ADC prescalerini fosc/64 olarak ayarladým(adc 250khzde çalýþacak) ve adcnin marþýna bastým 
} 

uint16_t adc_read_servo_pot()
{
	ADMUX = 0x40 | 0x03; // PA3 ü analog read giriþi olarak seçtim
	ADCSRA |= (1<<ADSC); // ADC yi çalýþtýrdýk
	while(!(ADCSRA & ~(1<<ADIF)));
	ADCSRA |= (1<<ADIF);
	uint16_t result=0;
	result =  ADCL;//ilk önce low byte okunmalý yoksaa okumaya izin vermiyor
	result += ((uint16_t)ADCH<<8); 
	//result |= ((uint16_t)ADCL>>6);
	//result |= ((uint16_t)ADCH<<2);
	return result;
}

uint16_t adc_read_motor_pot()
{
	ADMUX =  0x40 | 0x04; // PA4 ü analog read giriþi olarak seçtim
	ADCSRA |= (1<<ADSC); // ADC yi çalýþtýrdýk
	while(!(ADCSRA & ~(1<<ADIF)));
	ADCSRA |= (1<<ADIF);
	uint16_t result=0;
	result =  ADCL;
	result += ((uint16_t)ADCH<<8);
	//result |= ((uint16_t)ADCL>>6);
    //result |= ((uint16_t)ADCH<<2);
	return result;
}

void drive_servo_by_pot(uint16_t a)
{
	OCR1A = 130 + (a * 0.55); //To do a 16-bit write, the high byte must be written before the low byte. For a 16-bit read, the low byte must be read before the high byte.
	//ama c compileri bizim için 16 bit register okuma olayýný hallediyor
	/*
	OCR1A = 130 ;
	_delay_ms(1500);
	OCR1A = 600;
	_delay_ms(1500);
	OCR1A = 320;
	_delay_ms(1500);
	*/
}

void drive_motor_by_pot(uint16_t a)
{
	OCR1B = (a * 4.88);
	/*
	OCR1B = 3500 ;
	_delay_ms(1500);
	OCR1B = 4200;
	_delay_ms(1500);
	OCR1B = 4990;
	_delay_ms(1500);
	*/
}



//analog to digital dönüþümünü kullanýcam adc 3 te potumun referans bacaðý var. avccyi 100 nf kapasite ile topraða çektim. bilmem kaç henri bobin ile de vcc hattýna baðladým



//nrf i þuan tx moduna ayarlýyorum ve command göndericem ilk olarak

void ce( uint8_t level );
void csn( uint8_t level );
uint8_t write_spi(uint8_t data);
uint8_t read_register(uint8_t reg);
void write_register(uint8_t reg, uint8_t value);
void read_register_long(uint8_t reg, uint8_t* buf, uint8_t len);
void disableCRC(void);
void powerUp(void);
void powerDown(void);
void setPayloadSize(uint8_t size);
is_available available(void);
void nrf_init();
void tx_payload(uint8_t* buf, uint8_t len);
void rx_payload(uint8_t* buf, uint8_t len);
void nrf_listen();
void transmit();
void blink();

uint8_t flush_buffer;
uint16_t a=0;
uint16_t b=0;
char string_a[8];
char string_b[8];
unsigned char deger1 ;
unsigned char deger2 ;

uint8_t spidan_read;
uint8_t trash;

//uint8_t tx_data[] = {0x31,0x32,0x33};
//uint8_t rx_data[4];

uint8_t tx_data=0x33;
uint8_t rx_data;


uint8_t spidan_read_arr[8];

uint8_t status;
uint8_t config_reg;

int main(void){
	MCUCSR |= (1<<JTD);
	MCUCSR |= (1<<JTD);
	sei();
	//SREG |= (1<<7); // I BÝT SREG register is set and global interrupt enableD
	GICR |= (1<<6); // INT 0 pin gýcr register is set ýnt0 interrupt enableD
	// in gýfr register there is int 0 bit; it is set when irq triggered. and interrupts enabled
	// mcucr register int 0 bits indicate that low level on int0 pin trigger irq
	
	pin_init();
    spi_init() ;
	USART_init();
	
	
	uint8_t incoming_data;
	uint8_t status_state;
	uint8_t tx_payload = 0xAE;
	 
	USART_putstring(uc);
	 
	//nrf_chip_select();
	
	/*
	nrf_enabling_or_disabling_auto_acknowledgement ( 0,0,0,0,0,0 );
	nrf_enabling_or_disabling_rx_adress ( 0,0,0,0,0,1);
	nrf_setting_tx_adress ( tx_adress );
	nrf_setting_rx_adress ( 0, rx_adress_p0 );
	nrf_pwr_up ();
	nrf_enter_rx_mode ();
	
	
	nrf_listen_irq ();
	nrf_chip_not_enable();
	spi_write ( CMD_W_REGISTER | REG_STATUS );
	spi_write ( 0b01110000 ); //this two action were taken for cleaning interrupt flag 
	incoming_data = nrf_read_rx_fifo();
	USART_putstring(bir);
	
	
	/*
	servo_pin_init();
	motor_pin_init();
	servo_and_motor_pwm_init();
	adc_init();
	
	
	
	char kk[] = " ocr1a =  ";
	char ll[] = " ocr1b =  ";
	char mm[] = "                    ";
	*/
	/*
	uint8_t flush_buffer;
	uint16_t a=0;
	uint16_t b=0;
	char string_a[8];
	char string_b[8];
	unsigned char deger1 ;
	unsigned char deger2 ;
	uint8_t* ptr1 = (uint8_t*)calloc(2, 1);
	uint8_t* ptr2 = (uint8_t*)calloc(2, 1);
	uint8_t* ptr3 = (uint8_t*)calloc(2, 1);
	uint8_t spidan_read;
	
	uint8_t tx_data[] = {0x31,0x32,0x33};
	
	*/
	/*
	uint8_t* ptr1 = (uint8_t*)calloc(2, 1);
	uint8_t* ptr2 = (uint8_t*)calloc(2, 1);
	uint8_t* ptr3 = (uint8_t*)calloc(2, 1);
	*/
	
	nrf_init();
	nrf_listen();
	
	while(1)
	{
		
		if(available())
		{
		rx_payload(&rx_data,1);
		if(rx_data==0x33) blink();
		}
		/*
		for(int i=0; i<5; i++){flush_buffer = UDR;}
		USART_putstring("nrfe gönderilecek komutun byte sayýsýný girin \n\r");
		
		//_delay_ms(500);
		
		deger1 = USART_receive();
		
		USART_putstring(yeni_satir);
		USART_putstring("girdiginiz  deger = ");
		USART_send(deger1);
		deger1 &= 0x0F;
		ptr2 = realloc(ptr2,1);
		ptr3 = realloc(ptr3,1);
		ptr1[0] = deger1;
		
		USART_putstring(yeni_satir);
		USART_putstring("belirttiðiniz byte sayýsý kadar \n\rgöndereceðeniz commandý veya command + datayý giriniz =    \n\r");
		
		for(int i=0; i<deger1; i++)
		{
			deger2 = USART_receive();
			*(ptr2+i) = deger2;
		}
		
		USART_putstring("gönderilecek degerler = ");
		for(int i=0; i<deger1; i++)
		{
			USART_ptr_send((ptr2+i));
		}
		USART_putstring(yeni_satir);
		
		for(int i=0; i<deger1; i++)
		{
			*(ptr3+i) = *(ptr2+i) & 0x0F;
		}
		*/
		
		
		//USART_putstring(yeni_satir);
		
		
		
		/*
		drive_servo_by_pot(adc_read_servo_pot());
		
		a= OCR1A;
		itoa(a,string_a,10);
		
		//USART_putstring(kk);
		USART_putstring(string_a);
		USART_putstring(mm);
		//USART_putstring(yeni_satir);
		
		drive_motor_by_pot(adc_read_motor_pot());
		b= OCR1B;
		itoa(b,string_b,10);
		
		//USART_putstring(ll);
		USART_putstring(string_b);
		USART_putstring(yeni_satir);
		/*
		servo_go_minus_ninty_degree();
		_delay_ms(1500);
		servo_go_zero_degree();
		_delay_ms(1500);
		servo_go_plus_ninty_degree();
		_delay_ms(1500);
	    */	
	
		//PORTA |= (1<<2);
		/*
		_delay_ms(500);
		PORTA &= ~(1<<2);
		_delay_ms(500);
		
		if ( incoming_data ==0xAE )
		{
		
			PORTA |= (1<<2);
			_delay_ms(500);
			PORTA &= ~(1<<2);
			
			USART_putstring(iki);
		}
		*/
	}
	
	
	return(0);
}


ISR(INT0_vect){
	//nrf_write_tx_fifo ( &tx_payload, sizeof(tx_payload));
	

	PORTA |= (1<<2);
	_delay_ms(500);
	PORTA &= ~(1<<2);
	_delay_ms(500);
	USART_putstring("nrfe command ve data? gönderiliyor........\n\r");
	
	//spidan_read = spi_write_with_read_status(CMD_R_REGISTER | REG_STATUS);
	nrf_chip_select();
	status = write_spi(CMD_R_REGISTER | REG_STATUS);
	nrf_chip_not_select();
	USART_putstring("nrften gelen status reg degeri =     \n\r");
	USART_print_bcd(status);
	USART_putstring(yeni_satir);
	
	spidan_read = read_register(REG_SETUP_AW);
	USART_putstring("reg en_aa degerleri önce =     \n\r");
	USART_print_bcd(spidan_read);
	USART_putstring(yeni_satir);
	
	write_register(REG_SETUP_AW, 0x01);
	_delay_ms(1000);
	spidan_read = read_register(REG_SETUP_AW);
	USART_putstring("reg en_aa degerleri sonra =     \n\r");
	USART_print_bcd(spidan_read);
	USART_putstring(yeni_satir);
	
	/*
	nrf_chip_select();
	nrf_pwr_up();
	nrf_chip_not_select();
	
	USART_putstring("nrf pwr up yapýlýyor ve tx moduna alýndý \n\r");
	
	nrf_chip_enable();
	USART_putstring("chip enable edildi \n\r");
	nrf_chip_select();
	nrf_write_tx_fifo(tx_data, sizeof(tx_data));
	nrf_chip_not_select();
	USART_putstring("tx fifoya yazýldý \n\r");
	nrf_chip_not_enable();
	
	spi_write(CMD_R_REGISTER | REG_STATUS);
	spidan_read =  spi_read();
	USART_putstring("nrften gelen status reg degeri =     \n\r");
	USART_print_bcd(spidan_read);
	USART_putstring(yeni_satir);
	
	
	spi_write(CMD_W_REGISTER | REG_STATUS);
	spi_write(0xFF);
	
	USART_putstring(" status reg tx flag temizlendi     \n\r");
	
	spi_write(CMD_R_REGISTER | REG_STATUS);
	spidan_read =  spi_read();
	USART_putstring("nrften gelen status reg degeri =     \n\r");
	USART_print_bcd(spidan_read);
	USART_putstring(yeni_satir);
	*/
	
	
	
	
}
/*
void ce( uint8_t level );
void csn( uint8_t level );
uint8_t write_spi(uint8_t data);
uint8_t read_register(uint8_t reg);
void write_register(uint8_t reg, uint8_t value);
void read_register_long(uint8_t reg, uint8_t* buf, uint8_t len);
void disableCRC(void);
void powerUp(void);
void powerDown(void);
void setPayloadSize(uint8_t size);
is_available available(void);
void nrf_init();
void tx_payload(uint8_t* buf, uint8_t len);
void rx_payload(uint8_t* buf, uint8_t len);
void nrf_listen();
void transmit();
void blink();
*/
void ce( uint8_t level )
{
	if(level==0) PORTA &= ~(1<<0);
	else PORTA |= (1<<0);
}

void csn( uint8_t level )
{
	if(level==0) PORTB &= ~(1<<SS);
	else PORTB |= (1<<SS);
}

uint8_t write_spi(uint8_t data)
{
	SPDR = data;
	asm volatile("nop");
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}


uint8_t read_register(uint8_t reg)
{
	uint8_t result;
	csn(LOW);
	status = write_spi(CMD_R_REGISTER | reg);
	result = write_spi(0xAA);	
	csn(HIGH);
	return result;
}

void read_register_long(uint8_t reg, uint8_t* buf, uint8_t len)
{
	csn(LOW);
	status = write_spi(CMD_R_REGISTER | reg);
	while(len--)
	{
		*buf++ = write_spi(0xFF);
	}
	csn(HIGH);
}

void write_register(uint8_t reg, uint8_t value)
{
	csn(LOW);
	status = write_spi(CMD_W_REGISTER | reg);
	trash = write_spi(value);
	csn(HIGH);
}

void write_register_long(uint8_t reg, uint8_t* buf, uint8_t len)
{
	csn(LOW);
	status = write_spi(CMD_W_REGISTER | reg);
	while(len--)
	{
		write_spi(*buf++);
	}
	csn(HIGH);
}
/*
cihazý açma ve power down modunda iken 
crc disable etme 
auto acký disable etme rx0 ve tx adreslerini yazma 
0.pipe aktif hale getirme 


*/

void disableCRC(void)
{
    config_reg = (uint8_t)(config_reg & ~(1<<EN_CRC));
    write_register(REG_CONFIG, config_reg);
}

void powerUp(void)
{
    // if not powered up then power up and wait for the radio to initialize
    if (!(config_reg & (1<<PWR_UP))) {
        config_reg |= (1<<PWR_UP);
        write_register(REG_CONFIG, config_reg);
        _delay_ms(5);
    }
}

void powerDown(void)
{
    ce(LOW); // Guarantee CE is low on powerDown
    config_reg = (uint8_t)(config_reg & ~(1<<PWR_UP));
    write_register(REG_CONFIG, config_reg);
}

void setPayloadSize(uint8_t size)
{
    // payload size must be in range [1, 32]
    // write static payload size setting for all pipes
    for (uint8_t i = 0; i < 6; ++i) {
        write_register((uint8_t)(REG_RX_PW_P0 + i), size);
    }
}

is_available available(void)
{
	if(read_register(REG_FIFO_STATUS) & 0x01)return yes;
	else return no;
}

void nrf_init()
{
	config_reg=0;
	write_register(REG_EN_AA,0x00); //auto acknoeledgementi tüm pipelar için kapattým
	write_register(REG_CONFIG,0x00); // crc yi kapattým
	write_register(REG_EN_RXADDR,0x01); // sadece pipe 0 rx alabilecek
	write_register_long(REG_TX_ADDR,&tx_adress,5); 
	write_register_long(REG_RX_ADDR_P0,&rx_adress_p0,5);
	write_register(REG_RX_PW_P0,0x01);//pipe 0 dan 1 byte geleceðini söyledim gelirse
}

void tx_payload(uint8_t* buf, uint8_t len)
{
	status = write_spi(CMD_R_RX_PAYLOAD);
	while(len--)
	{
		*buf++ = write_spi(0xFF);
	}
}

void rx_payload(uint8_t* buf, uint8_t len)
{
	status = write_spi(CMD_R_RX_PAYLOAD);
	while(len--)
	{
		*buf++ = write_spi(0xFF);
	}
}

void nrf_listen()
{
	nrf_init();
	config_reg |= (1<<PRIM_RX);
	write_register(REG_CONFIG,config_reg);
	powerUp();
	ce(HIGH);
	/**
	if(available())
	{
		rx_payload(&rx_data,1);
		if(rx_data==0x33) blink();
	}
	*/
}	

void transmit()
{
	nrf_init();
	config_reg &= ~(1<<PRIM_RX);
	write_register(REG_CONFIG,config_reg);
	tx_payload(&tx_data,1);
	powerUp();
	ce(HIGH);
}

void blink()
{
	PORTA |= (1<<2);
	_delay_ms(500);
	PORTA &= ~(1<<2);
	_delay_ms(500);
}
