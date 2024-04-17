#include "nrf24lib.h"
#include "usart_atmega.h"

uint8_t tx_data=0x33;
uint8_t rx_data;


uint8_t spidan_read_arr[8];

uint8_t status;
uint8_t config_reg;

uint8_t tx_adress[5] = { 0x78, 0x78, 0x78, 0x78, 0x78 };
uint8_t rx_adress_p0[5] = { 0x78, 0x78, 0x78, 0x78, 0x78 };



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
cihazı açma ve power down modunda iken 
crc disable etme 
auto ackı disable etme rx0 ve tx adreslerini yazma 
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
	
	write_register(REG_EN_RXADDR,0x01); // sadece pipe 0 rx alabilecek
	write_register_long(REG_TX_ADDR,&tx_adress,5); 
	write_register_long(REG_RX_ADDR_P0,&rx_adress_p0,5);
	write_register(REG_RX_PW_P0,0x01);//pipe 0 dan 1 byte geleceğini söyledim gelirse
}

void tx_payload(uint8_t* buf, uint8_t len)
{
	status = write_spi(CMD_W_TX_PAYLOAD);
	while(len--)
	{
		write_spi(*buf++);
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

void clear_nrf_status_flag()
{
	write_register(NRF_STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT)); // statustaki interrupt flagları temizliyor
}

void set_retries(uint8_t delay, uint8_t count) // msb 4 bit bekleme zamanı 15 e kadar değer alır. lsb 4 bit tekrar sayısı 15 e kadar değer alır
{	
	write_register(REG_SETUP_RETR, delay<<4 | count);
}

void set_channel(uint8_t channel) // 0 ile 125 arasında olmalı
{
	write_register(REG_RF_CH, channel);
}

void set_nrf_data_rate(nrf_datarate_e datarate)
{
	uint8_t k= read_register(REG_RF_SETUP);
	k |= (((uint8_t)datarate & 0x02)<<4) | (((uint8_t)datarate & 0x01)<<3);
	write_register(REG_RF_SETUP, k);
}

void set_nrf_rf_power(rf_power_e rfpower)
{
	uint8_t k= read_register(REG_RF_SETUP);
	k |= (((uint8_t)rfpower & 0x02)<<1) | (((uint8_t)rfpower & 0x01)<<1);
	write_register(REG_RF_SETUP, k);
}



void print_usart_reg_value(uint8_t reg)
{
	uint8_t k= read_register(reg);
	USART_putstring("nrften gelen ");
	switch (reg)
	{
		case REG_CONFIG: USART_putstring(" config register "); break;
		case REG_STATUS: USART_putstring(" status register "); break;	
		case REG_RF_CH: USART_putstring(" rf channel register "); break;
	    case REG_DYNPD: USART_putstring(" dynamic payload register "); break;
		case REG_EN_AA: USART_putstring(" enable auto ack register "); break;
		case REG_EN_RXADDR: USART_putstring(" enable rx adress register "); break;
		case REG_FEATURE: USART_putstring(" feature register "); break;
		case REG_FIFO_STATUS: USART_putstring(" fıfo status register "); break;
		case REG_OBSERVE_TX: USART_putstring(" observe tx register "); break;
		case REG_RF_SETUP: USART_putstring(" rf setup register "); break;
		case REG_RPD: USART_putstring(" received power detector register "); break;
		case REG_RX_ADDR_P0: USART_putstring(" rx pipe 0 adress register "); break;
		case REG_RX_ADDR_P1: USART_putstring(" rx pipe 1 adress register "); break;
		case REG_RX_PW_P0: USART_putstring(" rx pipe 0 payload width register "); break;
		case REG_RX_PW_P1: USART_putstring(" rx pipe 1 payload width register "); break;
		case REG_SETUP_AW: USART_putstring(" setup adress width register "); break;
		case REG_TX_ADDR: USART_putstring(" tx adress register "); break;
		case REG_SETUP_RETR: USART_putstring(" setup retransmit register "); break;
	}
	USART_putstring(" degeri = \n\r");
	USART_print_bcd(k);
	USART_putstring(yeni_satir);
}

uint8_t flush_rx()
{
	return write_spi(CMD_FLUSH_RX);
}

uint8_t flush_tx()
{
	return write_spi(CMD_FLUSH_TX);
}
