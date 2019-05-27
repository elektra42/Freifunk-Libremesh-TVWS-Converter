/* **********************************************************************
 * AVR-GCC source code for frequency transverter
 * Copyright (C) 2019  by Corinna 'Elektra' Aichele 
 * 
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This source code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this source file. If not, see http://www.gnu.org/licenses/. 
 *************************************************************************/


#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
// #include <util/atomic.h>
// #include <string.h>

/* UART settings */

#ifndef F_CPU
#warning "F_CPU not defined in Makefile. Using 3.686400 MHz"
#define F_CPU 3686400UL 
#endif

/* Define baud rate for serial port */
#define BAUD 115200UL

/* Calculate UART BAUD register setting */
#define UBRR_SETTING ((F_CPU+BAUD*8)/(BAUD*16)-1)

// MCU, firmware revision and controller type
#define FIRM_REV "ATmega8_A_1"

#define UART_MAXSTRLEN 11

/*
#define Mode_Mixer_A PD5 // MODEPIN=23
#define SDATA_A PD6 // SDATAPIN=22
#define ENBL_A PD7 // ENBLPIN=21
#define SCLK_A PB0 // SCLKPIN=19
#define ENX_A PB1 // ENXPIN=20
#define Reset_A PB2

#define Mode_Mixer_B PC5
#define SDATA_B PC1
#define ENBL_B PC0
#define SCLK_B PC2
#define ENX_B PC4 
#define Reset_B PC3
*/

    // uint8_t length = 1;
    uint8_t step = 0x1;
    uint16_t ticks;
    uint16_t count;
    char vo[] = "";
    volatile uint8_t uart_rx_str_complete = 0; 
    volatile uint8_t uart_rx_str_count = 0;
    volatile char uart_rx_string[UART_MAXSTRLEN + 1];
    char receive_rx_serial_char;
    char config_string[8] = "0";
    uint16_t new_config_value = 0;
    uint16_t watchdog_timer_eeprom EEMEM;
    uint16_t lvd_on_eeprom EEMEM;
    uint16_t lvd_off_eeprom EEMEM;
    uint8_t trigger_eeprom_reload = 0;
    uint8_t disable_serial_messages = 1;


/* UART send single character */
int uart_putc(char c)
    {
        // wait until transmit is possible 
        while (!(UCSRA & (1<<UDRE)))  
        {
        }                             
	// send character
        UDR = c;
        return 0;
        
        }

    
/* UART send string */
void uart_puts(char *s)
    {
        while (*s)
        {   //transmit as long as  *s != '\0' 
            uart_putc(*s);
            s++;
        }
    }
        


void bit_set(uint8_t bit, uint8_t *byte)
//Set bit in bit position, starting with 0 from the right in *byte
{
    bit = 1 << bit;
    *byte = *byte | bit;
}

int bit_read(uint8_t bit, uint8_t byte)
//Read bit in bit position, starting with 0 from the right in byte
{
    return((byte >> bit)  & 0x01);
    
}



void send_address_A(int address_byte) {
    
            // Read bits 6 to 0 in address byte
            // Make sure SDATA_A is set to output
            DDRD = (1<<PD6);
            uint8_t bit_position = 7;
            uint8_t highlow;
        
            while (bit_position != 0){
            bit_position--;
            PORTB = (0<<PB0);
            highlow = bit_read(bit_position, address_byte);
            itoa(highlow,vo,10);
            uart_puts (vo);
            PORTD = (highlow << PD6);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            }
            uart_puts (" ");
            }
            

void send_address_B(int address_byte) {
    
            // Read bits 6 to 0 in address byte
            // Make sure SDATA_B is set to output
            uint8_t bit_position = 7;
            uint8_t highlow;
            
            while (bit_position != 0){
            bit_position--;
            PORTC = (0<<PC2);
            highlow = bit_read(bit_position, address_byte);
            
            PORTC = (highlow << PC1);
            _delay_ms(1);
            PORTC = (1<<PC2);
            _delay_ms(1);
            }
            }

            
void initialize_write_mixer_A (void) {
    
            DDRD = (1<<PD6);
            PORTB = (0<<PB0);
            PORTB = (1<<PB1);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTB = (0<<PB0);
            PORTB = (0<<PB1);
            PORTD = (1<<PD6);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTD = (0<<PD6);
            PORTB = (0<<PB0);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
}            

void initialize_write_mixer_B (void) {
    
            DDRC = (1<<PC1);
            PORTC = (0<<PC2);
            PORTC = (1<<PC4);
            _delay_ms(1);
            PORTC = (1<<PC2);
            _delay_ms(1);
            PORTC = (0<<PC2);
            PORTC = (0<<PC4);
            PORTC = (1<<PC1);
            _delay_ms(1);
            PORTC = (1<<PC2);
            _delay_ms(1);
            PORTC = (0<<PC1);
            PORTC = (0<<PC2);
            _delay_ms(1);
            PORTC = (1<<PC2);
            _delay_ms(1);
}


void initialize_read_mixer_A (void) {
    
            DDRD = (0<<PD6);
            PORTB = (0<<PB0);
            PORTB = (1<<PB1);
            _delay_ms(3);
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTB = (0<<PB0);
            PORTB = (0<<PB1);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTB = (0<<PB0);
            DDRD = (1<<PD6);
            PORTD = (1<<PD6);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
}

void initialize_read_mixer_B (void) {
    
            DDRC = (0<<PC1);
            PORTC = (0<<PC2);
            PORTC = (1<<PC4);
            _delay_ms(3);
            PORTC = (1<<PC2);
            _delay_ms(1);
            PORTC = (0<<PC2);
            PORTC = (0<<PC4);
            _delay_ms(1);
            PORTC = (1<<PC2);
            _delay_ms(1);
            PORTC = (0<<PC2);
            DDRC = (1<<PC1);
            PORTC = (1<<PC1);
            _delay_ms(1);
            PORTC = (1<<PC2);
            _delay_ms(1);
}


struct result_settings_bytes {
            uint8_t high_byte;
            uint8_t low_byte;
            };



struct result_settings_bytes read_settings_mixer_A(void) {
            
            struct result_settings_bytes mixer_A;
            // Make sure SDATA_A is set to input
            DDRD = (0<<PD6);
            PORTB = (0<<PB0);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTB = (0<<PB0);
            _delay_ms(1);
            
            mixer_A.high_byte = 0;
            mixer_A.low_byte = 0;
            // uart_puts ("Debug: Reading registers\r\n");
            // Read bits 7 to 0 in mixer register
            uint8_t bit_position = 8;
            
            while (bit_position != 0){
            bit_position--;
            PORTB = (1<<PB0);
            _delay_ms(1);
            if (PIND & (1<<PD6)){
            bit_set(bit_position, &mixer_A.high_byte);
            uart_puts ("1");
            }
            else {uart_puts ("0");}
            
            PORTB = (0<<PB0);
            _delay_ms(1);
            }
            
            uart_puts (" ");
            bit_position = 8;
            
            while (bit_position != 0){
            bit_position--;
            
            /*itoa(bit_position, vo, 10);
            uart_puts(vo);
            uart_puts("\r\n");*/
            PORTB = (1<<PB0);
            _delay_ms(1);
            if (PIND & (1<<PD6)){
            bit_set(bit_position, &mixer_A.low_byte);
            uart_puts ("1");
            }
            else {uart_puts ("0");}
            PORTB = (0<<PB0);
            _delay_ms(1);
            }
    
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTB = (0<<PB0);
            _delay_ms(1);
            PORTB = (1<<PB1);
            uart_puts (" ");
    
    return mixer_A;
}





void send_data_to_mixer_A(uint8_t data_byte_high, uint8_t data_byte_low) {
    
            // Read bits 7 to 0 in address byte
            // Make sure SDATA_A is set to output
            uint8_t bit_position = 8;
            uint8_t highlow;
            
            while (bit_position != 0){
            --bit_position;
            highlow = bit_read(bit_position, data_byte_high);
            itoa(highlow, vo, 10);
            uart_puts(vo);
            PORTD = (highlow << PD6);
            PORTB = (0<<PB0);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            }
            uart_puts(" ");
            bit_position = 8;
            while (bit_position != 0){
            --bit_position;
            highlow = bit_read(bit_position, data_byte_low);
            itoa(highlow, vo, 10);
            uart_puts(vo);
            PORTD = (highlow << PD6);
            PORTB = (0<<PB0);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            }

            
            
            PORTB = (0<<PB0);
            PORTB = (1<<PB1);
            _delay_ms(1);
            PORTB = (1<<PB0);
            _delay_ms(1);
            PORTB = (0<<PB0);
            uart_puts(" ");
            }



void read_A(int address_byte){
    
initialize_read_mixer_A();
send_address_A(address_byte);
struct result_settings_bytes result_settings_bytes_A;
result_settings_bytes_A = read_settings_mixer_A();
uart_puts ("A: 0x");
    if (address_byte <= 15) {
            uart_puts ("0");
            }
itoa(address_byte,vo,16);
uart_puts (vo);
uart_puts (" 0x");
itoa(result_settings_bytes_A.high_byte,vo,16);
    if (result_settings_bytes_A.high_byte <= 15) {
        uart_puts ("0");
        }
uart_puts (vo);
itoa(result_settings_bytes_A.low_byte,vo,16);
    if (result_settings_bytes_A.low_byte <= 15) {
        uart_puts ("0");
        }

uart_puts (vo);
uart_puts ("\r\n");
}

void dump_A(void){
    uart_puts ("\r\nAll data registers of mixer A:\r\n\r\nAddress HighByte Low Byte\r\n");
    int address_byte; 
    for (address_byte = 0; address_byte < 32; address_byte++){
        read_A(address_byte);
    }
    uart_puts ("\r\n");
}

void write_A(int address_byte, uint8_t data_byte_high, uint8_t data_byte_low){
    uart_puts ("\r\nSetting register of mixer A:\r\n");
    initialize_write_mixer_A();
    send_address_A(address_byte);
    send_data_to_mixer_A(data_byte_high, data_byte_low);
    uart_puts("A: 0x");
    if (address_byte <= 15) {
        uart_puts ("0");
        }
    itoa(address_byte, vo, 16);
    uart_puts(vo);
    uart_puts(" 0x");
    if (data_byte_high <= 15) {
        uart_puts ("0");
        }
    itoa(data_byte_high, vo, 16);
    uart_puts(vo);
    if (data_byte_low <= 15) {
        uart_puts ("0");
        }
    itoa(data_byte_low, vo, 16);
    uart_puts(vo);
    uart_puts("\r\n");
}


void serial_terminal (void)
    {
        
        uart_puts ("\r\n");
        uart_puts ("Board firmware: ");
        uart_puts (FIRM_REV);
        uart_puts ("\r\n\r\n");
        uart_puts ("Commands:\r\nh = help\r\np = print all mixer settings\r\nex = enable mixer x (a or b or ab))\r\ndx = disable mixer x (a or b or ab))\r\ncXx00x0000 = configure mixer X (a or b):register:hexcode)\r\n");
        uart_puts ("\r\n"); 
        
    }

void serial_config (void) {

    
  if (uart_rx_str_count > 12) {
       
      uart_rx_str_complete = 0;
      uart_rx_str_count = 0;
      uart_rx_string[0] = '\0';
  }
   
    if (uart_rx_str_complete == 1) {
      
      if (uart_rx_string[0] == 'p') {
           
           dump_A();
       }
      
      if (uart_rx_string[0] == 'h') {
           
           serial_terminal();
       }
       
     if (uart_rx_string[0] == 'e' && uart_rx_string[1] == 'a' && uart_rx_string[2] != 'b') {
           
           // Enable Mixer A PD7
              PORTD = (1<<PD7);
            uart_puts("Mixer A enabled.\r\n");
        }
        
        
    if (uart_rx_string[0] == 'e' && uart_rx_string[1] == 'b') {
        
        // Enable Mixer B PC0
           PORTC = (1<<PC0);
           uart_puts("Mixer B enabled.\r\n");
        }
       
    if (uart_rx_string[0] == 'e' && uart_rx_string[1] == 'a' && uart_rx_string[2] == 'b' ) {
        
        // Enable Mixer A PD7
           PORTD = (1<<PD7);
        
        // Enable Mixer B PC0
           PORTC = (1<<PC0);
           uart_puts("Mixers A and B enabled.\r\n");
        }  
       
     if (uart_rx_string[0] == 'd' && uart_rx_string[1] == 'a' && uart_rx_string[2] != 'b') {
           
           // Enable Mixer A PD7
              PORTD = (1<<PD7);
            uart_puts("Mixer A disabled.\r\n");
        }
        
        
    if (uart_rx_string[0] == 'd' && uart_rx_string[1] == 'b') {
        
        // Enable Mixer B PC0
           PORTC = (1<<PC0);
           uart_puts("Mixer B disabled.\r\n");
        }
       
    if (uart_rx_string[0] == 'd' && uart_rx_string[1] == 'a' && uart_rx_string[2] == 'b' ) {
        
        // Enable Mixer A PD7
           PORTD = (1<<PD7);
        
        // Enable Mixer B PC0
           PORTC = (1<<PC0);
           uart_puts("Mixers A and B disabled.\r\n");
        }
    
    if (uart_rx_string[0] == 'c' && uart_rx_string[1] == 'a' && uart_rx_string[2] == 'x' && uart_rx_str_count == 10 ) {
        
        uart_puts("Setting register\r\n");
    
           // This is our required input format: cXx00x0000 = configure mixer X (a or b):register:hexcode)
           // define local variables
           char address_byte_hex[2];
           char data_byte_high_hex[3];
           char data_byte_low_hex[3];
           long address_byte_long;
           long data_byte_high_long;
           long data_byte_low_long;
           char *pointer;
           int address_byte;
           uint8_t data_byte_high;
           uint8_t data_byte_low;
           
           // Pick the config chars from input string
           address_byte_hex[0] = uart_rx_string[3];
           address_byte_hex[1] = uart_rx_string[4];
           data_byte_high_hex[0] = uart_rx_string[6];
           data_byte_high_hex[1] = uart_rx_string[7];
           data_byte_low_hex[0] = uart_rx_string[8];
           data_byte_low_hex[1] = uart_rx_string[9];
           
           // Convert hex to long using strtol function
           address_byte_long = strtol(address_byte_hex, &pointer, 16);
           data_byte_high_long = strtol(data_byte_high_hex, &pointer, 16);
           data_byte_low_long = strtol(data_byte_low_hex, &pointer, 16);
           
           // Cast long to int
           address_byte = (int) address_byte_long;
           data_byte_high = (uint8_t) data_byte_high_long;
           data_byte_low = (uint8_t) data_byte_low_long;
           
           write_A(address_byte, data_byte_high, data_byte_low);
           
       //new_config_value = atol(config_string);
    }

      
      uart_rx_str_complete = 0;
      
      uart_rx_str_count = 0;
      
      uart_rx_string[0] = '\0';
     
    }
}

/***********************   Main  *************************************/

int main(void)
{
    
    // Set up GPIOs as output communication ports to mixer A and B
    
    DDRB |= (1<<PB0)| (1<<PB1)| (1<<PB2);
    DDRC |= (1<<PC0)| (1<<PC2)| (1<<PC3)| (1<<PC4)| (1<<PC5);
    DDRD |= (1<<PD5) | (1<<PD7);
    
    PORTB = (1<<PB2);
    
     _delay_ms(100);
     
    PORTB = (0<<PB2); 
    
     _delay_ms(100);
     
    PORTB = (1<<PB2);
    
 
    // Set prescaler of Timer/Counter2 to 1024
    TCCR2 |= ( 1<<CS02 )| ( 1<<CS01)| ( 1<<CS00 ); 
    
    
     // Enable UART 
        UBRRH = UBRR_SETTING >> 8; 
        UBRRL = UBRR_SETTING & 0xFF;
        
        
        UCSRB |= (1<<RXCIE) | (1<<RXEN) | (1<<TXEN);
        //UCSRB |= (1<<TXEN);
        
	// select asynchronous mode 8N1 
       UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // asynchronous mode 8N1 
    
       
       _delay_ms(800);
       
       
    
        _delay_ms(5);


         // Enable interrupts
        sei();
        
        uart_puts("Ready. Type 'h' and return for help\r\n");
        

/***********************   Main program loop  *************************************/


while (1) {
    
 
        serial_config();
        
        _delay_ms(100);

        
/*        if (trigger_eeprom_reload == 1) {
            apply_eeprom_settings(); }
*/
       
          
}

   return 0; // never reached 
}


/***********************************
 * Interrupt service routines ******
 * ********************************/
        
        
      

ISR(USART_RXC_vect) {
    
  // read data from buffer 
    receive_rx_serial_char = UDR;
    uart_putc(receive_rx_serial_char);
     
    if( uart_rx_str_complete == 0 ) {
 
    if( receive_rx_serial_char != '\n' &&
        receive_rx_serial_char != '\r' &&
        uart_rx_str_count < UART_MAXSTRLEN ) {
        uart_rx_string[uart_rx_str_count] = receive_rx_serial_char;
        uart_rx_str_count++; }
        
    else {
      
      uart_rx_string[uart_rx_str_count] = '\0';
      uart_rx_str_complete = 1;
        
      }
    }
 }

    
