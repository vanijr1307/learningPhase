#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL // 16 MHz crystal

// Initialize UART with given baud rate
void uart_init(uint32_t baud) {
    uint16_t ubrr_val = (F_CPU / (16UL * baud)) - 1;

    // Set baud rate registers
    UBRR0H = (ubrr_val >> 8) & 0xFF;
    UBRR0L = ubrr_val & 0xFF;

    // Enable transmitter only
    UCSR0B = (1 << TXEN0);

    // 8-bit data, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send one character
void uart_send_char(char c) {
    // Wait until the data register is empty
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c; // Send character
}

// Send a string
void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

int main(void) {
    // Initialize UART at 9600 baud
    uart_init(9600);

    while (1) {
        // Send string over UART
        uart_send_string("Hello Saleae! UART test.\r\n");

        _delay_ms(500); // Wait 0.5 second

    }
}
