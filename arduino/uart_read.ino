#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL // 16 MHz crystal
#define BUFFER_SIZE 64

// Initialize UART with given baud rate
void uart_init(uint32_t baud) {
    uint16_t ubrr_val = (F_CPU / (16UL * baud)) - 1;

    // Set baud rate
    UBRR0H = (ubrr_val >> 8) & 0xFF;
    UBRR0L = ubrr_val & 0xFF;

    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    // 8-bit data, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send one character
void uart_send_char(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for data register empty
    UDR0 = c;
}

// Send a string
void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

// Receive one character (blocking)
char uart_receive_char() {
    while (!(UCSR0A & (1 << RXC0))); // Wait until data is received
    return UDR0;
}

int main(void) {
    char buffer[BUFFER_SIZE];
    uint8_t idx = 0;

    uart_init(9600); // Initialize UART at 9600 baud

    uart_send_string("UART Echo Ready!\r\n");

    while (1) {
        char received = uart_receive_char(); // Read a character

        // Store in buffer
        buffer[idx++] = received;

        // Echo back immediately
        uart_send_char(received);

        // Reset buffer if full
        if (idx >= BUFFER_SIZE) {
            idx = 0;
        }
      
    }
}
