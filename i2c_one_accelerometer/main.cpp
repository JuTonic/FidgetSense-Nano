#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

const uint8_t ADXL345_ADDR = 0x53;

volatile uint32_t timer_millis = 0;

void setup_timer() {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8
    OCR1A = 1999; // 16MHz/8/2000 = 1000Hz
    TIMSK1 = (1 << OCIE1A);
    sei();
}

ISR(TIMER1_COMPA_vect) {
    timer_millis++;
}

uint32_t millis() {
    uint32_t m;
    cli(); 
    m = timer_millis;
    sei(); 
    return m;
}

void i2c_init() {
    TWSR = 0;
    TWBR = ((F_CPU / 100000) - 16) / 2;
    TWCR = (1 << TWEN);

    _delay_ms(1000);
}

void i2c_start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
}

void i2c_stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO)) {
    }
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
}

uint8_t i2c_read(bool ack) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    while (!(TWCR & (1 << TWINT))) {
    }
    return TWDR;
}

void write_reg(uint8_t reg, uint8_t value) {
    i2c_start();
    i2c_write(ADXL345_ADDR << 1);
    i2c_write(reg);
    i2c_write(value);
    i2c_stop();
}

void read_accel(int16_t &x, int16_t &y, int16_t &z) {
    i2c_start();
    i2c_write(ADXL345_ADDR << 1);
    i2c_write(0x32);
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x01);
    
    uint8_t xl = i2c_read(true), xh = i2c_read(true);
    uint8_t yl = i2c_read(true), yh = i2c_read(true);
    uint8_t zl = i2c_read(true), zh = i2c_read(false);
    
    i2c_stop();
    
    x = (xh << 8) | xl;
    y = (yh << 8) | yl;
    z = (zh << 8) | zl;
}

void serial_write(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = c;
}

void serial_print(const char *str) {
    while (*str) {
        serial_write(*str++);
    }
}

void print_number(long num) {
    if (num >= 10) {
        print_number(num / 10);
    }

    serial_write('0' + (num % 10));
}

// void enable_pullups() {
//     PORTC |= (1 << PC4) | (1 << PC5);
//     _delay_ms(10);
// }

int main() {
    UBRR0L = 8; // 115200 baud
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    i2c_init();

    setup_timer();
    
    write_reg(0x2D, 0x08); // Power on
    write_reg(0x31, 0x00); // ±2g
    write_reg(0x2C, 0x0C); // 400Hz
    // write_reg(0x2C, 0x0F); // 3200Hz
    
    while (1) {
        int16_t x, y, z;
        read_accel(x, y, z);
        
        print_number(millis());
        serial_print(";");
        print_number(x);
        serial_print(";");
        print_number(y);
        serial_print(";");
        print_number(z);
        serial_print("\n");
        _delay_ms(100);
    }
}