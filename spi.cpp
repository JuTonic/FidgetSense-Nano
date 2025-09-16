#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// SPI Chip Select pins
#define CS1_PIN     PB2  // D10
#define CS2_PIN     PB1  // D9  
#define CS3_PIN     PB0  // D8

// SPI port definitions
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define MOSI        PB3   // D11
#define MISO        PB4   // D12
#define SCK         PB5   // D13

// ADXL345 registers
#define DEVID          0x00
#define POWER_CTL      0x2D
#define DATA_FORMAT    0x31
#define BW_RATE        0x2C
#define DATAX0         0x32

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

struct accel_data_t {
    int16_t x;
    int16_t y;
    int16_t z;
} ;

accel_data_t sensor_data[3];
bool sensor_status[3];

void setup_serial() {
    UBRR0L = 8;  // 115200 baud at 16MHz
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    _delay_ms(100);
}

void serial_print_char(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = c;
}

void serial_print(const char *str) {
    while (*str) {
        serial_print_char(*str++);
    }
}

void print_uint32(uint32_t num) {
    if (num >= 10) {
        print_uint32(num / 10);
    }

    serial_print_char('0' + (num % 10));
}

void print_int16(int16_t value) {
    if (value < 0) {
        print_int16(-value);
        serial_print_char('-');
        return;
    }

    print_uint32(static_cast<uint32_t>(value));
}

void spi_init() {
    // Set MOSI, SCK, and CS pins as output
    SPI_DDR |= (1 << MOSI) | (1 << SCK) | (1 << CS1_PIN) | (1 << CS2_PIN) | (1 << CS3_PIN);
    // Set MISO as input
    SPI_DDR &= ~(1 << MISO);
    
    // Enable SPI, Master, set clock rate fck/16 (1MHz at 16MHz CPU), MODE 3
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << CPOL) | (1 << CPHA);
    SPSR = 0x00;
    
    // Deselect all chips
    SPI_PORT |= (1 << CS1_PIN) | (1 << CS2_PIN) | (1 << CS3_PIN);
}

uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))) {}
    return SPDR;
}

void adxl345_write(uint8_t cs_pin, uint8_t reg, uint8_t data) {
    SPI_PORT &= ~(1 << cs_pin);
    _delay_us(5);
    spi_transfer(reg & 0x7F);
    spi_transfer(data);
    _delay_us(5);
    SPI_PORT |= (1 << cs_pin);
    _delay_us(5);
}

uint8_t adxl345_read(uint8_t cs_pin, uint8_t reg) {
    SPI_PORT &= ~(1 << cs_pin);
    _delay_us(5);
    spi_transfer(0x80 | reg);
    uint8_t data = spi_transfer(0x00);
    _delay_us(5);
    SPI_PORT |= (1 << cs_pin);
    _delay_us(5);
    return data;
}

bool adxl345_init(uint8_t cs_pin, uint8_t sensor_index) {
    // Check WHO_AM_I register
    uint8_t whoami = adxl345_read(cs_pin, DEVID);
    if (whoami != 0xE5) {
        return false;
    }
    
    // Initialize device
    adxl345_write(cs_pin, POWER_CTL, 0x00);
    _delay_ms(10);
    adxl345_write(cs_pin, DATA_FORMAT, 0x00);
    _delay_ms(10);
    adxl345_write(cs_pin, BW_RATE, 0x0A);
    _delay_ms(10);
    adxl345_write(cs_pin, POWER_CTL, 0x08);
    _delay_ms(10);
    
    return (adxl345_read(cs_pin, POWER_CTL) == 0x08);
}

void read_accelerometer_data(uint8_t cs_pin, uint8_t sensor_index) {
    SPI_PORT &= ~(1 << cs_pin);
    _delay_us(5);
    
    spi_transfer(0x80 | 0x40 | DATAX0);
    
    uint8_t x0 = spi_transfer(0x00);
    uint8_t x1 = spi_transfer(0x00);
    uint8_t y0 = spi_transfer(0x00);
    uint8_t y1 = spi_transfer(0x00);
    uint8_t z0 = spi_transfer(0x00);
    uint8_t z1 = spi_transfer(0x00);
    
    sensor_data[sensor_index].x = (int16_t)((x1 << 8) | x0);
    sensor_data[sensor_index].y = (int16_t)((y1 << 8) | y0);
    sensor_data[sensor_index].z = (int16_t)((z1 << 8) | z0);
    
    _delay_us(5);
    SPI_PORT |= (1 << cs_pin);
    _delay_us(5);
}

void collect_all_sensor_data() {
    read_accelerometer_data(CS1_PIN, 0);
    read_accelerometer_data(CS2_PIN, 1);
    read_accelerometer_data(CS3_PIN, 2);
}

void send_data_package() {
    print_uint32(millis());
    for (uint8_t i = 0; i < 3; i++) {
        serial_print_char(';');
        print_int16(sensor_data[i].x);
        
        serial_print_char(';');
        print_int16(sensor_data[i].y);
        
        serial_print_char(';');
        print_int16(sensor_data[i].z);
    }
    
    serial_print_char('\n');
}

int main() {
    setup_serial();
    setup_timer();
    
    spi_init();
    
    // Initialize all sensors
    sensor_status[0] = adxl345_init(CS1_PIN, 0);
    _delay_ms(50);
    sensor_status[1] = adxl345_init(CS2_PIN, 1);
    _delay_ms(50);
    sensor_status[2] = adxl345_init(CS3_PIN, 2);
    _delay_ms(50);
    
    // Main loop
    while (1) {
        collect_all_sensor_data();
        send_data_package();
        _delay_ms(100);
    }
    
    return 0;
}