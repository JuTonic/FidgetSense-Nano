#include "serial_buffer.hpp"

SerialBuffer::SerialBuffer() : m_buffer_index(0) {}

void SerialBuffer::clear() {
    m_buffer_index = 0;
}

void SerialBuffer::add_char(const char c) {
    if (m_buffer_index < BUFFER_SIZE - 1) {
        m_buffer[m_buffer_index++] = c;
    }
}

void SerialBuffer::add_uint32(const uint32_t num) {
    if (num >= 10) {
        add_uint32(num / 10);
    }
    add_char('0' + (num % 10));
}

void SerialBuffer::add_int16(const int16_t value) {
    if (value < 0) {
        add_char('-');
        add_int16(-value);
        return;
    }
    add_uint32(static_cast<uint32_t>(value));
}

void SerialBuffer::add_string(const char *str) {
    while (*str) {
        add_char(*str++);
    }
}

uint16_t SerialBuffer::size() const {
    return m_buffer_index;
}

void SerialBuffer::send() {
    for (uint16_t i = 0; i < m_buffer_index; i++) {
        // Wait until the transmit buffer is empty
        while (!(UCSR0A & (1 << UDRE0))) {}
        UDR0 = m_buffer[i];
    }
    
    // Wait for the last character to be completely sent
    while (!(UCSR0A & (1 << TXC0))) {}
    
    clear();
}