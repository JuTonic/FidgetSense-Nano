#include <avr/io.h>
#include <avr/interrupt.h>

class SerialBuffer {
public:
    SerialBuffer();
    
    void clear();    
    void add_char(const char c);
    void add_uint32(const uint32_t num);
    void add_int16(const int16_t value);    
    void add_string(const char *str);    
    uint16_t size() const;    
    void send();
    
private:
    static const uint16_t BUFFER_SIZE = 128;
    char m_buffer[BUFFER_SIZE];
    uint16_t m_buffer_index;
};