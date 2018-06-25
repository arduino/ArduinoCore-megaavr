#pragma once
/*
ARDUINO PIN	ATMEGA 328	ATMEGA 4809
0	        PD0	        PC5
1	        PD1	        PC4
2	        PD2	        PA0
3	        PD3	        PF5
4	        PD4	        PC6
5	        PD5	        PB2
6	        PD6	        PF4
7	        PD7	        PA1
8	        PB0     	PE3
9	        PB1     	PB0
10	        PB2     	PB1
11	        PB3     	PE0
12	        PB4     	PE1
13	        PB5     	PE2
A0	        PC0     	PD0
A1	        PC1     	PD1
A2	        PC2     	PD2
A3	        PC3     	PD3
A4	        PC4     	PD4
A5	        PC5     	PD5
*/

#define COMPATIBILITY_DEBUG true

#define PORTA_ARDUINO        (*(PORT_t *) 0x0400) /* I/O Ports */
#define PORTB_ARDUINO        (*(PORT_t *) 0x0420) /* I/O Ports */
#define PORTC_ARDUINO        (*(PORT_t *) 0x0440) /* I/O Ports */
#define PORTD_ARDUINO        (*(PORT_t *) 0x0460) /* I/O Ports */
#define PORTE_ARDUINO        (*(PORT_t *) 0x0480) /* I/O Ports */
#define PORTF_ARDUINO        (*(PORT_t *) 0x04A0) /* I/O Ports */

#define PORTD_OFFSET    0
#define PORTB_OFFSET    8
#define PORTC_OFFSET    16

#undef PORTA
#undef PORTB
#undef PORTC
#undef PORTD
#undef PORTE
#undef PORTF

#ifdef COMPATIBILITY_DEBUG
void printPortAndPin(int port, uint8_t pin, bool value) {
    Serial.print("Writing ");
    Serial.print(value);
    Serial.print(" on ");

    switch(port) {
        case (int) &PORTA_ARDUINO: Serial.print("PORTA"); break;
        case (int) &PORTB_ARDUINO: Serial.print("PORTB"); break;
        case (int) &PORTC_ARDUINO: Serial.print("PORTC"); break;
        case (int) &PORTD_ARDUINO: Serial.print("PORTD"); break;
        case (int) &PORTE_ARDUINO: Serial.print("PORTE"); break;
        case (int) &PORTF_ARDUINO: Serial.print("PORTF"); break;
    }
    
    Serial.println(pin);
}
#endif

typedef struct pinPort {
    volatile PORT_t*  port;
    uint8_t pin;
};

pinPort mapping[20] = {
    {&PORTC_ARDUINO, 5},
    {&PORTC_ARDUINO, 4},
    {&PORTA_ARDUINO, 0},
    {&PORTF_ARDUINO, 5},
    {&PORTC_ARDUINO, 6},
    {&PORTB_ARDUINO, 2},
    {&PORTF_ARDUINO, 4},
    {&PORTA_ARDUINO, 1},
    {&PORTE_ARDUINO, 3},
    {&PORTB_ARDUINO, 0},
    {&PORTB_ARDUINO, 1},
    {&PORTE_ARDUINO, 0},
    {&PORTE_ARDUINO, 1},
    {&PORTE_ARDUINO, 2},
    {&PORTD_ARDUINO, 0},
    {&PORTD_ARDUINO, 1},
    {&PORTD_ARDUINO, 2},
    {&PORTD_ARDUINO, 3},
    {&PORTD_ARDUINO, 4},
    {&PORTD_ARDUINO, 5},

};

/** DDR Classes**/
class DDRClass {
    public:
        DDRClass(uint8_t _offset):offset(_offset){}
        DDRClass& operator=(int value){
            for (int i=0; i<6; i++) {
                if (value & (1 << i)) {
                    mapping[i + offset].port->DIR |= ( 1 << mapping[i + offset].pin);
                } else {
                    mapping[i + offset].port->DIR &= ~( 1 << mapping[i + offset].pin);
                }
            }
            return *this; 
        }
    private:
        uint8_t offset;
};

DDRClass DDRB(PORTB_OFFSET);
DDRClass DDRC(PORTC_OFFSET);
DDRClass DDRD(PORTD_OFFSET);

/** PORT Classes**/
class PORTClass {
    public:
    PORTClass(uint8_t _offset):offset(_offset){}
    PORTClass& operator=(int value){
        for (int i=0; i<6; i++) {
            if (value & (1 << i)) {
#ifdef COMPATIBILITY_DEBUG
                printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, true);
#endif
                mapping[i + offset].port->OUT |= ( 1 << mapping[i + offset].pin);
            } else {
#ifdef COMPATIBILITY_DEBUG
                printPortAndPin((int) mapping[i + offset].port, mapping[i + offset].pin, false);
#endif
                mapping[i + offset].port->OUT &= ~( 1 << mapping[i + offset].pin);
            }
        }
        return *this; 
    }
    private:
        uint8_t offset;
};

PORTClass PORTB(PORTB_OFFSET);
PORTClass PORTC(PORTC_OFFSET);
PORTClass PORTD(PORTD_OFFSET);
