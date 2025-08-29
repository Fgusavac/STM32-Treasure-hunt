#ifndef GPIO_HEADER
#define GPIO_HEADER

#include <stdint.h>


struct _GPIO;

typedef struct _GPIO GPIO;



typedef enum {INPUT, OUTPUT, ANALOG} port_mode;

typedef enum {RISING_EDGE, FALLING_EDGE} trigger_type;
typedef enum { A, B, C, D, E } port_name_link;

GPIO *init_port(port_name_link name, port_mode mode, uint8_t pin_lower, uint8_t pin_upper);
uint16_t read_pins(GPIO *port_pt);
uint8_t read_single_pin(GPIO *port_pt, uint8_t pin_index);
void write_pins(GPIO *port_pt, uint16_t data);
void write_single_pin(GPIO *port_pt, uint8_t single_bit, uint8_t pin_index);
void enable_interrupt(GPIO *port_pt, uint8_t pin_index, trigger_type trigger, uint8_t priority, void (*interrupt_handler)(void));
void set_rate_limit_enabled(uint8_t enable);
uint8_t get_rate_limit_enabled(void);
void set_led_update_allowed(uint8_t allowed);

#endif
