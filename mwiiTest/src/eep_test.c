#include "eep_test.h"

uint8_t EEMEM slot1;
uint16_t EEMEM slot2;
float EEMEM slot3;

void print_out(void) {
	m_usb_tx_hexchar(eeprom_read_byte(&slot1));
	m_usb_tx_string(" ");
	m_usb_tx_hex(eeprom_read_word(&slot2));
	m_usb_tx_string(" ");
	m_usb_tx_uint(eeprom_read_float(&slot3)*100);
}