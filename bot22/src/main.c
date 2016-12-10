#include "m_wii.h"
#include "m_bus.h"
#include "m_usb.h"
#include "eep_locations.h"

#define NEG_Y -1
#define POS_Y 1

#define RED 0
#define BLUE 1

#define BOT_ADDRESS 22

int main(void) {
	m_clockdivide(0);

	m_usb_init();

	eeprom_write_byte(&eepAddress, BOT_ADDRESS);
	// eeprom_write_byte(&eepTeam, *TEAM_COLOR_HERE*);
	// eeprom_write_byte(&eepDirection, (uint8_t) *DIRECTION_HERE*);

	while(true) {
		m_usb_tx_uint(eeprom_read_byte(&eepAddress));
		m_usb_tx_string(" ");
		m_usb_tx_uint(eeprom_read_byte(&eepTeam));
		m_usb_tx_string(" ");
		m_usb_tx_int(eeprom_read_byte(&eepDirection));
		m_usb_tx_string(" ");
		m_usb_tx_int(eeprom_read_float(&eepCalX)*100);
		m_usb_tx_string(" ");
		m_usb_tx_int(eeprom_read_float(&eepCalY)*100);
		m_usb_tx_string("\n");
		m_wait(1000);
	}
}