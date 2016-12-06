#include "m_wii.h"
#include "m_bus.h"
#include "m_usb.h"
#include "eep_locations.h"

#define BOT_ADDRESS 22

int main(void) {
	m_clockdivide(0);

	m_usb_init();

	eeprom_write_byte(&eepAddress, BOT_ADDRESS);

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