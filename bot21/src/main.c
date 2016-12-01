#include "m_wii.h"
#include "m_bus.h"
#include "m_usb.h"
#include "eep_locations.h"

#define BOT_ADDRESS 21

int main(void) {
	m_clockdivide(0);

	m_usb_init();

	eeprom_write_byte(&eepAddress, BOT_ADDRESS);

	while(true) {
		m_usb_tx_hexchar(eeprom_read_byte(&eepAddress));
		m_usb_tx_string(" ");
		m_usb_tx_string("\n");
		m_wait(1000);
	}
}