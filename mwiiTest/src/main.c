#include "m_wii.h"
#include "m_bus.h"
#include "m_usb.h"
#include "eep_test.h"

int main(void) {
	m_clockdivide(0);
	// unsigned int wiiBuffer[12] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	// m_wii_open();
	m_usb_init();

	// m_wii_read(wiiBuffer);
	// uint8_t i;
	// for (i = 0; i < 10; i++) {
	// 	eeprom_write_byte((uint8_t *)i,i);
	// }

	// eeprom_write_byte(&slot1, 0x10);
	// eeprom_write_word(&slot2, 0x3000);
	// eeprom_write_float(&slot3, 2.4);

	while(true) {
		// for (i = 0; i < 10; i++){
		// 	m_usb_tx_uint(eeprom_read_byte((uint8_t *) i));
		// 	m_usb_tx_string(" ");
		// }
		m_usb_tx_hexchar(eeprom_read_byte(&slot1));
		m_usb_tx_string(" ");
		m_usb_tx_hex(eeprom_read_word(&slot2));
		m_usb_tx_string(" ");
		m_usb_tx_uint(eeprom_read_float(&slot3)*100);
		// print_out();
		// m_red(TOGGLE);
		// m_usb_tx_uint(wiiBuffer[0]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[1]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[3]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[4]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[6]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[7]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[9]);
		// m_usb_tx_string(" ");
		// m_usb_tx_uint(wiiBuffer[10]);
		m_usb_tx_string("\n");
		m_wait(1000);
	}
}