/* OSU Robotics Club Rover 2016
 * Miniboard Firmware
 *
 * compass.c - Compass Module.
 * Author(s): Zachary Homen
 */
#include <stdint.h>
#include "compass.h"
#include "twi.h"

struct compass_packet {
	uint8_t addr; 		/* Address of the motor driver, from 128-135 */
	uint8_t cmd; 			/* Command byte, from 0-12 */
	uint8_t data; 		/* Data byte to accompany the command byte */
} __attribute__((__packed__));

uint8_t comp_addr = 0x42;
uint8_t eepromAddress = 00; // the address in the HMC6352 EEPROM from which to request the value of the I2C Slave Address
//uint8_t slaveAddress;
//uint8_t addressData[1];
//uint8_t addressValue;
uint8_t addr_data[6];

void setup(void)
{
	//  Shift the device's documented slave address (0x42) 1 bit right
	//  This compensates for how the TWI library only wants the
	//  7 most significant bits (with the high bit padded with 0)
	//slaveAddress = comp_addr >> 1;   // This results in 0x21 as the address to pass to TWI
	//Serial.begin(9600);
	TWI_Master_Initialise();
	config_rega(comp_addr);
	config_regb(comp_addr);
	mode(comp_addr);
	retrieve(comp_addr);
}

void retrieve(comp_addr){
	
	rdxa(comp_addr);
	rdxb(comp_addr);
	rdya(comp_addr);
	rdyb(comp_addr);
	rdza(comp_addr);
	rdzb(comp_addr);
}

void compass_wr(struct compass_packet *packet)
{
	TWI_Start_Transceiver_With_Data((uint8_t *) packet, 3);
}

uint8_t compass_rd(struct compass_packet *packet)
{
	return TWI_Get_Data_From_Transceiver((uint8_t *) packet, 3);
}

void config_rega(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 0;
	packet.data = 0x00;

	compass_wr(&packet);
}

void config_regb(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 1;
	packet.data = 0x00;

	compass_wr(&packet);
}

void mode(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 2;
	packet.data = 0x00;

	compass_wr(&packet);
}

void rdxa(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 3;
	packet.data = 0x00;

	addr_data[0] = compass_rd(&packet);
}

void rdxb(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 4;
	packet.data = 0x00;

	addr_data[1] = compass_rd(&packet);
}

void rdya(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 5;
	packet.data = 0x00;

	addr_data[2] = compass_rd(&packet);
}

void rdyb(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 6;
	packet.data = 0x00;

	addr_data[3] = compass_rd(&packet);
}

void rdza(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 7;
	packet.data = 0x00;

	addr_data[4] = compass_rd(&packet);
}

void rdzb(uint8_t comp_addr)
{
	struct compass_packet packet;

	packet.addr = comp_addr;
	packet.cmd = 8;
	packet.data = 0x00;

	addr_data[5] = compass_rd(&packet);
}