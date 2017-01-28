/* OSU Robotics Club Rover 2016
 * Miniboard Firmware
 *
 * compass.h - HMC5883l external compass module.
 * Author(s): Zachary Homen
 */
#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>

/* This module will need to communicate with an HMC5883l compass over I2C
 * (built into https://hobbyking.com/en_us/ublox-neo-7m-gps-with-compass-and-pedestal-mount.html)
 * 3-axis magnetometer to determine the magnetic heading.
 * If nessecary, the current gravity vector can be supplied from the
 * IMU (if heading calculation depends on the tilt of the compass).
 * The compass will be on its own SPI bus. */

/* Get acceleration and magnetometer values from the IMU,
 * then use them to return the magnetic compass heading
 * of the rover. 
 * TODO: Units */


void setup(void);
//void compass_wr(struct compass_packet *packet);
void config_rega(uint8_t comp_addr);
void config_regb(uint8_t comp_addr);
void mode(uint8_t comp_addr);
void rdxa(uint8_t comp_addr);
void rdxb(uint8_t comp_addr);
void rdya(uint8_t comp_addr);
void rdyb(uint8_t comp_addr);
void rdza(uint8_t comp_addr);
void rdzb(uint8_t comp_addr);

#endif /* COMPASS_H */
//uint16_t compass_heading(void);