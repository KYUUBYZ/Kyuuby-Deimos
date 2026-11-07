#ifndef BRINGUP_H
#define BRINGUP_H

/*
 * Kyuuby Deimos v1 - test di bring-up.
 *
 * bringup_init() : configura GPIO e SPI1/SPI2/SPI3 (registri diretti, senza HAL SPI).
 *                  Chiamarla dopo MX_USB_DEVICE_Init().
 * bringup_loop() : esegue i test e manda il report sulla porta seriale virtuale USB.
 *                  Chiamarla dentro il while(1).
 */

void bringup_init(void);
void bringup_loop(void);

#endif /* BRINGUP_H */
