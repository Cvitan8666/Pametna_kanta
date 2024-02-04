#include "stm32f4xx_hal.h"

void lcd_inicijalizacija (void);   // initialize lcd

void lcd_slanje_naredbe (char naredba);  // slanje naredbe na lcd

void lcd_slanje_podatka (char podatak);  // send data to the lcd

void lcd_slanje_stringa (char *string);  // send string to the lcd

void lcd_postavi_kursor(int redak, int stupac);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_ocisti (void);
