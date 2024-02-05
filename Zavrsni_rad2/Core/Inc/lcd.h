#include "stm32f4xx_hal.h"

void lcd_inicijalizacija (void);   // inicijalizacija

void lcd_slanje_naredbe (char naredba);  // slanje naredbe na lcd

void lcd_slanje_podatka (char podatak);  // slanje podatka na lcd

void lcd_slanje_stringa (char *string);  // slanje stringa na lcd

void lcd_postavi_kursor(int redak, int stupac);  // postavljanje kursora

void lcd_ocisti (void); //ciscenje
