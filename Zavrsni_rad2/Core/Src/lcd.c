


#include "lcd.h"
extern I2C_HandleTypeDef hi2c1;

#define SLAVE_ADRESA 0x4E

void lcd_slanje_naredbe (char naredba)
{
  char podatak_gornji, podatak_donji;
	uint8_t podatak_niz[4];
	podatak_gornji = (naredba&0xf0);
	podatak_donji = ((naredba<<4)&0xf0);
	podatak_niz[0] = podatak_gornji|0x0C;  //en=1, rs=0
	podatak_niz[1] = podatak_gornji|0x08;  //en=0, rs=0
	podatak_niz[2] = podatak_donji|0x0C;  //en=1, rs=0
	podatak_niz[3] = podatak_donji|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADRESA,(uint8_t *) podatak_niz, 4, 100);
}

void lcd_slanje_podatka (char podatak)
{
	char podatak_gornji, podatak_donji;
	uint8_t podatak_niz[4];
	podatak_gornji = (podatak&0xf0);
	podatak_donji = ((podatak<<4)&0xf0);
	podatak_niz[0] = podatak_gornji|0x0D;  //en=1, rs=1
	podatak_niz[1] = podatak_gornji|0x09;  //en=0, rs=1
	podatak_niz[2] = podatak_donji|0x0D;  //en=1, rs=1
	podatak_niz[3] = podatak_donji|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADRESA,(uint8_t *) podatak_niz, 4, 100);
}

void lcd_ocisti(void)
{
	lcd_slanje_naredbe(0x01);
	HAL_Delay(2);

}

void lcd_postavi_kursor(int redak, int stupac)
{
    switch (redak)
    {
        case 0:
            stupac |= 0x80;
            break;
        case 1:
            stupac |= 0xC0;
            break;
    }

    lcd_slanje_naredbe (stupac);
}


void lcd_inicijalizacija (void)
{
	//4 bitna inicijalizacija
	HAL_Delay(50);
	lcd_slanje_naredbe (0x30);
	HAL_Delay(5);
	lcd_slanje_naredbe (0x30);
	HAL_Delay(1);
	lcd_slanje_naredbe (0x30);
	HAL_Delay(10);
	lcd_slanje_naredbe (0x20);
	HAL_Delay(10);

	//inicijalizacija displaya
	lcd_slanje_naredbe (0x28); // Function set(0 0 0 0 1 DL N F) DL=0 (4 bitni mode), N = 1 (display u 2 linije) F = 0 (5x7)
	HAL_Delay(1);
	lcd_slanje_naredbe (0x08); //Display switch(OFF)(0 0 0 0 0 0 1 D C B) D=0,C=0, B=0- display off
	HAL_Delay(1);
	lcd_slanje_naredbe (0x01);  // Screen clear
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_slanje_naredbe (0x06); //Entry mode set(0 0 0 0 0 0 0 1 I/D S) I/D = 1 (increment mode),S = 0 (no shift)
	HAL_Delay(1);
	lcd_slanje_naredbe (0x0C); //Display switch (On)(0 0 0 0 0 0 1 D C B) D = 1, C=0, B = 0. D=entire display on,C=cursor on,B=cursor position on
}

void lcd_slanje_stringa (char *string)
{
	while (*string) lcd_slanje_podatka (*string++);
}
