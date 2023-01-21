/*-------------------------------------------------------------------------------------
					Technika Mikroprocesorowa 2 - laboratorium
					Lab 8 - Cwiczenie 3: sterowanie gestami
					autor: Mariusz Sokolowski
					wersja: 10.10.2022r.
----------------------------------------------------------------------------*/

#include "frdm_bsp.h" 
#include "lcd1602.h"
#include "leds.h" 
#include "stdio.h"
#include "math.h"
#include "i2c.h"
#include "sygnaly.h"

// ///

volatile uint8_t S1_press=0;	// "1" - sygnal wystepuje    "0" - sygnal nie wystepuje
volatile uint8_t S2_press=0;
volatile uint8_t S3_press=0;

// ///

void PORTA_IRQHandler(void)	// Podprogram obslugi przerwania od klawiszy S2, S3 i S4
{
	uint32_t buf;
	buf=PORTA->ISFR & (S1_MASK | S2_MASK | S3_MASK);
								

									if(S1_MASK)
									{
									//	DELAY(10);
										if(!(PTA->PDIR&S1_MASK))		// Minimalizacja drgan zestyków
										{
											if(!(PTA->PDIR&S1_MASK))	// Minimalizacja drgan zestyków (c.d.)
											{
												if(!S1_press)
												{
													S1_press=1;
												}
											}
										}
									}
									
									if(S2_MASK)
									{
									//	DELAY(10);
										if(!(PTA->PDIR&S2_MASK))		// Minimalizacja drgan zestyków
										{
											if(!(PTA->PDIR&S2_MASK))	// Minimalizacja drgan zestyków (c.d.)
											{
												if(!S2_press)
												{
													S2_press=1;
												}
											}
										}
									}	


									if(S3_MASK)
									{
									//	DELAY(10);
										if(!(PTA->PDIR&S3_MASK))		// Minimalizacja drgan zestyków
										{
											if(!(PTA->PDIR&S3_MASK))	// Minimalizacja drgan zestyków (c.d.)
											{
												if(!S3_press)
												{
													S3_press=1;
												}
											}
										}
									}
									
									
	PORTA->ISFR |=  S1_MASK | S2_MASK | S3_MASK;	// Kasowanie wszystkich bitów ISF
	NVIC_ClearPendingIRQ(PORTA_IRQn);
}


int main (void)
{ 
	char display[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
	
	Sygnaly_Init();								// Inicjalizacja Sygnalow
	LED_Init();
	LCD1602_Init();	// Tu jest równiez inicjalizacja portu I2C0
	LCD1602_Backlight(TRUE);

	
		// ----------------------------------------------------------------------------------------
	
	while(1)		// Poczatek petli glównej
	{
				PORTA_IRQHandler();
		
		if(S3_press == 1)											// - wlaczone sterowanie pilotem -
		{
			PTB->PCOR|=LED_B_MASK;	// Wlacz niebeiska diode LED
		}
		
		if(S1_press == 1 && S2_press == 1)				// - jazda do przodu -
		{
			if(S3_press == 0)
			{
				PTB->PSOR|=(LED_B_MASK); // Wylacz 	niebieska
			}
			
			PTB->PCOR|=LED_R_MASK;	// Wlacz czerwona diode LED
			PTB->PCOR|=LED_G_MASK;	// Wlacz zielona diode LED
		}
		else if(S1_press == 1 && S2_press == 0)				// - jazda w prawo -
		{
			if(S3_press == 0)
			{
				PTB->PSOR|=(LED_B_MASK); // Wylacz 	niebieska
			}
			
			PTB->PSOR|=(LED_G_MASK); // Wylacz zielona
			
			PTB->PCOR|=LED_R_MASK;	// Wlacz czerwona diode LED
		}
		else if(S2_press == 1 && S1_press == 0)				// - jazda w lewo -
		{
			if(S3_press == 0)
			{
				PTB->PSOR|=(LED_B_MASK); // Wylacz 	niebieska
			}
			
			PTB->PSOR|=(LED_R_MASK); // Wylacz czerwona
			
			PTB->PCOR|=LED_G_MASK;	// Wlacz zielona diode LED
		}
		else if(S1_press == 0 && S2_press == 0) 		// - postoj -
		{
			if(S3_press == 0)
			{
				PTB->PSOR|=(LED_B_MASK); // Wylacz 	niebieska
			}
			
			PTB->PSOR|=(LED_G_MASK); // Wylacz zielona
			PTB->PSOR|=(LED_R_MASK); // Wylacz czerwona
		}
		
				if(S3_press == 0)	 //  === wyswietlanie trybu pracy === 
		{
		//	LCD1602_ClearAll();	
			LCD1602_Print("Tryb pracy:");
			LCD1602_SetCursor(0,1);
			LCD1602_Print(" Line Follower  ");
		}
		else
		{
		//	LCD1602_ClearAll();	
			LCD1602_Print("Tryb pracy:");
			LCD1602_SetCursor(0,1);
			LCD1602_Print(" Pilot          ");
		}
		
			// Zerowanie zmiennych
		S1_press=0;
		S2_press=0;
		S3_press=0;
		
	}
	
	// ----------------------------------------------------------------------------------------
	
}
