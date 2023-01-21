#include "sygnaly.h"

void Sygnaly_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Wlaczenie portu A
	
	PORTA->PCR[S1] |= PORT_PCR_MUX(1);
	PORTA->PCR[S2] |= PORT_PCR_MUX(1);
	PORTA->PCR[S3] |= PORT_PCR_MUX(1);

	PORTA->PCR[S1] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[S2] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[S3] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

}

