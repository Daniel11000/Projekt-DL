#ifndef SYGS1_H
#define SYGS1_H

#include "MKL05Z4.h"
#define S1_MASK	(1<<6)		// Maska dla klawisza S1
#define S1	6							// Numer bitu dla klawisza S1


void SygnalS1_Init(void);
void SygS1Int(void);

#endif  /* SYGS1_H */