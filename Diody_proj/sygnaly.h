#ifndef SYGNALY_H
#define SYGNALY_H

#include "MKL05Z4.h"
#define S1_MASK	(1<<12)		// Maska dla klawisza S1
#define S2_MASK	(1<<10)		// Maska dla klawisza S2
#define S3_MASK	(1<<11)		// Maska dla klawisza S3
#define S1	12							// Numer bitu dla klawisza S1
#define S2	10						// Numer bitu dla klawisza S2
#define S3	11						// Numer bitu dla klawisza S3

void Sygnaly_Init(void);

#endif  /* SYGNALY_H */