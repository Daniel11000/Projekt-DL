/******************************************************************************
 * This file is a part of the SM2 Tutorial (C).                               *
 *                                                                            *
 * Adding extra defs makes life easy :)                                       *
 ******************************************************************************/

/**
 * @file frdm_bsp.h
 * @author Koryciak
 * @date Oct 2019
 * @brief File containing info about evaluation board used in project.
 * @ver 0.1
 */

#ifndef FRDM_BSP_H
#define FRDM_BSP_H

/******************************************************************************\
* Global definitions
\******************************************************************************/
#define FRDM_KL05Z       										 /* define eval used in project */

#ifndef TRUE
# define TRUE        (1)
#endif

#ifndef	FALSE
# define FALSE       (0)
#endif

/******************************************************************************\
* Global macros
\******************************************************************************/
#define MASK(x)		 (uint32_t)(1UL << (x))    /* turn bit number into 32b mask */
#define DELAY(x)   for(uint32_t i=0;i<(x*10000);i++)__nop(); 					/* wait */

#ifdef FRDM_KL05Z
# include "MKL05Z4.h"                        /* header with CMSIS */
#endif /* FRDM_KL05Z */

#endif /* FRDM_BSP_H */
