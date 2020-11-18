/*
 * mode.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef MODE_H
#define MODE_H

/*** Charger mode ***/

//#define ATM 		// AT command mode.
#define NM 			// Nominal mode.

/*** Debug mode ***/

//#define DEBUG		// Use programming pins for debug purpose if defined.

/*** Error management ***/

#if (defined ATM && defined NM)
#error "Only 1 mode must be selected."
#endif

#endif /* MODE_H */
