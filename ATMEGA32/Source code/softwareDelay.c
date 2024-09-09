/*
 * softwareDelay.c
 *
 *  Created on: Dec 3, 2019
 *      Author: Knight
 */

#include "softwareDelay.h"

/**
 * Description: this delay function operate in a polling manner
 * 				don't use it with RTOSs
 * @param n: the milli-seconds
 */
void SwDelay_ms(uint32 n) {
	uint32 i;
	/*
	 for (i=0;i<n;i++){
	 for (j=0;j<n;j++){
	 for (k=0;k<n;k++){

	 }
	 }
	 }
	 */

	for (i = 0; i < n * 220; i++) {
		/*for (j = 0; j < (n * 1000); j++) {

		 }*/
	}
}


/**
 * Description: this delay function operate in a polling manner
 * 				don't use it with RTOSs
 * @param n: the micro-seconds
 */
void SwDelay_us(uint32 n){
	while(n--);
}

