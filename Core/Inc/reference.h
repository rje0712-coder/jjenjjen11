/*
 * reference.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Owner
 */

#ifndef INC_REFERENCE_H_
#define INC_REFERENCE_H_
typedef struct {
	float t_ref;
	float cnt_ref;
	float gain_ref;
	int taget_ref;
}ref_compare;

float generate_reference (ref_compare *ref);

#endif /* INC_REFERENCE_H_ */
