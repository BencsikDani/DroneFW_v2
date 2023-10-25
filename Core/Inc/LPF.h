#ifndef LPF_H
#define LPF_H

#include <stdint.h>

typedef struct {
	// Previous values
	float prevU, prevY;

	// Sampling time of the discrete LPF
	float T;
	// Cutoff frequency and tau time constant
	float f_cutoff, tau;
	// Coefficent values for the LPF
	float alpha, beta;

} LPF;

void LPF_Init(LPF *lpf);
float LPF_Update(LPF *lpf, float U);

#endif
