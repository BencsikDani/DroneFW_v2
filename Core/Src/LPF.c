#include "LPF.h"

void LPF_Init(LPF *lpf)
{
	lpf->prevU = 0.0;
	lpf->prevY = 0.0;

	lpf->tau = 1.0 / lpf->f_cutoff;

	lpf->alpha = lpf->T / (2.0 * lpf->tau + lpf->T);
	lpf->beta = (2.0 * lpf->tau - lpf->T) / (2.0 * lpf->tau + lpf->T);
}

float LPF_Calculate(LPF *lpf, float U)
{
	float Y = lpf->alpha * (U + lpf->prevU) + lpf->beta * lpf->prevY;

	lpf->prevU = U;
	lpf->prevY = Y;

	return Y;
}
