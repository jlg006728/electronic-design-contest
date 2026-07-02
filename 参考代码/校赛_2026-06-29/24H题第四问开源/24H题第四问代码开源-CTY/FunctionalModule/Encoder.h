#ifndef __Encoder_H
#define __Encoder_H

#include "headfile.h"

void EncodersInit(Motors_t* Motors);
void EncoderDataUpdate(Motors_t* Motors);
void EncoderLengthClear(Motors_t* Motors);
	
#endif
