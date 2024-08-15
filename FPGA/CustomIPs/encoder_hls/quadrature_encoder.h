#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <ap_int.h>

void quadrature_encoder(bool A, bool B, bool reset, volatile int *count);

#endif // QUADRATURE_ENCODER_H
