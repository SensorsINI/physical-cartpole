#ifndef CONTROLLER_API_H
#define CONTROLLER_API_H

#include <stdint.h>

/* Transport-agnostic metadata the PC spec to build inputs in the correct order. */
typedef struct {
  uint8_t            version;     /* bump if semantics change */
  uint8_t            n_inputs;    /* number of float32 inputs */
  uint8_t            n_outputs;   /* number of float32 outputs */
  const char* const* names;       /* array of n_inputs ASCII tokens (wire order) */
} ControllerSpec;

/* Minimal, stable interface every controller implements. */
typedef struct {
  const ControllerSpec* (*spec)(void);               /* must return a stable pointer */
  void                   (*init)(void);                /* optional heavy init */
  void                   (*evaluate)(const float*, float*); /* core logic on floats */
  void                   (*release)(void);             /* optional cleanup */
} ControllerOps;

#endif /* CONTROLLER_API_H */
