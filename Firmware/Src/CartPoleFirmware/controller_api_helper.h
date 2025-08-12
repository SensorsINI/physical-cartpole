#ifndef CONTROLLER_BIND_H
#define CONTROLLER_BIND_H

#include <stdint.h>
#include "controller_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Conservative, small stack footprint (can raise later if a spec needs it). */
#ifndef CB_MAX_INPUTS
#define CB_MAX_INPUTS   16
#endif
#ifndef CB_MAX_OUTPUTS
#define CB_MAX_OUTPUTS  4
#endif

/* Name→pointer entry supplied by the application (your variables live elsewhere). */
typedef struct {
    const char* name;   /* ASCII token exactly as in ControllerSpec->names[i] */
    float*      ptr;    /* address of the live variable carrying that signal */
} SignalEntry;

typedef struct {
    const ControllerOps* ops_last;            /* to rebind on change and run init/release */
    float*               in_ptrs[CB_MAX_INPUTS];
    uint8_t              n_inputs;
    float                zero;                /* default when a token is absent in SignalEntry[] */
} ControllerBinding;

/* One-time init; safe to call again. */
void CB_Init(ControllerBinding* cb);

/* (Re)bind when desired controller changes. Also calls release/init appropriately. */
void CB_RebindOnChange(ControllerBinding* cb,
                       const ControllerOps* desired_ops,
                       const SignalEntry* table, uint8_t table_len);

/* Evaluate the currently bound controller: dereference pointers → call evaluate(). */
float CB_Eval(ControllerBinding* cb);

#ifdef __cplusplus
}
#endif
#endif /* CONTROLLER_BIND_H */
