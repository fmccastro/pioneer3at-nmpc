/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>
int M(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int M_alloc_mem(void);
int M_init_mem(int mem);
void M_free_mem(int mem);
int M_checkout(void);
void M_release(int mem);
void M_incref(void);
void M_decref(void);
casadi_int M_n_out(void);
casadi_int M_n_in(void);
casadi_real M_default_in(casadi_int i);
const char* M_name_in(casadi_int i);
const char* M_name_out(casadi_int i);
const casadi_int* M_sparsity_in(casadi_int i);
const casadi_int* M_sparsity_out(casadi_int i);
int M_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
casadi_functions* M_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
