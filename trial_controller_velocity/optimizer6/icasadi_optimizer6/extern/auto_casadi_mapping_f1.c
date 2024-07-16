/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) mapping_f1_phGtujSYPnSEUHbdqHPZ_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[86] = {82, 1, 0, 82, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* mapping_f1_phGtujSYPnSEUHbdqHPZ:(i0[9],i1[82])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int mapping_f1_phGtujSYPnSEUHbdqHPZ(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int mapping_f1_phGtujSYPnSEUHbdqHPZ_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int mapping_f1_phGtujSYPnSEUHbdqHPZ_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void mapping_f1_phGtujSYPnSEUHbdqHPZ_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int mapping_f1_phGtujSYPnSEUHbdqHPZ_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void mapping_f1_phGtujSYPnSEUHbdqHPZ_release(int mem) {
}

CASADI_SYMBOL_EXPORT void mapping_f1_phGtujSYPnSEUHbdqHPZ_incref(void) {
}

CASADI_SYMBOL_EXPORT void mapping_f1_phGtujSYPnSEUHbdqHPZ_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int mapping_f1_phGtujSYPnSEUHbdqHPZ_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int mapping_f1_phGtujSYPnSEUHbdqHPZ_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real mapping_f1_phGtujSYPnSEUHbdqHPZ_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* mapping_f1_phGtujSYPnSEUHbdqHPZ_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* mapping_f1_phGtujSYPnSEUHbdqHPZ_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* mapping_f1_phGtujSYPnSEUHbdqHPZ_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* mapping_f1_phGtujSYPnSEUHbdqHPZ_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int mapping_f1_phGtujSYPnSEUHbdqHPZ_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
