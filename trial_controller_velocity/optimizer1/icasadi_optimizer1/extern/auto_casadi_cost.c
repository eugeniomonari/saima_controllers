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
  #define CASADI_PREFIX(ID) phi_kPLOdHBInRkOYGECbkQK_ ## ID
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
#define casadi_fmax CASADI_PREFIX(fmax)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[54] = {50, 1, 0, 50, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49};

/* phi_kPLOdHBInRkOYGECbkQK:(i0[5],i1,i2[50])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=arg[2]? arg[2][45] : 0;
  a2=(a0<a1);
  a3=arg[2]? arg[2][46] : 0;
  a4=(a0-a1);
  a4=casadi_sq(a4);
  a3=(a3*a4);
  a3=(a2?a3:0);
  a2=(!a2);
  a4=arg[2]? arg[2][47] : 0;
  a1=(a0-a1);
  a1=casadi_sq(a1);
  a4=(a4*a1);
  a2=(a2?a4:0);
  a3=(a3+a2);
  a2=arg[2]? arg[2][48] : 0;
  a4=arg[0]? arg[0][1] : 0;
  a1=casadi_sq(a4);
  a2=(a2*a1);
  a3=(a3+a2);
  a2=arg[2]? arg[2][49] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a5=casadi_sq(a1);
  a6=arg[0]? arg[0][3] : 0;
  a7=casadi_sq(a6);
  a5=(a5+a7);
  a7=arg[0]? arg[0][4] : 0;
  a8=casadi_sq(a7);
  a5=(a5+a8);
  a2=(a2*a5);
  a3=(a3+a2);
  a2=arg[1]? arg[1][0] : 0;
  a5=0.;
  a8=arg[2]? arg[2][0] : 0;
  a9=arg[2]? arg[2][21] : 0;
  a10=(a9/a0);
  a10=(a10+a1);
  a10=(a8*a10);
  a11=arg[2]? arg[2][1] : 0;
  a12=arg[2]? arg[2][22] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a11*a13);
  a10=(a10+a13);
  a13=arg[2]? arg[2][2] : 0;
  a14=arg[2]? arg[2][23] : 0;
  a15=(a14/a0);
  a15=(a15+a7);
  a15=(a13*a15);
  a10=(a10+a15);
  a15=arg[2]? arg[2][24] : 0;
  a16=(a15*a4);
  a10=(a10+a16);
  a16=arg[2]? arg[2][31] : 0;
  a10=(a10-a16);
  a10=casadi_fmax(a5,a10);
  a10=casadi_sq(a10);
  a16=arg[2]? arg[2][38] : 0;
  a17=(a9/a0);
  a17=(a17+a1);
  a8=(a8*a17);
  a17=(a12/a0);
  a17=(a17+a6);
  a11=(a11*a17);
  a8=(a8+a11);
  a11=(a14/a0);
  a11=(a11+a7);
  a13=(a13*a11);
  a8=(a8+a13);
  a15=(a15*a4);
  a8=(a8+a15);
  a16=(a16-a8);
  a16=casadi_fmax(a5,a16);
  a16=casadi_sq(a16);
  a10=(a10+a16);
  a16=arg[2]? arg[2][3] : 0;
  a8=(a9/a0);
  a8=(a8+a1);
  a8=(a16*a8);
  a15=arg[2]? arg[2][4] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a15*a13);
  a8=(a8+a13);
  a13=arg[2]? arg[2][5] : 0;
  a11=(a14/a0);
  a11=(a11+a7);
  a11=(a13*a11);
  a8=(a8+a11);
  a11=arg[2]? arg[2][25] : 0;
  a17=(a11*a4);
  a8=(a8+a17);
  a17=arg[2]? arg[2][32] : 0;
  a8=(a8-a17);
  a8=casadi_fmax(a5,a8);
  a8=casadi_sq(a8);
  a10=(a10+a8);
  a8=arg[2]? arg[2][39] : 0;
  a17=(a9/a0);
  a17=(a17+a1);
  a16=(a16*a17);
  a17=(a12/a0);
  a17=(a17+a6);
  a15=(a15*a17);
  a16=(a16+a15);
  a15=(a14/a0);
  a15=(a15+a7);
  a13=(a13*a15);
  a16=(a16+a13);
  a11=(a11*a4);
  a16=(a16+a11);
  a8=(a8-a16);
  a8=casadi_fmax(a5,a8);
  a8=casadi_sq(a8);
  a10=(a10+a8);
  a8=arg[2]? arg[2][6] : 0;
  a16=(a9/a0);
  a16=(a16+a1);
  a16=(a8*a16);
  a11=arg[2]? arg[2][7] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a11*a13);
  a16=(a16+a13);
  a13=arg[2]? arg[2][8] : 0;
  a15=(a14/a0);
  a15=(a15+a7);
  a15=(a13*a15);
  a16=(a16+a15);
  a15=arg[2]? arg[2][26] : 0;
  a17=(a15*a4);
  a16=(a16+a17);
  a17=arg[2]? arg[2][33] : 0;
  a16=(a16-a17);
  a16=casadi_fmax(a5,a16);
  a16=casadi_sq(a16);
  a10=(a10+a16);
  a16=arg[2]? arg[2][40] : 0;
  a17=(a9/a0);
  a17=(a17+a1);
  a8=(a8*a17);
  a17=(a12/a0);
  a17=(a17+a6);
  a11=(a11*a17);
  a8=(a8+a11);
  a11=(a14/a0);
  a11=(a11+a7);
  a13=(a13*a11);
  a8=(a8+a13);
  a15=(a15*a4);
  a8=(a8+a15);
  a16=(a16-a8);
  a16=casadi_fmax(a5,a16);
  a16=casadi_sq(a16);
  a10=(a10+a16);
  a16=arg[2]? arg[2][9] : 0;
  a8=(a9/a0);
  a8=(a8+a1);
  a8=(a16*a8);
  a15=arg[2]? arg[2][10] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a15*a13);
  a8=(a8+a13);
  a13=arg[2]? arg[2][11] : 0;
  a11=(a14/a0);
  a11=(a11+a7);
  a11=(a13*a11);
  a8=(a8+a11);
  a11=arg[2]? arg[2][27] : 0;
  a17=(a11*a4);
  a8=(a8+a17);
  a17=arg[2]? arg[2][34] : 0;
  a8=(a8-a17);
  a8=casadi_fmax(a5,a8);
  a8=casadi_sq(a8);
  a10=(a10+a8);
  a8=arg[2]? arg[2][41] : 0;
  a17=(a9/a0);
  a17=(a17+a1);
  a16=(a16*a17);
  a17=(a12/a0);
  a17=(a17+a6);
  a15=(a15*a17);
  a16=(a16+a15);
  a15=(a14/a0);
  a15=(a15+a7);
  a13=(a13*a15);
  a16=(a16+a13);
  a11=(a11*a4);
  a16=(a16+a11);
  a8=(a8-a16);
  a8=casadi_fmax(a5,a8);
  a8=casadi_sq(a8);
  a10=(a10+a8);
  a8=arg[2]? arg[2][12] : 0;
  a16=(a9/a0);
  a16=(a16+a1);
  a16=(a8*a16);
  a11=arg[2]? arg[2][13] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a11*a13);
  a16=(a16+a13);
  a13=arg[2]? arg[2][14] : 0;
  a15=(a14/a0);
  a15=(a15+a7);
  a15=(a13*a15);
  a16=(a16+a15);
  a15=arg[2]? arg[2][28] : 0;
  a17=(a15*a4);
  a16=(a16+a17);
  a17=arg[2]? arg[2][35] : 0;
  a16=(a16-a17);
  a16=casadi_fmax(a5,a16);
  a16=casadi_sq(a16);
  a10=(a10+a16);
  a16=arg[2]? arg[2][42] : 0;
  a17=(a9/a0);
  a17=(a17+a1);
  a8=(a8*a17);
  a17=(a12/a0);
  a17=(a17+a6);
  a11=(a11*a17);
  a8=(a8+a11);
  a11=(a14/a0);
  a11=(a11+a7);
  a13=(a13*a11);
  a8=(a8+a13);
  a15=(a15*a4);
  a8=(a8+a15);
  a16=(a16-a8);
  a16=casadi_fmax(a5,a16);
  a16=casadi_sq(a16);
  a10=(a10+a16);
  a16=arg[2]? arg[2][15] : 0;
  a8=(a9/a0);
  a8=(a8+a1);
  a8=(a16*a8);
  a15=arg[2]? arg[2][16] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a15*a13);
  a8=(a8+a13);
  a13=arg[2]? arg[2][17] : 0;
  a11=(a14/a0);
  a11=(a11+a7);
  a11=(a13*a11);
  a8=(a8+a11);
  a11=arg[2]? arg[2][29] : 0;
  a17=(a11*a4);
  a8=(a8+a17);
  a17=arg[2]? arg[2][36] : 0;
  a8=(a8-a17);
  a8=casadi_fmax(a5,a8);
  a8=casadi_sq(a8);
  a10=(a10+a8);
  a8=arg[2]? arg[2][43] : 0;
  a17=(a9/a0);
  a17=(a17+a1);
  a16=(a16*a17);
  a17=(a12/a0);
  a17=(a17+a6);
  a15=(a15*a17);
  a16=(a16+a15);
  a15=(a14/a0);
  a15=(a15+a7);
  a13=(a13*a15);
  a16=(a16+a13);
  a11=(a11*a4);
  a16=(a16+a11);
  a8=(a8-a16);
  a8=casadi_fmax(a5,a8);
  a8=casadi_sq(a8);
  a10=(a10+a8);
  a8=arg[2]? arg[2][18] : 0;
  a16=(a9/a0);
  a16=(a16+a1);
  a16=(a8*a16);
  a11=arg[2]? arg[2][19] : 0;
  a13=(a12/a0);
  a13=(a13+a6);
  a13=(a11*a13);
  a16=(a16+a13);
  a13=arg[2]? arg[2][20] : 0;
  a15=(a14/a0);
  a15=(a15+a7);
  a15=(a13*a15);
  a16=(a16+a15);
  a15=arg[2]? arg[2][30] : 0;
  a17=(a15*a4);
  a16=(a16+a17);
  a17=arg[2]? arg[2][37] : 0;
  a16=(a16-a17);
  a16=casadi_fmax(a5,a16);
  a16=casadi_sq(a16);
  a10=(a10+a16);
  a16=arg[2]? arg[2][44] : 0;
  a9=(a9/a0);
  a9=(a9+a1);
  a8=(a8*a9);
  a12=(a12/a0);
  a12=(a12+a6);
  a11=(a11*a12);
  a8=(a8+a11);
  a14=(a14/a0);
  a14=(a14+a7);
  a13=(a13*a14);
  a8=(a8+a13);
  a15=(a15*a4);
  a8=(a8+a15);
  a16=(a16-a8);
  a5=casadi_fmax(a5,a16);
  a5=casadi_sq(a5);
  a10=(a10+a5);
  a2=(a2*a10);
  a10=2.;
  a2=(a2/a10);
  a3=(a3+a2);
  if (res[0]!=0) res[0][0]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int phi_kPLOdHBInRkOYGECbkQK(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int phi_kPLOdHBInRkOYGECbkQK_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int phi_kPLOdHBInRkOYGECbkQK_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void phi_kPLOdHBInRkOYGECbkQK_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int phi_kPLOdHBInRkOYGECbkQK_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void phi_kPLOdHBInRkOYGECbkQK_release(int mem) {
}

CASADI_SYMBOL_EXPORT void phi_kPLOdHBInRkOYGECbkQK_incref(void) {
}

CASADI_SYMBOL_EXPORT void phi_kPLOdHBInRkOYGECbkQK_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int phi_kPLOdHBInRkOYGECbkQK_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int phi_kPLOdHBInRkOYGECbkQK_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real phi_kPLOdHBInRkOYGECbkQK_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* phi_kPLOdHBInRkOYGECbkQK_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* phi_kPLOdHBInRkOYGECbkQK_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* phi_kPLOdHBInRkOYGECbkQK_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* phi_kPLOdHBInRkOYGECbkQK_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int phi_kPLOdHBInRkOYGECbkQK_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif