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
  #define CASADI_PREFIX(ID) grad_phi_ZcBOCJeGbhGljDTUIEQn_ ## ID
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

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[87] = {83, 1, 0, 83, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82};

/* grad_phi_ZcBOCJeGbhGljDTUIEQn:(i0[8],i1,i2[83])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[2]? arg[2][44] : 0;
  a1=arg[0]? arg[0][0] : 0;
  a2=(a0/a1);
  a3=(a2/a1);
  a4=1.0000000000000000e-03;
  a5=arg[2]? arg[2][38] : 0;
  a6=arg[2]? arg[2][74] : 0;
  a7=arg[2]? arg[2][36] : 0;
  a8=arg[2]? arg[2][48] : 0;
  a9=arg[2]? arg[2][42] : 0;
  a10=(a9/a1);
  a11=(a4*a10);
  a11=(a8+a11);
  a12=1.;
  a13=arg[2]? arg[2][77] : 0;
  a14=(a13/a1);
  a15=(a4*a14);
  a15=(a12+a15);
  a11=(a11/a15);
  a16=arg[0]? arg[0][2] : 0;
  a17=(a11+a16);
  a17=(a7*a17);
  a18=arg[2]? arg[2][37] : 0;
  a19=arg[2]? arg[2][49] : 0;
  a20=arg[2]? arg[2][43] : 0;
  a21=(a20/a1);
  a22=(a4*a21);
  a22=(a19+a22);
  a23=(a13/a1);
  a24=(a4*a23);
  a24=(a12+a24);
  a22=(a22/a24);
  a25=arg[0]? arg[0][3] : 0;
  a26=(a22+a25);
  a26=(a18*a26);
  a17=(a17+a26);
  a26=arg[2]? arg[2][50] : 0;
  a2=(a4*a2);
  a2=(a26+a2);
  a27=(a13/a1);
  a28=(a4*a27);
  a28=(a12+a28);
  a2=(a2/a28);
  a29=arg[0]? arg[0][4] : 0;
  a30=(a2+a29);
  a30=(a5*a30);
  a17=(a17+a30);
  a30=arg[2]? arg[2][39] : 0;
  a31=arg[2]? arg[2][51] : 0;
  a32=arg[2]? arg[2][45] : 0;
  a33=arg[2]? arg[2][76] : 0;
  a34=(a32/a33);
  a34=(a4*a34);
  a34=(a31+a34);
  a35=arg[2]? arg[2][78] : 0;
  a36=(a35/a33);
  a36=(a4*a36);
  a36=(a12+a36);
  a34=(a34/a36);
  a36=arg[0]? arg[0][5] : 0;
  a34=(a34+a36);
  a34=(a30*a34);
  a17=(a17+a34);
  a34=arg[2]? arg[2][40] : 0;
  a37=arg[2]? arg[2][52] : 0;
  a38=arg[2]? arg[2][46] : 0;
  a39=(a38/a33);
  a39=(a4*a39);
  a39=(a37+a39);
  a40=(a35/a33);
  a40=(a4*a40);
  a40=(a12+a40);
  a39=(a39/a40);
  a40=arg[0]? arg[0][6] : 0;
  a39=(a39+a40);
  a39=(a34*a39);
  a17=(a17+a39);
  a39=arg[2]? arg[2][41] : 0;
  a41=arg[2]? arg[2][53] : 0;
  a42=arg[2]? arg[2][47] : 0;
  a43=(a42/a33);
  a43=(a4*a43);
  a43=(a41+a43);
  a44=(a35/a33);
  a44=(a4*a44);
  a44=(a12+a44);
  a43=(a43/a44);
  a44=arg[0]? arg[0][7] : 0;
  a43=(a43+a44);
  a43=(a39*a43);
  a17=(a17+a43);
  a43=arg[2]? arg[2][60] : 0;
  a45=arg[0]? arg[0][1] : 0;
  a46=(a43*a45);
  a17=(a17+a46);
  a6=(a6-a17);
  a17=0.;
  a46=(a6<=a17);
  a46=(!a46);
  a6=casadi_fmax(a17,a6);
  a6=(a6+a6);
  a47=5.0000000000000000e-01;
  a48=arg[1]? arg[1][0] : 0;
  a47=(a47*a48);
  a6=(a6*a47);
  a46=(a46*a6);
  a6=(a5*a46);
  a48=(a6/a28);
  a48=(a4*a48);
  a3=(a3*a48);
  a27=(a27/a1);
  a2=(a2/a28);
  a2=(a2*a6);
  a2=(a4*a2);
  a27=(a27*a2);
  a3=(a3-a27);
  a23=(a23/a1);
  a22=(a22/a24);
  a27=(a18*a46);
  a22=(a22*a27);
  a22=(a4*a22);
  a23=(a23*a22);
  a3=(a3-a23);
  a21=(a21/a1);
  a24=(a27/a24);
  a24=(a4*a24);
  a21=(a21*a24);
  a3=(a3+a21);
  a14=(a14/a1);
  a11=(a11/a15);
  a21=(a7*a46);
  a11=(a11*a21);
  a11=(a4*a11);
  a14=(a14*a11);
  a3=(a3-a14);
  a10=(a10/a1);
  a15=(a21/a15);
  a15=(a4*a15);
  a10=(a10*a15);
  a3=(a3+a10);
  a10=(a13/a1);
  a15=(a10/a1);
  a14=(a0/a1);
  a11=(a4*a14);
  a11=(a26+a11);
  a10=(a4*a10);
  a10=(a12+a10);
  a11=(a11/a10);
  a24=(a11/a10);
  a23=(a9/a1);
  a22=(a4*a23);
  a22=(a8+a22);
  a2=(a13/a1);
  a28=(a4*a2);
  a28=(a12+a28);
  a22=(a22/a28);
  a48=(a22+a16);
  a48=(a7*a48);
  a49=(a20/a1);
  a50=(a4*a49);
  a50=(a19+a50);
  a51=(a13/a1);
  a52=(a4*a51);
  a52=(a12+a52);
  a50=(a50/a52);
  a53=(a50+a25);
  a53=(a18*a53);
  a48=(a48+a53);
  a11=(a11+a29);
  a11=(a5*a11);
  a48=(a48+a11);
  a11=(a32/a33);
  a11=(a4*a11);
  a11=(a31+a11);
  a53=(a35/a33);
  a53=(a4*a53);
  a53=(a12+a53);
  a11=(a11/a53);
  a11=(a11+a36);
  a11=(a30*a11);
  a48=(a48+a11);
  a11=(a38/a33);
  a11=(a4*a11);
  a11=(a37+a11);
  a53=(a35/a33);
  a53=(a4*a53);
  a53=(a12+a53);
  a11=(a11/a53);
  a11=(a11+a40);
  a11=(a34*a11);
  a48=(a48+a11);
  a11=(a42/a33);
  a11=(a4*a11);
  a11=(a41+a11);
  a53=(a35/a33);
  a53=(a4*a53);
  a53=(a12+a53);
  a11=(a11/a53);
  a11=(a11+a44);
  a11=(a39*a11);
  a48=(a48+a11);
  a11=(a43*a45);
  a48=(a48+a11);
  a11=arg[2]? arg[2][67] : 0;
  a48=(a48-a11);
  a11=(a48<=a17);
  a11=(!a11);
  a48=casadi_fmax(a17,a48);
  a48=(a48+a48);
  a48=(a48*a47);
  a11=(a11*a48);
  a5=(a5*a11);
  a24=(a24*a5);
  a24=(a4*a24);
  a15=(a15*a24);
  a3=(a3+a15);
  a14=(a14/a1);
  a10=(a5/a10);
  a10=(a4*a10);
  a14=(a14*a10);
  a3=(a3-a14);
  a51=(a51/a1);
  a50=(a50/a52);
  a18=(a18*a11);
  a50=(a50*a18);
  a50=(a4*a50);
  a51=(a51*a50);
  a3=(a3+a51);
  a49=(a49/a1);
  a52=(a18/a52);
  a52=(a4*a52);
  a49=(a49*a52);
  a3=(a3-a49);
  a2=(a2/a1);
  a22=(a22/a28);
  a7=(a7*a11);
  a22=(a22*a7);
  a22=(a4*a22);
  a2=(a2*a22);
  a3=(a3+a2);
  a23=(a23/a1);
  a28=(a7/a28);
  a28=(a4*a28);
  a23=(a23*a28);
  a3=(a3-a23);
  a23=(a13/a1);
  a28=(a23/a1);
  a2=(a0/a1);
  a22=(a4*a2);
  a22=(a26+a22);
  a23=(a4*a23);
  a23=(a12+a23);
  a22=(a22/a23);
  a49=(a22/a23);
  a52=arg[2]? arg[2][32] : 0;
  a51=arg[2]? arg[2][73] : 0;
  a50=arg[2]? arg[2][30] : 0;
  a14=(a9/a1);
  a10=(a4*a14);
  a10=(a8+a10);
  a15=(a13/a1);
  a24=(a4*a15);
  a24=(a12+a24);
  a10=(a10/a24);
  a48=(a10+a16);
  a48=(a50*a48);
  a53=arg[2]? arg[2][31] : 0;
  a54=(a20/a1);
  a55=(a4*a54);
  a55=(a19+a55);
  a56=(a13/a1);
  a57=(a4*a56);
  a57=(a12+a57);
  a55=(a55/a57);
  a58=(a55+a25);
  a58=(a53*a58);
  a48=(a48+a58);
  a22=(a22+a29);
  a22=(a52*a22);
  a48=(a48+a22);
  a22=arg[2]? arg[2][33] : 0;
  a58=(a32/a33);
  a58=(a4*a58);
  a58=(a31+a58);
  a59=(a35/a33);
  a59=(a4*a59);
  a59=(a12+a59);
  a58=(a58/a59);
  a58=(a58+a36);
  a58=(a22*a58);
  a48=(a48+a58);
  a58=arg[2]? arg[2][34] : 0;
  a59=(a38/a33);
  a59=(a4*a59);
  a59=(a37+a59);
  a60=(a35/a33);
  a60=(a4*a60);
  a60=(a12+a60);
  a59=(a59/a60);
  a59=(a59+a40);
  a59=(a58*a59);
  a48=(a48+a59);
  a59=arg[2]? arg[2][35] : 0;
  a60=(a42/a33);
  a60=(a4*a60);
  a60=(a41+a60);
  a61=(a35/a33);
  a61=(a4*a61);
  a61=(a12+a61);
  a60=(a60/a61);
  a60=(a60+a44);
  a60=(a59*a60);
  a48=(a48+a60);
  a60=arg[2]? arg[2][59] : 0;
  a61=(a60*a45);
  a48=(a48+a61);
  a51=(a51-a48);
  a48=(a51<=a17);
  a48=(!a48);
  a51=casadi_fmax(a17,a51);
  a51=(a51+a51);
  a51=(a51*a47);
  a48=(a48*a51);
  a51=(a52*a48);
  a49=(a49*a51);
  a49=(a4*a49);
  a28=(a28*a49);
  a3=(a3-a28);
  a2=(a2/a1);
  a23=(a51/a23);
  a23=(a4*a23);
  a2=(a2*a23);
  a3=(a3+a2);
  a56=(a56/a1);
  a55=(a55/a57);
  a2=(a53*a48);
  a55=(a55*a2);
  a55=(a4*a55);
  a56=(a56*a55);
  a3=(a3-a56);
  a54=(a54/a1);
  a57=(a2/a57);
  a57=(a4*a57);
  a54=(a54*a57);
  a3=(a3+a54);
  a15=(a15/a1);
  a10=(a10/a24);
  a54=(a50*a48);
  a10=(a10*a54);
  a10=(a4*a10);
  a15=(a15*a10);
  a3=(a3-a15);
  a14=(a14/a1);
  a24=(a54/a24);
  a24=(a4*a24);
  a14=(a14*a24);
  a3=(a3+a14);
  a14=(a13/a1);
  a24=(a14/a1);
  a15=(a0/a1);
  a10=(a4*a15);
  a10=(a26+a10);
  a14=(a4*a14);
  a14=(a12+a14);
  a10=(a10/a14);
  a57=(a10/a14);
  a56=(a9/a1);
  a55=(a4*a56);
  a55=(a8+a55);
  a23=(a13/a1);
  a28=(a4*a23);
  a28=(a12+a28);
  a55=(a55/a28);
  a49=(a55+a16);
  a49=(a50*a49);
  a61=(a20/a1);
  a62=(a4*a61);
  a62=(a19+a62);
  a63=(a13/a1);
  a64=(a4*a63);
  a64=(a12+a64);
  a62=(a62/a64);
  a65=(a62+a25);
  a65=(a53*a65);
  a49=(a49+a65);
  a10=(a10+a29);
  a10=(a52*a10);
  a49=(a49+a10);
  a10=(a32/a33);
  a10=(a4*a10);
  a10=(a31+a10);
  a65=(a35/a33);
  a65=(a4*a65);
  a65=(a12+a65);
  a10=(a10/a65);
  a10=(a10+a36);
  a10=(a22*a10);
  a49=(a49+a10);
  a10=(a38/a33);
  a10=(a4*a10);
  a10=(a37+a10);
  a65=(a35/a33);
  a65=(a4*a65);
  a65=(a12+a65);
  a10=(a10/a65);
  a10=(a10+a40);
  a10=(a58*a10);
  a49=(a49+a10);
  a10=(a42/a33);
  a10=(a4*a10);
  a10=(a41+a10);
  a65=(a35/a33);
  a65=(a4*a65);
  a65=(a12+a65);
  a10=(a10/a65);
  a10=(a10+a44);
  a10=(a59*a10);
  a49=(a49+a10);
  a10=(a60*a45);
  a49=(a49+a10);
  a10=arg[2]? arg[2][66] : 0;
  a49=(a49-a10);
  a10=(a49<=a17);
  a10=(!a10);
  a49=casadi_fmax(a17,a49);
  a49=(a49+a49);
  a49=(a49*a47);
  a10=(a10*a49);
  a52=(a52*a10);
  a57=(a57*a52);
  a57=(a4*a57);
  a24=(a24*a57);
  a3=(a3+a24);
  a15=(a15/a1);
  a14=(a52/a14);
  a14=(a4*a14);
  a15=(a15*a14);
  a3=(a3-a15);
  a63=(a63/a1);
  a62=(a62/a64);
  a53=(a53*a10);
  a62=(a62*a53);
  a62=(a4*a62);
  a63=(a63*a62);
  a3=(a3+a63);
  a61=(a61/a1);
  a64=(a53/a64);
  a64=(a4*a64);
  a61=(a61*a64);
  a3=(a3-a61);
  a23=(a23/a1);
  a55=(a55/a28);
  a50=(a50*a10);
  a55=(a55*a50);
  a55=(a4*a55);
  a23=(a23*a55);
  a3=(a3+a23);
  a56=(a56/a1);
  a28=(a50/a28);
  a28=(a4*a28);
  a56=(a56*a28);
  a3=(a3-a56);
  a56=(a13/a1);
  a28=(a56/a1);
  a23=(a0/a1);
  a55=(a4*a23);
  a55=(a26+a55);
  a56=(a4*a56);
  a56=(a12+a56);
  a55=(a55/a56);
  a61=(a55/a56);
  a64=arg[2]? arg[2][26] : 0;
  a63=arg[2]? arg[2][72] : 0;
  a62=arg[2]? arg[2][24] : 0;
  a15=(a9/a1);
  a14=(a4*a15);
  a14=(a8+a14);
  a24=(a13/a1);
  a57=(a4*a24);
  a57=(a12+a57);
  a14=(a14/a57);
  a49=(a14+a16);
  a49=(a62*a49);
  a65=arg[2]? arg[2][25] : 0;
  a66=(a20/a1);
  a67=(a4*a66);
  a67=(a19+a67);
  a68=(a13/a1);
  a69=(a4*a68);
  a69=(a12+a69);
  a67=(a67/a69);
  a70=(a67+a25);
  a70=(a65*a70);
  a49=(a49+a70);
  a55=(a55+a29);
  a55=(a64*a55);
  a49=(a49+a55);
  a55=arg[2]? arg[2][27] : 0;
  a70=(a32/a33);
  a70=(a4*a70);
  a70=(a31+a70);
  a71=(a35/a33);
  a71=(a4*a71);
  a71=(a12+a71);
  a70=(a70/a71);
  a70=(a70+a36);
  a70=(a55*a70);
  a49=(a49+a70);
  a70=arg[2]? arg[2][28] : 0;
  a71=(a38/a33);
  a71=(a4*a71);
  a71=(a37+a71);
  a72=(a35/a33);
  a72=(a4*a72);
  a72=(a12+a72);
  a71=(a71/a72);
  a71=(a71+a40);
  a71=(a70*a71);
  a49=(a49+a71);
  a71=arg[2]? arg[2][29] : 0;
  a72=(a42/a33);
  a72=(a4*a72);
  a72=(a41+a72);
  a73=(a35/a33);
  a73=(a4*a73);
  a73=(a12+a73);
  a72=(a72/a73);
  a72=(a72+a44);
  a72=(a71*a72);
  a49=(a49+a72);
  a72=arg[2]? arg[2][58] : 0;
  a73=(a72*a45);
  a49=(a49+a73);
  a63=(a63-a49);
  a49=(a63<=a17);
  a49=(!a49);
  a63=casadi_fmax(a17,a63);
  a63=(a63+a63);
  a63=(a63*a47);
  a49=(a49*a63);
  a63=(a64*a49);
  a61=(a61*a63);
  a61=(a4*a61);
  a28=(a28*a61);
  a3=(a3-a28);
  a23=(a23/a1);
  a56=(a63/a56);
  a56=(a4*a56);
  a23=(a23*a56);
  a3=(a3+a23);
  a68=(a68/a1);
  a67=(a67/a69);
  a23=(a65*a49);
  a67=(a67*a23);
  a67=(a4*a67);
  a68=(a68*a67);
  a3=(a3-a68);
  a66=(a66/a1);
  a69=(a23/a69);
  a69=(a4*a69);
  a66=(a66*a69);
  a3=(a3+a66);
  a24=(a24/a1);
  a14=(a14/a57);
  a66=(a62*a49);
  a14=(a14*a66);
  a14=(a4*a14);
  a24=(a24*a14);
  a3=(a3-a24);
  a15=(a15/a1);
  a57=(a66/a57);
  a57=(a4*a57);
  a15=(a15*a57);
  a3=(a3+a15);
  a15=(a13/a1);
  a57=(a15/a1);
  a24=(a0/a1);
  a14=(a4*a24);
  a14=(a26+a14);
  a15=(a4*a15);
  a15=(a12+a15);
  a14=(a14/a15);
  a69=(a14/a15);
  a68=(a9/a1);
  a67=(a4*a68);
  a67=(a8+a67);
  a56=(a13/a1);
  a28=(a4*a56);
  a28=(a12+a28);
  a67=(a67/a28);
  a61=(a67+a16);
  a61=(a62*a61);
  a73=(a20/a1);
  a74=(a4*a73);
  a74=(a19+a74);
  a75=(a13/a1);
  a76=(a4*a75);
  a76=(a12+a76);
  a74=(a74/a76);
  a77=(a74+a25);
  a77=(a65*a77);
  a61=(a61+a77);
  a14=(a14+a29);
  a14=(a64*a14);
  a61=(a61+a14);
  a14=(a32/a33);
  a14=(a4*a14);
  a14=(a31+a14);
  a77=(a35/a33);
  a77=(a4*a77);
  a77=(a12+a77);
  a14=(a14/a77);
  a14=(a14+a36);
  a14=(a55*a14);
  a61=(a61+a14);
  a14=(a38/a33);
  a14=(a4*a14);
  a14=(a37+a14);
  a77=(a35/a33);
  a77=(a4*a77);
  a77=(a12+a77);
  a14=(a14/a77);
  a14=(a14+a40);
  a14=(a70*a14);
  a61=(a61+a14);
  a14=(a42/a33);
  a14=(a4*a14);
  a14=(a41+a14);
  a77=(a35/a33);
  a77=(a4*a77);
  a77=(a12+a77);
  a14=(a14/a77);
  a14=(a14+a44);
  a14=(a71*a14);
  a61=(a61+a14);
  a14=(a72*a45);
  a61=(a61+a14);
  a14=arg[2]? arg[2][65] : 0;
  a61=(a61-a14);
  a14=(a61<=a17);
  a14=(!a14);
  a61=casadi_fmax(a17,a61);
  a61=(a61+a61);
  a61=(a61*a47);
  a14=(a14*a61);
  a64=(a64*a14);
  a69=(a69*a64);
  a69=(a4*a69);
  a57=(a57*a69);
  a3=(a3+a57);
  a24=(a24/a1);
  a15=(a64/a15);
  a15=(a4*a15);
  a24=(a24*a15);
  a3=(a3-a24);
  a75=(a75/a1);
  a74=(a74/a76);
  a65=(a65*a14);
  a74=(a74*a65);
  a74=(a4*a74);
  a75=(a75*a74);
  a3=(a3+a75);
  a73=(a73/a1);
  a76=(a65/a76);
  a76=(a4*a76);
  a73=(a73*a76);
  a3=(a3-a73);
  a56=(a56/a1);
  a67=(a67/a28);
  a62=(a62*a14);
  a67=(a67*a62);
  a67=(a4*a67);
  a56=(a56*a67);
  a3=(a3+a56);
  a68=(a68/a1);
  a28=(a62/a28);
  a28=(a4*a28);
  a68=(a68*a28);
  a3=(a3-a68);
  a68=(a13/a1);
  a28=(a68/a1);
  a56=(a0/a1);
  a67=(a4*a56);
  a67=(a26+a67);
  a68=(a4*a68);
  a68=(a12+a68);
  a67=(a67/a68);
  a73=(a67/a68);
  a76=arg[2]? arg[2][20] : 0;
  a75=arg[2]? arg[2][71] : 0;
  a74=arg[2]? arg[2][18] : 0;
  a24=(a9/a1);
  a15=(a4*a24);
  a15=(a8+a15);
  a57=(a13/a1);
  a69=(a4*a57);
  a69=(a12+a69);
  a15=(a15/a69);
  a61=(a15+a16);
  a61=(a74*a61);
  a77=arg[2]? arg[2][19] : 0;
  a78=(a20/a1);
  a79=(a4*a78);
  a79=(a19+a79);
  a80=(a13/a1);
  a81=(a4*a80);
  a81=(a12+a81);
  a79=(a79/a81);
  a82=(a79+a25);
  a82=(a77*a82);
  a61=(a61+a82);
  a67=(a67+a29);
  a67=(a76*a67);
  a61=(a61+a67);
  a67=arg[2]? arg[2][21] : 0;
  a82=(a32/a33);
  a82=(a4*a82);
  a82=(a31+a82);
  a83=(a35/a33);
  a83=(a4*a83);
  a83=(a12+a83);
  a82=(a82/a83);
  a82=(a82+a36);
  a82=(a67*a82);
  a61=(a61+a82);
  a82=arg[2]? arg[2][22] : 0;
  a83=(a38/a33);
  a83=(a4*a83);
  a83=(a37+a83);
  a84=(a35/a33);
  a84=(a4*a84);
  a84=(a12+a84);
  a83=(a83/a84);
  a83=(a83+a40);
  a83=(a82*a83);
  a61=(a61+a83);
  a83=arg[2]? arg[2][23] : 0;
  a84=(a42/a33);
  a84=(a4*a84);
  a84=(a41+a84);
  a85=(a35/a33);
  a85=(a4*a85);
  a85=(a12+a85);
  a84=(a84/a85);
  a84=(a84+a44);
  a84=(a83*a84);
  a61=(a61+a84);
  a84=arg[2]? arg[2][57] : 0;
  a85=(a84*a45);
  a61=(a61+a85);
  a75=(a75-a61);
  a61=(a75<=a17);
  a61=(!a61);
  a75=casadi_fmax(a17,a75);
  a75=(a75+a75);
  a75=(a75*a47);
  a61=(a61*a75);
  a75=(a76*a61);
  a73=(a73*a75);
  a73=(a4*a73);
  a28=(a28*a73);
  a3=(a3-a28);
  a56=(a56/a1);
  a68=(a75/a68);
  a68=(a4*a68);
  a56=(a56*a68);
  a3=(a3+a56);
  a80=(a80/a1);
  a79=(a79/a81);
  a56=(a77*a61);
  a79=(a79*a56);
  a79=(a4*a79);
  a80=(a80*a79);
  a3=(a3-a80);
  a78=(a78/a1);
  a81=(a56/a81);
  a81=(a4*a81);
  a78=(a78*a81);
  a3=(a3+a78);
  a57=(a57/a1);
  a15=(a15/a69);
  a78=(a74*a61);
  a15=(a15*a78);
  a15=(a4*a15);
  a57=(a57*a15);
  a3=(a3-a57);
  a24=(a24/a1);
  a69=(a78/a69);
  a69=(a4*a69);
  a24=(a24*a69);
  a3=(a3+a24);
  a24=(a13/a1);
  a69=(a24/a1);
  a57=(a0/a1);
  a15=(a4*a57);
  a15=(a26+a15);
  a24=(a4*a24);
  a24=(a12+a24);
  a15=(a15/a24);
  a81=(a15/a24);
  a80=(a9/a1);
  a79=(a4*a80);
  a79=(a8+a79);
  a68=(a13/a1);
  a28=(a4*a68);
  a28=(a12+a28);
  a79=(a79/a28);
  a73=(a79+a16);
  a73=(a74*a73);
  a85=(a20/a1);
  a86=(a4*a85);
  a86=(a19+a86);
  a87=(a13/a1);
  a88=(a4*a87);
  a88=(a12+a88);
  a86=(a86/a88);
  a89=(a86+a25);
  a89=(a77*a89);
  a73=(a73+a89);
  a15=(a15+a29);
  a15=(a76*a15);
  a73=(a73+a15);
  a15=(a32/a33);
  a15=(a4*a15);
  a15=(a31+a15);
  a89=(a35/a33);
  a89=(a4*a89);
  a89=(a12+a89);
  a15=(a15/a89);
  a15=(a15+a36);
  a15=(a67*a15);
  a73=(a73+a15);
  a15=(a38/a33);
  a15=(a4*a15);
  a15=(a37+a15);
  a89=(a35/a33);
  a89=(a4*a89);
  a89=(a12+a89);
  a15=(a15/a89);
  a15=(a15+a40);
  a15=(a82*a15);
  a73=(a73+a15);
  a15=(a42/a33);
  a15=(a4*a15);
  a15=(a41+a15);
  a89=(a35/a33);
  a89=(a4*a89);
  a89=(a12+a89);
  a15=(a15/a89);
  a15=(a15+a44);
  a15=(a83*a15);
  a73=(a73+a15);
  a15=(a84*a45);
  a73=(a73+a15);
  a15=arg[2]? arg[2][64] : 0;
  a73=(a73-a15);
  a15=(a73<=a17);
  a15=(!a15);
  a73=casadi_fmax(a17,a73);
  a73=(a73+a73);
  a73=(a73*a47);
  a15=(a15*a73);
  a76=(a76*a15);
  a81=(a81*a76);
  a81=(a4*a81);
  a69=(a69*a81);
  a3=(a3+a69);
  a57=(a57/a1);
  a24=(a76/a24);
  a24=(a4*a24);
  a57=(a57*a24);
  a3=(a3-a57);
  a87=(a87/a1);
  a86=(a86/a88);
  a77=(a77*a15);
  a86=(a86*a77);
  a86=(a4*a86);
  a87=(a87*a86);
  a3=(a3+a87);
  a85=(a85/a1);
  a88=(a77/a88);
  a88=(a4*a88);
  a85=(a85*a88);
  a3=(a3-a85);
  a68=(a68/a1);
  a79=(a79/a28);
  a74=(a74*a15);
  a79=(a79*a74);
  a79=(a4*a79);
  a68=(a68*a79);
  a3=(a3+a68);
  a80=(a80/a1);
  a28=(a74/a28);
  a28=(a4*a28);
  a80=(a80*a28);
  a3=(a3-a80);
  a80=(a13/a1);
  a28=(a80/a1);
  a68=(a0/a1);
  a79=(a4*a68);
  a79=(a26+a79);
  a80=(a4*a80);
  a80=(a12+a80);
  a79=(a79/a80);
  a85=(a79/a80);
  a88=arg[2]? arg[2][14] : 0;
  a87=arg[2]? arg[2][70] : 0;
  a86=arg[2]? arg[2][12] : 0;
  a57=(a9/a1);
  a24=(a4*a57);
  a24=(a8+a24);
  a69=(a13/a1);
  a81=(a4*a69);
  a81=(a12+a81);
  a24=(a24/a81);
  a73=(a24+a16);
  a73=(a86*a73);
  a89=arg[2]? arg[2][13] : 0;
  a90=(a20/a1);
  a91=(a4*a90);
  a91=(a19+a91);
  a92=(a13/a1);
  a93=(a4*a92);
  a93=(a12+a93);
  a91=(a91/a93);
  a94=(a91+a25);
  a94=(a89*a94);
  a73=(a73+a94);
  a79=(a79+a29);
  a79=(a88*a79);
  a73=(a73+a79);
  a79=arg[2]? arg[2][15] : 0;
  a94=(a32/a33);
  a94=(a4*a94);
  a94=(a31+a94);
  a95=(a35/a33);
  a95=(a4*a95);
  a95=(a12+a95);
  a94=(a94/a95);
  a94=(a94+a36);
  a94=(a79*a94);
  a73=(a73+a94);
  a94=arg[2]? arg[2][16] : 0;
  a95=(a38/a33);
  a95=(a4*a95);
  a95=(a37+a95);
  a96=(a35/a33);
  a96=(a4*a96);
  a96=(a12+a96);
  a95=(a95/a96);
  a95=(a95+a40);
  a95=(a94*a95);
  a73=(a73+a95);
  a95=arg[2]? arg[2][17] : 0;
  a96=(a42/a33);
  a96=(a4*a96);
  a96=(a41+a96);
  a97=(a35/a33);
  a97=(a4*a97);
  a97=(a12+a97);
  a96=(a96/a97);
  a96=(a96+a44);
  a96=(a95*a96);
  a73=(a73+a96);
  a96=arg[2]? arg[2][56] : 0;
  a97=(a96*a45);
  a73=(a73+a97);
  a87=(a87-a73);
  a73=(a87<=a17);
  a73=(!a73);
  a87=casadi_fmax(a17,a87);
  a87=(a87+a87);
  a87=(a87*a47);
  a73=(a73*a87);
  a87=(a88*a73);
  a85=(a85*a87);
  a85=(a4*a85);
  a28=(a28*a85);
  a3=(a3-a28);
  a68=(a68/a1);
  a80=(a87/a80);
  a80=(a4*a80);
  a68=(a68*a80);
  a3=(a3+a68);
  a92=(a92/a1);
  a91=(a91/a93);
  a68=(a89*a73);
  a91=(a91*a68);
  a91=(a4*a91);
  a92=(a92*a91);
  a3=(a3-a92);
  a90=(a90/a1);
  a93=(a68/a93);
  a93=(a4*a93);
  a90=(a90*a93);
  a3=(a3+a90);
  a69=(a69/a1);
  a24=(a24/a81);
  a90=(a86*a73);
  a24=(a24*a90);
  a24=(a4*a24);
  a69=(a69*a24);
  a3=(a3-a69);
  a57=(a57/a1);
  a81=(a90/a81);
  a81=(a4*a81);
  a57=(a57*a81);
  a3=(a3+a57);
  a57=(a13/a1);
  a81=(a57/a1);
  a69=(a0/a1);
  a24=(a4*a69);
  a24=(a26+a24);
  a57=(a4*a57);
  a57=(a12+a57);
  a24=(a24/a57);
  a93=(a24/a57);
  a92=(a9/a1);
  a91=(a4*a92);
  a91=(a8+a91);
  a80=(a13/a1);
  a28=(a4*a80);
  a28=(a12+a28);
  a91=(a91/a28);
  a85=(a91+a16);
  a85=(a86*a85);
  a97=(a20/a1);
  a98=(a4*a97);
  a98=(a19+a98);
  a99=(a13/a1);
  a100=(a4*a99);
  a100=(a12+a100);
  a98=(a98/a100);
  a101=(a98+a25);
  a101=(a89*a101);
  a85=(a85+a101);
  a24=(a24+a29);
  a24=(a88*a24);
  a85=(a85+a24);
  a24=(a32/a33);
  a24=(a4*a24);
  a24=(a31+a24);
  a101=(a35/a33);
  a101=(a4*a101);
  a101=(a12+a101);
  a24=(a24/a101);
  a24=(a24+a36);
  a24=(a79*a24);
  a85=(a85+a24);
  a24=(a38/a33);
  a24=(a4*a24);
  a24=(a37+a24);
  a101=(a35/a33);
  a101=(a4*a101);
  a101=(a12+a101);
  a24=(a24/a101);
  a24=(a24+a40);
  a24=(a94*a24);
  a85=(a85+a24);
  a24=(a42/a33);
  a24=(a4*a24);
  a24=(a41+a24);
  a101=(a35/a33);
  a101=(a4*a101);
  a101=(a12+a101);
  a24=(a24/a101);
  a24=(a24+a44);
  a24=(a95*a24);
  a85=(a85+a24);
  a24=(a96*a45);
  a85=(a85+a24);
  a24=arg[2]? arg[2][63] : 0;
  a85=(a85-a24);
  a24=(a85<=a17);
  a24=(!a24);
  a85=casadi_fmax(a17,a85);
  a85=(a85+a85);
  a85=(a85*a47);
  a24=(a24*a85);
  a88=(a88*a24);
  a93=(a93*a88);
  a93=(a4*a93);
  a81=(a81*a93);
  a3=(a3+a81);
  a69=(a69/a1);
  a57=(a88/a57);
  a57=(a4*a57);
  a69=(a69*a57);
  a3=(a3-a69);
  a99=(a99/a1);
  a98=(a98/a100);
  a89=(a89*a24);
  a98=(a98*a89);
  a98=(a4*a98);
  a99=(a99*a98);
  a3=(a3+a99);
  a97=(a97/a1);
  a100=(a89/a100);
  a100=(a4*a100);
  a97=(a97*a100);
  a3=(a3-a97);
  a80=(a80/a1);
  a91=(a91/a28);
  a86=(a86*a24);
  a91=(a91*a86);
  a91=(a4*a91);
  a80=(a80*a91);
  a3=(a3+a80);
  a92=(a92/a1);
  a28=(a86/a28);
  a28=(a4*a28);
  a92=(a92*a28);
  a3=(a3-a92);
  a92=(a13/a1);
  a28=(a92/a1);
  a80=(a0/a1);
  a91=(a4*a80);
  a91=(a26+a91);
  a92=(a4*a92);
  a92=(a12+a92);
  a91=(a91/a92);
  a97=(a91/a92);
  a100=arg[2]? arg[2][8] : 0;
  a99=arg[2]? arg[2][69] : 0;
  a98=arg[2]? arg[2][6] : 0;
  a69=(a9/a1);
  a57=(a4*a69);
  a57=(a8+a57);
  a81=(a13/a1);
  a93=(a4*a81);
  a93=(a12+a93);
  a57=(a57/a93);
  a85=(a57+a16);
  a85=(a98*a85);
  a101=arg[2]? arg[2][7] : 0;
  a102=(a20/a1);
  a103=(a4*a102);
  a103=(a19+a103);
  a104=(a13/a1);
  a105=(a4*a104);
  a105=(a12+a105);
  a103=(a103/a105);
  a106=(a103+a25);
  a106=(a101*a106);
  a85=(a85+a106);
  a91=(a91+a29);
  a91=(a100*a91);
  a85=(a85+a91);
  a91=arg[2]? arg[2][9] : 0;
  a106=(a32/a33);
  a106=(a4*a106);
  a106=(a31+a106);
  a107=(a35/a33);
  a107=(a4*a107);
  a107=(a12+a107);
  a106=(a106/a107);
  a106=(a106+a36);
  a106=(a91*a106);
  a85=(a85+a106);
  a106=arg[2]? arg[2][10] : 0;
  a107=(a38/a33);
  a107=(a4*a107);
  a107=(a37+a107);
  a108=(a35/a33);
  a108=(a4*a108);
  a108=(a12+a108);
  a107=(a107/a108);
  a107=(a107+a40);
  a107=(a106*a107);
  a85=(a85+a107);
  a107=arg[2]? arg[2][11] : 0;
  a108=(a42/a33);
  a108=(a4*a108);
  a108=(a41+a108);
  a109=(a35/a33);
  a109=(a4*a109);
  a109=(a12+a109);
  a108=(a108/a109);
  a108=(a108+a44);
  a108=(a107*a108);
  a85=(a85+a108);
  a108=arg[2]? arg[2][55] : 0;
  a109=(a108*a45);
  a85=(a85+a109);
  a99=(a99-a85);
  a85=(a99<=a17);
  a85=(!a85);
  a99=casadi_fmax(a17,a99);
  a99=(a99+a99);
  a99=(a99*a47);
  a85=(a85*a99);
  a99=(a100*a85);
  a97=(a97*a99);
  a97=(a4*a97);
  a28=(a28*a97);
  a3=(a3-a28);
  a80=(a80/a1);
  a92=(a99/a92);
  a92=(a4*a92);
  a80=(a80*a92);
  a3=(a3+a80);
  a104=(a104/a1);
  a103=(a103/a105);
  a80=(a101*a85);
  a103=(a103*a80);
  a103=(a4*a103);
  a104=(a104*a103);
  a3=(a3-a104);
  a102=(a102/a1);
  a105=(a80/a105);
  a105=(a4*a105);
  a102=(a102*a105);
  a3=(a3+a102);
  a81=(a81/a1);
  a57=(a57/a93);
  a102=(a98*a85);
  a57=(a57*a102);
  a57=(a4*a57);
  a81=(a81*a57);
  a3=(a3-a81);
  a69=(a69/a1);
  a93=(a102/a93);
  a93=(a4*a93);
  a69=(a69*a93);
  a3=(a3+a69);
  a69=(a13/a1);
  a93=(a69/a1);
  a81=(a0/a1);
  a57=(a4*a81);
  a57=(a26+a57);
  a69=(a4*a69);
  a69=(a12+a69);
  a57=(a57/a69);
  a105=(a57/a69);
  a104=(a9/a1);
  a103=(a4*a104);
  a103=(a8+a103);
  a92=(a13/a1);
  a28=(a4*a92);
  a28=(a12+a28);
  a103=(a103/a28);
  a97=(a103+a16);
  a97=(a98*a97);
  a109=(a20/a1);
  a110=(a4*a109);
  a110=(a19+a110);
  a111=(a13/a1);
  a112=(a4*a111);
  a112=(a12+a112);
  a110=(a110/a112);
  a113=(a110+a25);
  a113=(a101*a113);
  a97=(a97+a113);
  a57=(a57+a29);
  a57=(a100*a57);
  a97=(a97+a57);
  a57=(a32/a33);
  a57=(a4*a57);
  a57=(a31+a57);
  a113=(a35/a33);
  a113=(a4*a113);
  a113=(a12+a113);
  a57=(a57/a113);
  a57=(a57+a36);
  a57=(a91*a57);
  a97=(a97+a57);
  a57=(a38/a33);
  a57=(a4*a57);
  a57=(a37+a57);
  a113=(a35/a33);
  a113=(a4*a113);
  a113=(a12+a113);
  a57=(a57/a113);
  a57=(a57+a40);
  a57=(a106*a57);
  a97=(a97+a57);
  a57=(a42/a33);
  a57=(a4*a57);
  a57=(a41+a57);
  a113=(a35/a33);
  a113=(a4*a113);
  a113=(a12+a113);
  a57=(a57/a113);
  a57=(a57+a44);
  a57=(a107*a57);
  a97=(a97+a57);
  a57=(a108*a45);
  a97=(a97+a57);
  a57=arg[2]? arg[2][62] : 0;
  a97=(a97-a57);
  a57=(a97<=a17);
  a57=(!a57);
  a97=casadi_fmax(a17,a97);
  a97=(a97+a97);
  a97=(a97*a47);
  a57=(a57*a97);
  a100=(a100*a57);
  a105=(a105*a100);
  a105=(a4*a105);
  a93=(a93*a105);
  a3=(a3+a93);
  a81=(a81/a1);
  a69=(a100/a69);
  a69=(a4*a69);
  a81=(a81*a69);
  a3=(a3-a81);
  a111=(a111/a1);
  a110=(a110/a112);
  a101=(a101*a57);
  a110=(a110*a101);
  a110=(a4*a110);
  a111=(a111*a110);
  a3=(a3+a111);
  a109=(a109/a1);
  a112=(a101/a112);
  a112=(a4*a112);
  a109=(a109*a112);
  a3=(a3-a109);
  a92=(a92/a1);
  a103=(a103/a28);
  a98=(a98*a57);
  a103=(a103*a98);
  a103=(a4*a103);
  a92=(a92*a103);
  a3=(a3+a92);
  a104=(a104/a1);
  a28=(a98/a28);
  a28=(a4*a28);
  a104=(a104*a28);
  a3=(a3-a104);
  a104=(a13/a1);
  a28=(a104/a1);
  a92=(a0/a1);
  a103=(a4*a92);
  a103=(a26+a103);
  a104=(a4*a104);
  a104=(a12+a104);
  a103=(a103/a104);
  a109=(a103/a104);
  a112=arg[2]? arg[2][2] : 0;
  a111=arg[2]? arg[2][68] : 0;
  a110=arg[2]? arg[2][0] : 0;
  a81=(a9/a1);
  a69=(a4*a81);
  a69=(a8+a69);
  a93=(a13/a1);
  a105=(a4*a93);
  a105=(a12+a105);
  a69=(a69/a105);
  a97=(a69+a16);
  a97=(a110*a97);
  a113=arg[2]? arg[2][1] : 0;
  a114=(a20/a1);
  a115=(a4*a114);
  a115=(a19+a115);
  a116=(a13/a1);
  a117=(a4*a116);
  a117=(a12+a117);
  a115=(a115/a117);
  a118=(a115+a25);
  a118=(a113*a118);
  a97=(a97+a118);
  a103=(a103+a29);
  a103=(a112*a103);
  a97=(a97+a103);
  a103=arg[2]? arg[2][3] : 0;
  a118=(a32/a33);
  a118=(a4*a118);
  a118=(a31+a118);
  a119=(a35/a33);
  a119=(a4*a119);
  a119=(a12+a119);
  a118=(a118/a119);
  a118=(a118+a36);
  a118=(a103*a118);
  a97=(a97+a118);
  a118=arg[2]? arg[2][4] : 0;
  a119=(a38/a33);
  a119=(a4*a119);
  a119=(a37+a119);
  a120=(a35/a33);
  a120=(a4*a120);
  a120=(a12+a120);
  a119=(a119/a120);
  a119=(a119+a40);
  a119=(a118*a119);
  a97=(a97+a119);
  a119=arg[2]? arg[2][5] : 0;
  a120=(a42/a33);
  a120=(a4*a120);
  a120=(a41+a120);
  a121=(a35/a33);
  a121=(a4*a121);
  a121=(a12+a121);
  a120=(a120/a121);
  a120=(a120+a44);
  a120=(a119*a120);
  a97=(a97+a120);
  a120=arg[2]? arg[2][54] : 0;
  a121=(a120*a45);
  a97=(a97+a121);
  a111=(a111-a97);
  a97=(a111<=a17);
  a97=(!a97);
  a111=casadi_fmax(a17,a111);
  a111=(a111+a111);
  a111=(a111*a47);
  a97=(a97*a111);
  a111=(a112*a97);
  a109=(a109*a111);
  a109=(a4*a109);
  a28=(a28*a109);
  a3=(a3-a28);
  a92=(a92/a1);
  a104=(a111/a104);
  a104=(a4*a104);
  a92=(a92*a104);
  a3=(a3+a92);
  a116=(a116/a1);
  a115=(a115/a117);
  a92=(a113*a97);
  a115=(a115*a92);
  a115=(a4*a115);
  a116=(a116*a115);
  a3=(a3-a116);
  a114=(a114/a1);
  a117=(a92/a117);
  a117=(a4*a117);
  a114=(a114*a117);
  a3=(a3+a114);
  a93=(a93/a1);
  a69=(a69/a105);
  a114=(a110*a97);
  a69=(a69*a114);
  a69=(a4*a69);
  a93=(a93*a69);
  a3=(a3-a93);
  a81=(a81/a1);
  a105=(a114/a105);
  a105=(a4*a105);
  a81=(a81*a105);
  a3=(a3+a81);
  a81=(a13/a1);
  a105=(a81/a1);
  a0=(a0/a1);
  a93=(a4*a0);
  a26=(a26+a93);
  a81=(a4*a81);
  a81=(a12+a81);
  a26=(a26/a81);
  a93=(a26/a81);
  a9=(a9/a1);
  a69=(a4*a9);
  a8=(a8+a69);
  a69=(a13/a1);
  a117=(a4*a69);
  a117=(a12+a117);
  a8=(a8/a117);
  a116=(a8+a16);
  a116=(a110*a116);
  a20=(a20/a1);
  a115=(a4*a20);
  a19=(a19+a115);
  a13=(a13/a1);
  a115=(a4*a13);
  a115=(a12+a115);
  a19=(a19/a115);
  a104=(a19+a25);
  a104=(a113*a104);
  a116=(a116+a104);
  a26=(a26+a29);
  a26=(a112*a26);
  a116=(a116+a26);
  a32=(a32/a33);
  a32=(a4*a32);
  a31=(a31+a32);
  a32=(a35/a33);
  a32=(a4*a32);
  a32=(a12+a32);
  a31=(a31/a32);
  a31=(a31+a36);
  a31=(a103*a31);
  a116=(a116+a31);
  a38=(a38/a33);
  a38=(a4*a38);
  a37=(a37+a38);
  a38=(a35/a33);
  a38=(a4*a38);
  a38=(a12+a38);
  a37=(a37/a38);
  a37=(a37+a40);
  a37=(a118*a37);
  a116=(a116+a37);
  a42=(a42/a33);
  a42=(a4*a42);
  a41=(a41+a42);
  a35=(a35/a33);
  a35=(a4*a35);
  a12=(a12+a35);
  a41=(a41/a12);
  a41=(a41+a44);
  a41=(a119*a41);
  a116=(a116+a41);
  a41=(a120*a45);
  a116=(a116+a41);
  a41=arg[2]? arg[2][61] : 0;
  a116=(a116-a41);
  a41=(a116<=a17);
  a41=(!a41);
  a17=casadi_fmax(a17,a116);
  a17=(a17+a17);
  a17=(a17*a47);
  a41=(a41*a17);
  a112=(a112*a41);
  a93=(a93*a112);
  a93=(a4*a93);
  a105=(a105*a93);
  a3=(a3+a105);
  a0=(a0/a1);
  a81=(a112/a81);
  a81=(a4*a81);
  a0=(a0*a81);
  a3=(a3-a0);
  a13=(a13/a1);
  a19=(a19/a115);
  a113=(a113*a41);
  a19=(a19*a113);
  a19=(a4*a19);
  a13=(a13*a19);
  a3=(a3+a13);
  a20=(a20/a1);
  a115=(a113/a115);
  a115=(a4*a115);
  a20=(a20*a115);
  a3=(a3-a20);
  a69=(a69/a1);
  a8=(a8/a117);
  a110=(a110*a41);
  a8=(a8*a110);
  a8=(a4*a8);
  a69=(a69*a8);
  a3=(a3+a69);
  a9=(a9/a1);
  a117=(a110/a117);
  a4=(a4*a117);
  a9=(a9*a4);
  a3=(a3-a9);
  a9=arg[2]? arg[2][75] : 0;
  a4=(a1<a9);
  a117=(!a4);
  a69=arg[2]? arg[2][80] : 0;
  a8=(a1-a9);
  a8=(a8+a8);
  a69=(a69*a8);
  a117=(a117?a69:0);
  a3=(a3+a117);
  a117=arg[2]? arg[2][79] : 0;
  a1=(a1-a9);
  a1=(a1+a1);
  a117=(a117*a1);
  a4=(a4?a117:0);
  a3=(a3+a4);
  if (res[0]!=0) res[0][0]=a3;
  a3=(a43*a11);
  a43=(a43*a46);
  a3=(a3-a43);
  a43=(a60*a48);
  a3=(a3-a43);
  a60=(a60*a10);
  a3=(a3+a60);
  a60=(a72*a49);
  a3=(a3-a60);
  a72=(a72*a14);
  a3=(a3+a72);
  a72=(a84*a61);
  a3=(a3-a72);
  a84=(a84*a15);
  a3=(a3+a84);
  a84=(a96*a73);
  a3=(a3-a84);
  a96=(a96*a24);
  a3=(a3+a96);
  a96=(a108*a85);
  a3=(a3-a96);
  a108=(a108*a57);
  a3=(a3+a108);
  a108=(a120*a97);
  a3=(a3-a108);
  a120=(a120*a41);
  a3=(a3+a120);
  a45=(a45+a45);
  a120=arg[2]? arg[2][81] : 0;
  a45=(a45*a120);
  a3=(a3+a45);
  if (res[0]!=0) res[0][1]=a3;
  a7=(a7-a21);
  a7=(a7-a54);
  a7=(a7+a50);
  a7=(a7-a66);
  a7=(a7+a62);
  a7=(a7-a78);
  a7=(a7+a74);
  a7=(a7-a90);
  a7=(a7+a86);
  a7=(a7-a102);
  a7=(a7+a98);
  a7=(a7-a114);
  a7=(a7+a110);
  a16=(a16+a16);
  a110=arg[2]? arg[2][82] : 0;
  a16=(a16*a110);
  a7=(a7+a16);
  if (res[0]!=0) res[0][2]=a7;
  a18=(a18-a27);
  a18=(a18-a2);
  a18=(a18+a53);
  a18=(a18-a23);
  a18=(a18+a65);
  a18=(a18-a56);
  a18=(a18+a77);
  a18=(a18-a68);
  a18=(a18+a89);
  a18=(a18-a80);
  a18=(a18+a101);
  a18=(a18-a92);
  a18=(a18+a113);
  a25=(a25+a25);
  a25=(a25*a110);
  a18=(a18+a25);
  if (res[0]!=0) res[0][3]=a18;
  a5=(a5-a6);
  a5=(a5-a51);
  a5=(a5+a52);
  a5=(a5-a63);
  a5=(a5+a64);
  a5=(a5-a75);
  a5=(a5+a76);
  a5=(a5-a87);
  a5=(a5+a88);
  a5=(a5-a99);
  a5=(a5+a100);
  a5=(a5-a111);
  a5=(a5+a112);
  a29=(a29+a29);
  a29=(a29*a110);
  a5=(a5+a29);
  if (res[0]!=0) res[0][4]=a5;
  a5=(a30*a11);
  a30=(a30*a46);
  a5=(a5-a30);
  a30=(a22*a48);
  a5=(a5-a30);
  a22=(a22*a10);
  a5=(a5+a22);
  a22=(a55*a49);
  a5=(a5-a22);
  a55=(a55*a14);
  a5=(a5+a55);
  a55=(a67*a61);
  a5=(a5-a55);
  a67=(a67*a15);
  a5=(a5+a67);
  a67=(a79*a73);
  a5=(a5-a67);
  a79=(a79*a24);
  a5=(a5+a79);
  a79=(a91*a85);
  a5=(a5-a79);
  a91=(a91*a57);
  a5=(a5+a91);
  a91=(a103*a97);
  a5=(a5-a91);
  a103=(a103*a41);
  a5=(a5+a103);
  a36=(a36+a36);
  a36=(a36*a110);
  a5=(a5+a36);
  if (res[0]!=0) res[0][5]=a5;
  a5=(a34*a11);
  a34=(a34*a46);
  a5=(a5-a34);
  a34=(a58*a48);
  a5=(a5-a34);
  a58=(a58*a10);
  a5=(a5+a58);
  a58=(a70*a49);
  a5=(a5-a58);
  a70=(a70*a14);
  a5=(a5+a70);
  a70=(a82*a61);
  a5=(a5-a70);
  a82=(a82*a15);
  a5=(a5+a82);
  a82=(a94*a73);
  a5=(a5-a82);
  a94=(a94*a24);
  a5=(a5+a94);
  a94=(a106*a85);
  a5=(a5-a94);
  a106=(a106*a57);
  a5=(a5+a106);
  a106=(a118*a97);
  a5=(a5-a106);
  a118=(a118*a41);
  a5=(a5+a118);
  a40=(a40+a40);
  a40=(a40*a110);
  a5=(a5+a40);
  if (res[0]!=0) res[0][6]=a5;
  a11=(a39*a11);
  a39=(a39*a46);
  a11=(a11-a39);
  a48=(a59*a48);
  a11=(a11-a48);
  a59=(a59*a10);
  a11=(a11+a59);
  a49=(a71*a49);
  a11=(a11-a49);
  a71=(a71*a14);
  a11=(a11+a71);
  a61=(a83*a61);
  a11=(a11-a61);
  a83=(a83*a15);
  a11=(a11+a83);
  a73=(a95*a73);
  a11=(a11-a73);
  a95=(a95*a24);
  a11=(a11+a95);
  a85=(a107*a85);
  a11=(a11-a85);
  a107=(a107*a57);
  a11=(a11+a107);
  a97=(a119*a97);
  a11=(a11-a97);
  a119=(a119*a41);
  a11=(a11+a119);
  a44=(a44+a44);
  a44=(a44*a110);
  a11=(a11+a44);
  if (res[0]!=0) res[0][7]=a11;
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_ZcBOCJeGbhGljDTUIEQn(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int grad_phi_ZcBOCJeGbhGljDTUIEQn_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_ZcBOCJeGbhGljDTUIEQn_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_ZcBOCJeGbhGljDTUIEQn_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int grad_phi_ZcBOCJeGbhGljDTUIEQn_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_ZcBOCJeGbhGljDTUIEQn_release(int mem) {
}

CASADI_SYMBOL_EXPORT void grad_phi_ZcBOCJeGbhGljDTUIEQn_incref(void) {
}

CASADI_SYMBOL_EXPORT void grad_phi_ZcBOCJeGbhGljDTUIEQn_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_ZcBOCJeGbhGljDTUIEQn_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_ZcBOCJeGbhGljDTUIEQn_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real grad_phi_ZcBOCJeGbhGljDTUIEQn_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_ZcBOCJeGbhGljDTUIEQn_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_ZcBOCJeGbhGljDTUIEQn_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_ZcBOCJeGbhGljDTUIEQn_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_ZcBOCJeGbhGljDTUIEQn_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int grad_phi_ZcBOCJeGbhGljDTUIEQn_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif