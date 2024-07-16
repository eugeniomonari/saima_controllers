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
  #define CASADI_PREFIX(ID) grad_phi_qkoMKQYolbocpRexLwcl_ ## ID
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
static const casadi_int casadi_s2[79] = {75, 1, 0, 75, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74};

/* grad_phi_qkoMKQYolbocpRexLwcl:(i0[8],i1,i2[75])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[2]? arg[2][44] : 0;
  a1=arg[0]? arg[0][0] : 0;
  a2=(a0/a1);
  a3=(a2/a1);
  a4=arg[2]? arg[2][38] : 0;
  a5=arg[2]? arg[2][68] : 0;
  a6=arg[2]? arg[2][36] : 0;
  a7=arg[2]? arg[2][42] : 0;
  a8=(a7/a1);
  a9=arg[0]? arg[0][2] : 0;
  a10=(a8+a9);
  a10=(a6*a10);
  a11=arg[2]? arg[2][37] : 0;
  a12=arg[2]? arg[2][43] : 0;
  a13=(a12/a1);
  a14=arg[0]? arg[0][3] : 0;
  a15=(a13+a14);
  a15=(a11*a15);
  a10=(a10+a15);
  a15=arg[0]? arg[0][4] : 0;
  a2=(a2+a15);
  a2=(a4*a2);
  a10=(a10+a2);
  a2=arg[2]? arg[2][39] : 0;
  a16=arg[2]? arg[2][45] : 0;
  a17=arg[2]? arg[2][70] : 0;
  a18=(a16/a17);
  a19=arg[0]? arg[0][5] : 0;
  a18=(a18+a19);
  a18=(a2*a18);
  a10=(a10+a18);
  a18=arg[2]? arg[2][40] : 0;
  a20=arg[2]? arg[2][46] : 0;
  a21=(a20/a17);
  a22=arg[0]? arg[0][6] : 0;
  a21=(a21+a22);
  a21=(a18*a21);
  a10=(a10+a21);
  a21=arg[2]? arg[2][41] : 0;
  a23=arg[2]? arg[2][47] : 0;
  a24=(a23/a17);
  a25=arg[0]? arg[0][7] : 0;
  a24=(a24+a25);
  a24=(a21*a24);
  a10=(a10+a24);
  a24=arg[2]? arg[2][54] : 0;
  a26=arg[0]? arg[0][1] : 0;
  a27=(a24*a26);
  a10=(a10+a27);
  a5=(a5-a10);
  a10=0.;
  a27=(a5<=a10);
  a27=(!a27);
  a5=casadi_fmax(a10,a5);
  a5=(a5+a5);
  a28=5.0000000000000000e-01;
  a29=arg[1]? arg[1][0] : 0;
  a28=(a28*a29);
  a5=(a5*a28);
  a27=(a27*a5);
  a5=(a4*a27);
  a3=(a3*a5);
  a13=(a13/a1);
  a29=(a11*a27);
  a13=(a13*a29);
  a3=(a3+a13);
  a8=(a8/a1);
  a13=(a6*a27);
  a8=(a8*a13);
  a3=(a3+a8);
  a8=(a0/a1);
  a30=(a8/a1);
  a31=(a7/a1);
  a32=(a31+a9);
  a32=(a6*a32);
  a33=(a12/a1);
  a34=(a33+a14);
  a34=(a11*a34);
  a32=(a32+a34);
  a8=(a8+a15);
  a8=(a4*a8);
  a32=(a32+a8);
  a8=(a16/a17);
  a8=(a8+a19);
  a8=(a2*a8);
  a32=(a32+a8);
  a8=(a20/a17);
  a8=(a8+a22);
  a8=(a18*a8);
  a32=(a32+a8);
  a8=(a23/a17);
  a8=(a8+a25);
  a8=(a21*a8);
  a32=(a32+a8);
  a8=(a24*a26);
  a32=(a32+a8);
  a8=arg[2]? arg[2][61] : 0;
  a32=(a32-a8);
  a8=(a32<=a10);
  a8=(!a8);
  a32=casadi_fmax(a10,a32);
  a32=(a32+a32);
  a32=(a32*a28);
  a8=(a8*a32);
  a4=(a4*a8);
  a30=(a30*a4);
  a3=(a3-a30);
  a33=(a33/a1);
  a11=(a11*a8);
  a33=(a33*a11);
  a3=(a3-a33);
  a31=(a31/a1);
  a6=(a6*a8);
  a31=(a31*a6);
  a3=(a3-a31);
  a31=(a0/a1);
  a33=(a31/a1);
  a30=arg[2]? arg[2][32] : 0;
  a32=arg[2]? arg[2][67] : 0;
  a34=arg[2]? arg[2][30] : 0;
  a35=(a7/a1);
  a36=(a35+a9);
  a36=(a34*a36);
  a37=arg[2]? arg[2][31] : 0;
  a38=(a12/a1);
  a39=(a38+a14);
  a39=(a37*a39);
  a36=(a36+a39);
  a31=(a31+a15);
  a31=(a30*a31);
  a36=(a36+a31);
  a31=arg[2]? arg[2][33] : 0;
  a39=(a16/a17);
  a39=(a39+a19);
  a39=(a31*a39);
  a36=(a36+a39);
  a39=arg[2]? arg[2][34] : 0;
  a40=(a20/a17);
  a40=(a40+a22);
  a40=(a39*a40);
  a36=(a36+a40);
  a40=arg[2]? arg[2][35] : 0;
  a41=(a23/a17);
  a41=(a41+a25);
  a41=(a40*a41);
  a36=(a36+a41);
  a41=arg[2]? arg[2][53] : 0;
  a42=(a41*a26);
  a36=(a36+a42);
  a32=(a32-a36);
  a36=(a32<=a10);
  a36=(!a36);
  a32=casadi_fmax(a10,a32);
  a32=(a32+a32);
  a32=(a32*a28);
  a36=(a36*a32);
  a32=(a30*a36);
  a33=(a33*a32);
  a3=(a3+a33);
  a38=(a38/a1);
  a33=(a37*a36);
  a38=(a38*a33);
  a3=(a3+a38);
  a35=(a35/a1);
  a38=(a34*a36);
  a35=(a35*a38);
  a3=(a3+a35);
  a35=(a0/a1);
  a42=(a35/a1);
  a43=(a7/a1);
  a44=(a43+a9);
  a44=(a34*a44);
  a45=(a12/a1);
  a46=(a45+a14);
  a46=(a37*a46);
  a44=(a44+a46);
  a35=(a35+a15);
  a35=(a30*a35);
  a44=(a44+a35);
  a35=(a16/a17);
  a35=(a35+a19);
  a35=(a31*a35);
  a44=(a44+a35);
  a35=(a20/a17);
  a35=(a35+a22);
  a35=(a39*a35);
  a44=(a44+a35);
  a35=(a23/a17);
  a35=(a35+a25);
  a35=(a40*a35);
  a44=(a44+a35);
  a35=(a41*a26);
  a44=(a44+a35);
  a35=arg[2]? arg[2][60] : 0;
  a44=(a44-a35);
  a35=(a44<=a10);
  a35=(!a35);
  a44=casadi_fmax(a10,a44);
  a44=(a44+a44);
  a44=(a44*a28);
  a35=(a35*a44);
  a30=(a30*a35);
  a42=(a42*a30);
  a3=(a3-a42);
  a45=(a45/a1);
  a37=(a37*a35);
  a45=(a45*a37);
  a3=(a3-a45);
  a43=(a43/a1);
  a34=(a34*a35);
  a43=(a43*a34);
  a3=(a3-a43);
  a43=(a0/a1);
  a45=(a43/a1);
  a42=arg[2]? arg[2][26] : 0;
  a44=arg[2]? arg[2][66] : 0;
  a46=arg[2]? arg[2][24] : 0;
  a47=(a7/a1);
  a48=(a47+a9);
  a48=(a46*a48);
  a49=arg[2]? arg[2][25] : 0;
  a50=(a12/a1);
  a51=(a50+a14);
  a51=(a49*a51);
  a48=(a48+a51);
  a43=(a43+a15);
  a43=(a42*a43);
  a48=(a48+a43);
  a43=arg[2]? arg[2][27] : 0;
  a51=(a16/a17);
  a51=(a51+a19);
  a51=(a43*a51);
  a48=(a48+a51);
  a51=arg[2]? arg[2][28] : 0;
  a52=(a20/a17);
  a52=(a52+a22);
  a52=(a51*a52);
  a48=(a48+a52);
  a52=arg[2]? arg[2][29] : 0;
  a53=(a23/a17);
  a53=(a53+a25);
  a53=(a52*a53);
  a48=(a48+a53);
  a53=arg[2]? arg[2][52] : 0;
  a54=(a53*a26);
  a48=(a48+a54);
  a44=(a44-a48);
  a48=(a44<=a10);
  a48=(!a48);
  a44=casadi_fmax(a10,a44);
  a44=(a44+a44);
  a44=(a44*a28);
  a48=(a48*a44);
  a44=(a42*a48);
  a45=(a45*a44);
  a3=(a3+a45);
  a50=(a50/a1);
  a45=(a49*a48);
  a50=(a50*a45);
  a3=(a3+a50);
  a47=(a47/a1);
  a50=(a46*a48);
  a47=(a47*a50);
  a3=(a3+a47);
  a47=(a0/a1);
  a54=(a47/a1);
  a55=(a7/a1);
  a56=(a55+a9);
  a56=(a46*a56);
  a57=(a12/a1);
  a58=(a57+a14);
  a58=(a49*a58);
  a56=(a56+a58);
  a47=(a47+a15);
  a47=(a42*a47);
  a56=(a56+a47);
  a47=(a16/a17);
  a47=(a47+a19);
  a47=(a43*a47);
  a56=(a56+a47);
  a47=(a20/a17);
  a47=(a47+a22);
  a47=(a51*a47);
  a56=(a56+a47);
  a47=(a23/a17);
  a47=(a47+a25);
  a47=(a52*a47);
  a56=(a56+a47);
  a47=(a53*a26);
  a56=(a56+a47);
  a47=arg[2]? arg[2][59] : 0;
  a56=(a56-a47);
  a47=(a56<=a10);
  a47=(!a47);
  a56=casadi_fmax(a10,a56);
  a56=(a56+a56);
  a56=(a56*a28);
  a47=(a47*a56);
  a42=(a42*a47);
  a54=(a54*a42);
  a3=(a3-a54);
  a57=(a57/a1);
  a49=(a49*a47);
  a57=(a57*a49);
  a3=(a3-a57);
  a55=(a55/a1);
  a46=(a46*a47);
  a55=(a55*a46);
  a3=(a3-a55);
  a55=(a0/a1);
  a57=(a55/a1);
  a54=arg[2]? arg[2][20] : 0;
  a56=arg[2]? arg[2][65] : 0;
  a58=arg[2]? arg[2][18] : 0;
  a59=(a7/a1);
  a60=(a59+a9);
  a60=(a58*a60);
  a61=arg[2]? arg[2][19] : 0;
  a62=(a12/a1);
  a63=(a62+a14);
  a63=(a61*a63);
  a60=(a60+a63);
  a55=(a55+a15);
  a55=(a54*a55);
  a60=(a60+a55);
  a55=arg[2]? arg[2][21] : 0;
  a63=(a16/a17);
  a63=(a63+a19);
  a63=(a55*a63);
  a60=(a60+a63);
  a63=arg[2]? arg[2][22] : 0;
  a64=(a20/a17);
  a64=(a64+a22);
  a64=(a63*a64);
  a60=(a60+a64);
  a64=arg[2]? arg[2][23] : 0;
  a65=(a23/a17);
  a65=(a65+a25);
  a65=(a64*a65);
  a60=(a60+a65);
  a65=arg[2]? arg[2][51] : 0;
  a66=(a65*a26);
  a60=(a60+a66);
  a56=(a56-a60);
  a60=(a56<=a10);
  a60=(!a60);
  a56=casadi_fmax(a10,a56);
  a56=(a56+a56);
  a56=(a56*a28);
  a60=(a60*a56);
  a56=(a54*a60);
  a57=(a57*a56);
  a3=(a3+a57);
  a62=(a62/a1);
  a57=(a61*a60);
  a62=(a62*a57);
  a3=(a3+a62);
  a59=(a59/a1);
  a62=(a58*a60);
  a59=(a59*a62);
  a3=(a3+a59);
  a59=(a0/a1);
  a66=(a59/a1);
  a67=(a7/a1);
  a68=(a67+a9);
  a68=(a58*a68);
  a69=(a12/a1);
  a70=(a69+a14);
  a70=(a61*a70);
  a68=(a68+a70);
  a59=(a59+a15);
  a59=(a54*a59);
  a68=(a68+a59);
  a59=(a16/a17);
  a59=(a59+a19);
  a59=(a55*a59);
  a68=(a68+a59);
  a59=(a20/a17);
  a59=(a59+a22);
  a59=(a63*a59);
  a68=(a68+a59);
  a59=(a23/a17);
  a59=(a59+a25);
  a59=(a64*a59);
  a68=(a68+a59);
  a59=(a65*a26);
  a68=(a68+a59);
  a59=arg[2]? arg[2][58] : 0;
  a68=(a68-a59);
  a59=(a68<=a10);
  a59=(!a59);
  a68=casadi_fmax(a10,a68);
  a68=(a68+a68);
  a68=(a68*a28);
  a59=(a59*a68);
  a54=(a54*a59);
  a66=(a66*a54);
  a3=(a3-a66);
  a69=(a69/a1);
  a61=(a61*a59);
  a69=(a69*a61);
  a3=(a3-a69);
  a67=(a67/a1);
  a58=(a58*a59);
  a67=(a67*a58);
  a3=(a3-a67);
  a67=(a0/a1);
  a69=(a67/a1);
  a66=arg[2]? arg[2][14] : 0;
  a68=arg[2]? arg[2][64] : 0;
  a70=arg[2]? arg[2][12] : 0;
  a71=(a7/a1);
  a72=(a71+a9);
  a72=(a70*a72);
  a73=arg[2]? arg[2][13] : 0;
  a74=(a12/a1);
  a75=(a74+a14);
  a75=(a73*a75);
  a72=(a72+a75);
  a67=(a67+a15);
  a67=(a66*a67);
  a72=(a72+a67);
  a67=arg[2]? arg[2][15] : 0;
  a75=(a16/a17);
  a75=(a75+a19);
  a75=(a67*a75);
  a72=(a72+a75);
  a75=arg[2]? arg[2][16] : 0;
  a76=(a20/a17);
  a76=(a76+a22);
  a76=(a75*a76);
  a72=(a72+a76);
  a76=arg[2]? arg[2][17] : 0;
  a77=(a23/a17);
  a77=(a77+a25);
  a77=(a76*a77);
  a72=(a72+a77);
  a77=arg[2]? arg[2][50] : 0;
  a78=(a77*a26);
  a72=(a72+a78);
  a68=(a68-a72);
  a72=(a68<=a10);
  a72=(!a72);
  a68=casadi_fmax(a10,a68);
  a68=(a68+a68);
  a68=(a68*a28);
  a72=(a72*a68);
  a68=(a66*a72);
  a69=(a69*a68);
  a3=(a3+a69);
  a74=(a74/a1);
  a69=(a73*a72);
  a74=(a74*a69);
  a3=(a3+a74);
  a71=(a71/a1);
  a74=(a70*a72);
  a71=(a71*a74);
  a3=(a3+a71);
  a71=(a0/a1);
  a78=(a71/a1);
  a79=(a7/a1);
  a80=(a79+a9);
  a80=(a70*a80);
  a81=(a12/a1);
  a82=(a81+a14);
  a82=(a73*a82);
  a80=(a80+a82);
  a71=(a71+a15);
  a71=(a66*a71);
  a80=(a80+a71);
  a71=(a16/a17);
  a71=(a71+a19);
  a71=(a67*a71);
  a80=(a80+a71);
  a71=(a20/a17);
  a71=(a71+a22);
  a71=(a75*a71);
  a80=(a80+a71);
  a71=(a23/a17);
  a71=(a71+a25);
  a71=(a76*a71);
  a80=(a80+a71);
  a71=(a77*a26);
  a80=(a80+a71);
  a71=arg[2]? arg[2][57] : 0;
  a80=(a80-a71);
  a71=(a80<=a10);
  a71=(!a71);
  a80=casadi_fmax(a10,a80);
  a80=(a80+a80);
  a80=(a80*a28);
  a71=(a71*a80);
  a66=(a66*a71);
  a78=(a78*a66);
  a3=(a3-a78);
  a81=(a81/a1);
  a73=(a73*a71);
  a81=(a81*a73);
  a3=(a3-a81);
  a79=(a79/a1);
  a70=(a70*a71);
  a79=(a79*a70);
  a3=(a3-a79);
  a79=(a0/a1);
  a81=(a79/a1);
  a78=arg[2]? arg[2][8] : 0;
  a80=arg[2]? arg[2][63] : 0;
  a82=arg[2]? arg[2][6] : 0;
  a83=(a7/a1);
  a84=(a83+a9);
  a84=(a82*a84);
  a85=arg[2]? arg[2][7] : 0;
  a86=(a12/a1);
  a87=(a86+a14);
  a87=(a85*a87);
  a84=(a84+a87);
  a79=(a79+a15);
  a79=(a78*a79);
  a84=(a84+a79);
  a79=arg[2]? arg[2][9] : 0;
  a87=(a16/a17);
  a87=(a87+a19);
  a87=(a79*a87);
  a84=(a84+a87);
  a87=arg[2]? arg[2][10] : 0;
  a88=(a20/a17);
  a88=(a88+a22);
  a88=(a87*a88);
  a84=(a84+a88);
  a88=arg[2]? arg[2][11] : 0;
  a89=(a23/a17);
  a89=(a89+a25);
  a89=(a88*a89);
  a84=(a84+a89);
  a89=arg[2]? arg[2][49] : 0;
  a90=(a89*a26);
  a84=(a84+a90);
  a80=(a80-a84);
  a84=(a80<=a10);
  a84=(!a84);
  a80=casadi_fmax(a10,a80);
  a80=(a80+a80);
  a80=(a80*a28);
  a84=(a84*a80);
  a80=(a78*a84);
  a81=(a81*a80);
  a3=(a3+a81);
  a86=(a86/a1);
  a81=(a85*a84);
  a86=(a86*a81);
  a3=(a3+a86);
  a83=(a83/a1);
  a86=(a82*a84);
  a83=(a83*a86);
  a3=(a3+a83);
  a83=(a0/a1);
  a90=(a83/a1);
  a91=(a7/a1);
  a92=(a91+a9);
  a92=(a82*a92);
  a93=(a12/a1);
  a94=(a93+a14);
  a94=(a85*a94);
  a92=(a92+a94);
  a83=(a83+a15);
  a83=(a78*a83);
  a92=(a92+a83);
  a83=(a16/a17);
  a83=(a83+a19);
  a83=(a79*a83);
  a92=(a92+a83);
  a83=(a20/a17);
  a83=(a83+a22);
  a83=(a87*a83);
  a92=(a92+a83);
  a83=(a23/a17);
  a83=(a83+a25);
  a83=(a88*a83);
  a92=(a92+a83);
  a83=(a89*a26);
  a92=(a92+a83);
  a83=arg[2]? arg[2][56] : 0;
  a92=(a92-a83);
  a83=(a92<=a10);
  a83=(!a83);
  a92=casadi_fmax(a10,a92);
  a92=(a92+a92);
  a92=(a92*a28);
  a83=(a83*a92);
  a78=(a78*a83);
  a90=(a90*a78);
  a3=(a3-a90);
  a93=(a93/a1);
  a85=(a85*a83);
  a93=(a93*a85);
  a3=(a3-a93);
  a91=(a91/a1);
  a82=(a82*a83);
  a91=(a91*a82);
  a3=(a3-a91);
  a91=(a0/a1);
  a93=(a91/a1);
  a90=arg[2]? arg[2][2] : 0;
  a92=arg[2]? arg[2][62] : 0;
  a94=arg[2]? arg[2][0] : 0;
  a95=(a7/a1);
  a96=(a95+a9);
  a96=(a94*a96);
  a97=arg[2]? arg[2][1] : 0;
  a98=(a12/a1);
  a99=(a98+a14);
  a99=(a97*a99);
  a96=(a96+a99);
  a91=(a91+a15);
  a91=(a90*a91);
  a96=(a96+a91);
  a91=arg[2]? arg[2][3] : 0;
  a99=(a16/a17);
  a99=(a99+a19);
  a99=(a91*a99);
  a96=(a96+a99);
  a99=arg[2]? arg[2][4] : 0;
  a100=(a20/a17);
  a100=(a100+a22);
  a100=(a99*a100);
  a96=(a96+a100);
  a100=arg[2]? arg[2][5] : 0;
  a101=(a23/a17);
  a101=(a101+a25);
  a101=(a100*a101);
  a96=(a96+a101);
  a101=arg[2]? arg[2][48] : 0;
  a102=(a101*a26);
  a96=(a96+a102);
  a92=(a92-a96);
  a96=(a92<=a10);
  a96=(!a96);
  a92=casadi_fmax(a10,a92);
  a92=(a92+a92);
  a92=(a92*a28);
  a96=(a96*a92);
  a92=(a90*a96);
  a93=(a93*a92);
  a3=(a3+a93);
  a98=(a98/a1);
  a93=(a97*a96);
  a98=(a98*a93);
  a3=(a3+a98);
  a95=(a95/a1);
  a98=(a94*a96);
  a95=(a95*a98);
  a3=(a3+a95);
  a0=(a0/a1);
  a95=(a0/a1);
  a7=(a7/a1);
  a102=(a7+a9);
  a102=(a94*a102);
  a12=(a12/a1);
  a103=(a12+a14);
  a103=(a97*a103);
  a102=(a102+a103);
  a0=(a0+a15);
  a0=(a90*a0);
  a102=(a102+a0);
  a16=(a16/a17);
  a16=(a16+a19);
  a16=(a91*a16);
  a102=(a102+a16);
  a20=(a20/a17);
  a20=(a20+a22);
  a20=(a99*a20);
  a102=(a102+a20);
  a23=(a23/a17);
  a23=(a23+a25);
  a23=(a100*a23);
  a102=(a102+a23);
  a23=(a101*a26);
  a102=(a102+a23);
  a23=arg[2]? arg[2][55] : 0;
  a102=(a102-a23);
  a23=(a102<=a10);
  a23=(!a23);
  a10=casadi_fmax(a10,a102);
  a10=(a10+a10);
  a10=(a10*a28);
  a23=(a23*a10);
  a90=(a90*a23);
  a95=(a95*a90);
  a3=(a3-a95);
  a12=(a12/a1);
  a97=(a97*a23);
  a12=(a12*a97);
  a3=(a3-a12);
  a7=(a7/a1);
  a94=(a94*a23);
  a7=(a7*a94);
  a3=(a3-a7);
  a7=arg[2]? arg[2][69] : 0;
  a12=(a1<a7);
  a95=(!a12);
  a10=arg[2]? arg[2][72] : 0;
  a28=(a1-a7);
  a28=(a28+a28);
  a10=(a10*a28);
  a95=(a95?a10:0);
  a3=(a3+a95);
  a95=arg[2]? arg[2][71] : 0;
  a1=(a1-a7);
  a1=(a1+a1);
  a95=(a95*a1);
  a12=(a12?a95:0);
  a3=(a3+a12);
  if (res[0]!=0) res[0][0]=a3;
  a3=(a24*a8);
  a24=(a24*a27);
  a3=(a3-a24);
  a24=(a41*a36);
  a3=(a3-a24);
  a41=(a41*a35);
  a3=(a3+a41);
  a41=(a53*a48);
  a3=(a3-a41);
  a53=(a53*a47);
  a3=(a3+a53);
  a53=(a65*a60);
  a3=(a3-a53);
  a65=(a65*a59);
  a3=(a3+a65);
  a65=(a77*a72);
  a3=(a3-a65);
  a77=(a77*a71);
  a3=(a3+a77);
  a77=(a89*a84);
  a3=(a3-a77);
  a89=(a89*a83);
  a3=(a3+a89);
  a89=(a101*a96);
  a3=(a3-a89);
  a101=(a101*a23);
  a3=(a3+a101);
  a26=(a26+a26);
  a101=arg[2]? arg[2][73] : 0;
  a26=(a26*a101);
  a3=(a3+a26);
  if (res[0]!=0) res[0][1]=a3;
  a6=(a6-a13);
  a6=(a6-a38);
  a6=(a6+a34);
  a6=(a6-a50);
  a6=(a6+a46);
  a6=(a6-a62);
  a6=(a6+a58);
  a6=(a6-a74);
  a6=(a6+a70);
  a6=(a6-a86);
  a6=(a6+a82);
  a6=(a6-a98);
  a6=(a6+a94);
  a9=(a9+a9);
  a94=arg[2]? arg[2][74] : 0;
  a9=(a9*a94);
  a6=(a6+a9);
  if (res[0]!=0) res[0][2]=a6;
  a11=(a11-a29);
  a11=(a11-a33);
  a11=(a11+a37);
  a11=(a11-a45);
  a11=(a11+a49);
  a11=(a11-a57);
  a11=(a11+a61);
  a11=(a11-a69);
  a11=(a11+a73);
  a11=(a11-a81);
  a11=(a11+a85);
  a11=(a11-a93);
  a11=(a11+a97);
  a14=(a14+a14);
  a14=(a14*a94);
  a11=(a11+a14);
  if (res[0]!=0) res[0][3]=a11;
  a4=(a4-a5);
  a4=(a4-a32);
  a4=(a4+a30);
  a4=(a4-a44);
  a4=(a4+a42);
  a4=(a4-a56);
  a4=(a4+a54);
  a4=(a4-a68);
  a4=(a4+a66);
  a4=(a4-a80);
  a4=(a4+a78);
  a4=(a4-a92);
  a4=(a4+a90);
  a15=(a15+a15);
  a15=(a15*a94);
  a4=(a4+a15);
  if (res[0]!=0) res[0][4]=a4;
  a4=(a2*a8);
  a2=(a2*a27);
  a4=(a4-a2);
  a2=(a31*a36);
  a4=(a4-a2);
  a31=(a31*a35);
  a4=(a4+a31);
  a31=(a43*a48);
  a4=(a4-a31);
  a43=(a43*a47);
  a4=(a4+a43);
  a43=(a55*a60);
  a4=(a4-a43);
  a55=(a55*a59);
  a4=(a4+a55);
  a55=(a67*a72);
  a4=(a4-a55);
  a67=(a67*a71);
  a4=(a4+a67);
  a67=(a79*a84);
  a4=(a4-a67);
  a79=(a79*a83);
  a4=(a4+a79);
  a79=(a91*a96);
  a4=(a4-a79);
  a91=(a91*a23);
  a4=(a4+a91);
  a19=(a19+a19);
  a19=(a19*a94);
  a4=(a4+a19);
  if (res[0]!=0) res[0][5]=a4;
  a4=(a18*a8);
  a18=(a18*a27);
  a4=(a4-a18);
  a18=(a39*a36);
  a4=(a4-a18);
  a39=(a39*a35);
  a4=(a4+a39);
  a39=(a51*a48);
  a4=(a4-a39);
  a51=(a51*a47);
  a4=(a4+a51);
  a51=(a63*a60);
  a4=(a4-a51);
  a63=(a63*a59);
  a4=(a4+a63);
  a63=(a75*a72);
  a4=(a4-a63);
  a75=(a75*a71);
  a4=(a4+a75);
  a75=(a87*a84);
  a4=(a4-a75);
  a87=(a87*a83);
  a4=(a4+a87);
  a87=(a99*a96);
  a4=(a4-a87);
  a99=(a99*a23);
  a4=(a4+a99);
  a22=(a22+a22);
  a22=(a22*a94);
  a4=(a4+a22);
  if (res[0]!=0) res[0][6]=a4;
  a8=(a21*a8);
  a21=(a21*a27);
  a8=(a8-a21);
  a36=(a40*a36);
  a8=(a8-a36);
  a40=(a40*a35);
  a8=(a8+a40);
  a48=(a52*a48);
  a8=(a8-a48);
  a52=(a52*a47);
  a8=(a8+a52);
  a60=(a64*a60);
  a8=(a8-a60);
  a64=(a64*a59);
  a8=(a8+a64);
  a72=(a76*a72);
  a8=(a8-a72);
  a76=(a76*a71);
  a8=(a8+a76);
  a84=(a88*a84);
  a8=(a8-a84);
  a88=(a88*a83);
  a8=(a8+a88);
  a96=(a100*a96);
  a8=(a8-a96);
  a100=(a100*a23);
  a8=(a8+a100);
  a25=(a25+a25);
  a25=(a25*a94);
  a8=(a8+a25);
  if (res[0]!=0) res[0][7]=a8;
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_qkoMKQYolbocpRexLwcl(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int grad_phi_qkoMKQYolbocpRexLwcl_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_qkoMKQYolbocpRexLwcl_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_qkoMKQYolbocpRexLwcl_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int grad_phi_qkoMKQYolbocpRexLwcl_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_qkoMKQYolbocpRexLwcl_release(int mem) {
}

CASADI_SYMBOL_EXPORT void grad_phi_qkoMKQYolbocpRexLwcl_incref(void) {
}

CASADI_SYMBOL_EXPORT void grad_phi_qkoMKQYolbocpRexLwcl_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_qkoMKQYolbocpRexLwcl_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_qkoMKQYolbocpRexLwcl_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real grad_phi_qkoMKQYolbocpRexLwcl_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_qkoMKQYolbocpRexLwcl_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_qkoMKQYolbocpRexLwcl_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_qkoMKQYolbocpRexLwcl_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_qkoMKQYolbocpRexLwcl_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int grad_phi_qkoMKQYolbocpRexLwcl_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
