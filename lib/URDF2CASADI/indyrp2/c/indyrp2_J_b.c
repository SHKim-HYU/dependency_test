/* This file was automatically generated by CasADi 3.6.3+.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) indyrp2_J_b_ ## ID
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

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[52] = {6, 7, 0, 6, 12, 18, 24, 30, 36, 42, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};

/* J_b:(q[7])->(J_b[6x7]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=sin(a0);
  a2=2.2204460492503131e-16;
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a3=sin(a3);
  a5=(a5+a3);
  a6=(a1*a5);
  a0=cos(a0);
  a7=(a2*a3);
  a8=(a0*a7);
  a6=(a6+a8);
  a8=arg[0]? arg[0][2] : 0;
  a9=cos(a8);
  a10=(a2*a9);
  a11=-2.2204460492503131e-16;
  a8=sin(a8);
  a12=(a11*a8);
  a10=(a10+a12);
  a12=(a6*a10);
  a13=(a2*a3);
  a13=(a4-a13);
  a14=(a1*a13);
  a15=(a2*a4);
  a16=(a0*a15);
  a14=(a14+a16);
  a16=(a14*a9);
  a17=(a2*a1);
  a17=(a17-a0);
  a18=(a17*a8);
  a16=(a16-a18);
  a12=(a12+a16);
  a16=arg[0]? arg[0][3] : 0;
  a18=cos(a16);
  a19=(a2*a18);
  a16=sin(a16);
  a19=(a19+a16);
  a20=(a12*a19);
  a21=(a11*a9);
  a22=(a2*a8);
  a21=(a21-a22);
  a22=(a6*a21);
  a23=(a14*a8);
  a24=(a17*a9);
  a23=(a23+a24);
  a22=(a22-a23);
  a23=(a2*a16);
  a24=(a22*a23);
  a14=(a2*a14);
  a25=(a2*a17);
  a14=(a14+a25);
  a14=(a14-a6);
  a25=(a2*a16);
  a25=(a25-a18);
  a26=(a14*a25);
  a24=(a24+a26);
  a20=(a20+a24);
  a24=arg[0]? arg[0][4] : 0;
  a26=cos(a24);
  a27=(a2*a26);
  a24=sin(a24);
  a28=(a11*a24);
  a27=(a27+a28);
  a28=(a20*a27);
  a29=(a2*a16);
  a29=(a18-a29);
  a30=(a12*a29);
  a31=(a2*a18);
  a32=(a22*a31);
  a18=(a2*a18);
  a16=(a16+a18);
  a18=(a14*a16);
  a32=(a32+a18);
  a30=(a30+a32);
  a32=(a30*a26);
  a12=(a2*a12);
  a12=(a12-a22);
  a18=(a12*a24);
  a32=(a32-a18);
  a28=(a28+a32);
  a32=arg[0]? arg[0][5] : 0;
  a18=cos(a32);
  a33=(a2*a18);
  a32=sin(a32);
  a33=(a33+a32);
  a34=(a28*a33);
  a35=(a11*a26);
  a36=(a2*a24);
  a35=(a35-a36);
  a36=(a20*a35);
  a37=(a30*a24);
  a38=(a12*a26);
  a37=(a37+a38);
  a36=(a36-a37);
  a37=(a2*a32);
  a38=(a36*a37);
  a30=(a2*a30);
  a39=(a2*a12);
  a30=(a30+a39);
  a30=(a30-a20);
  a39=(a2*a32);
  a39=(a39-a18);
  a40=(a30*a39);
  a38=(a38+a40);
  a34=(a34+a38);
  a38=arg[0]? arg[0][6] : 0;
  a40=cos(a38);
  a41=(a2*a40);
  a38=sin(a38);
  a42=(a11*a38);
  a41=(a41+a42);
  a42=(a34*a41);
  a43=(a2*a32);
  a43=(a18-a43);
  a44=(a28*a43);
  a45=(a2*a18);
  a46=(a36*a45);
  a18=(a2*a18);
  a32=(a32+a18);
  a18=(a30*a32);
  a46=(a46+a18);
  a44=(a44+a46);
  a46=(a44*a40);
  a28=(a2*a28);
  a28=(a28-a36);
  a18=(a28*a38);
  a46=(a46-a18);
  a42=(a42+a46);
  a46=-3.8400000000000001e-01;
  a5=(a0*a5);
  a7=(a1*a7);
  a5=(a5-a7);
  a7=(a46*a5);
  a18=8.4699999999999998e-02;
  a47=(a2*a0);
  a47=(a47+a1);
  a48=(a18*a47);
  a7=(a7+a48);
  a48=-1.0929999999999999e-01;
  a49=(a48*a1);
  a7=(a7-a49);
  a50=1.1530000000000000e-01;
  a51=(a5*a21);
  a13=(a0*a13);
  a1=(a1*a15);
  a13=(a13-a1);
  a1=(a13*a8);
  a15=(a47*a9);
  a1=(a1+a15);
  a51=(a51-a1);
  a1=(a50*a51);
  a15=6.5500000000000003e-02;
  a52=(a2*a13);
  a53=(a2*a47);
  a52=(a52+a53);
  a52=(a52-a5);
  a53=(a15*a52);
  a1=(a1+a53);
  a1=(a7+a1);
  a53=-2.6650000000000001e-01;
  a5=(a5*a10);
  a13=(a13*a9);
  a54=(a47*a8);
  a13=(a13-a54);
  a5=(a5+a13);
  a13=(a5*a19);
  a54=(a51*a23);
  a55=(a52*a25);
  a54=(a54+a55);
  a13=(a13+a54);
  a54=(a53*a13);
  a55=-7.4700000000000003e-02;
  a56=(a2*a5);
  a56=(a56-a51);
  a57=(a55*a56);
  a54=(a54+a57);
  a54=(a1+a54);
  a57=-1.1430000000000000e-01;
  a58=(a13*a35);
  a5=(a5*a29);
  a51=(a51*a31);
  a59=(a52*a16);
  a51=(a51+a59);
  a5=(a5+a51);
  a51=(a5*a24);
  a59=(a56*a26);
  a51=(a51+a59);
  a58=(a58-a51);
  a51=(a57*a58);
  a59=8.3500000000000005e-02;
  a60=(a2*a5);
  a61=(a2*a56);
  a60=(a60+a61);
  a60=(a60-a13);
  a61=(a59*a60);
  a51=(a51+a61);
  a51=(a54+a51);
  a61=-1.6800000000000001e-01;
  a13=(a13*a27);
  a5=(a5*a26);
  a62=(a56*a24);
  a5=(a5-a62);
  a13=(a13+a5);
  a5=(a13*a33);
  a62=(a58*a37);
  a63=(a60*a39);
  a62=(a62+a63);
  a5=(a5+a62);
  a62=(a61*a5);
  a63=6.8699999999999997e-02;
  a64=(a2*a13);
  a64=(a64-a58);
  a65=(a63*a64);
  a62=(a62+a65);
  a62=(a51+a62);
  a65=5.9999999999999998e-02;
  a13=(a13*a43);
  a58=(a58*a45);
  a66=(a60*a32);
  a58=(a58+a66);
  a13=(a13+a58);
  a58=(a2*a13);
  a66=(a2*a64);
  a58=(a58+a66);
  a58=(a58-a5);
  a66=(a65*a58);
  a66=(a62+a66);
  a67=(a42*a66);
  a68=(a5*a41);
  a69=(a13*a40);
  a70=(a64*a38);
  a69=(a69-a70);
  a68=(a68+a69);
  a48=(a48*a0);
  a6=(a46*a6);
  a18=(a18*a17);
  a6=(a6+a18);
  a6=(a48+a6);
  a22=(a50*a22);
  a18=(a15*a14);
  a22=(a22+a18);
  a22=(a6+a22);
  a20=(a53*a20);
  a18=(a55*a12);
  a20=(a20+a18);
  a20=(a22+a20);
  a36=(a57*a36);
  a18=(a59*a30);
  a36=(a36+a18);
  a36=(a20+a36);
  a18=(a61*a34);
  a0=(a63*a28);
  a18=(a18+a0);
  a18=(a36+a18);
  a0=(a2*a44);
  a69=(a2*a28);
  a0=(a0+a69);
  a0=(a0-a34);
  a69=(a65*a0);
  a69=(a18+a69);
  a70=(a68*a69);
  a67=(a67-a70);
  if (res[0]!=0) res[0][0]=a67;
  a11=(a11*a40);
  a67=(a2*a38);
  a11=(a11-a67);
  a34=(a34*a11);
  a44=(a44*a38);
  a67=(a28*a40);
  a44=(a44+a67);
  a34=(a34-a44);
  a44=(a34*a66);
  a5=(a5*a11);
  a13=(a13*a38);
  a67=(a64*a40);
  a13=(a13+a67);
  a5=(a5-a13);
  a13=(a5*a69);
  a44=(a44-a13);
  if (res[0]!=0) res[0][1]=a44;
  a44=(a0*a66);
  a13=(a58*a69);
  a44=(a44-a13);
  if (res[0]!=0) res[0][2]=a44;
  a44=(a2*a3);
  a44=(a44-a4);
  a10=(a44*a10);
  a4=(a2*a4);
  a3=(a3+a4);
  a9=(a3*a9);
  a10=(a10+a9);
  a19=(a10*a19);
  a21=(a44*a21);
  a8=(a3*a8);
  a21=(a21-a8);
  a23=(a21*a23);
  a3=(a2*a3);
  a3=(a3-a44);
  a25=(a3*a25);
  a23=(a23+a25);
  a19=(a19+a23);
  a27=(a19*a27);
  a29=(a10*a29);
  a31=(a21*a31);
  a16=(a3*a16);
  a31=(a31+a16);
  a29=(a29+a31);
  a31=(a29*a26);
  a10=(a2*a10);
  a10=(a10-a21);
  a16=(a10*a24);
  a31=(a31-a16);
  a27=(a27+a31);
  a33=(a27*a33);
  a35=(a19*a35);
  a24=(a29*a24);
  a26=(a10*a26);
  a24=(a24+a26);
  a35=(a35-a24);
  a37=(a35*a37);
  a29=(a2*a29);
  a24=(a2*a10);
  a29=(a29+a24);
  a29=(a29-a19);
  a39=(a29*a39);
  a37=(a37+a39);
  a33=(a33+a37);
  a41=(a33*a41);
  a43=(a27*a43);
  a45=(a35*a45);
  a32=(a29*a32);
  a45=(a45+a32);
  a43=(a43+a45);
  a45=(a43*a40);
  a27=(a2*a27);
  a27=(a27-a35);
  a32=(a27*a38);
  a45=(a45-a32);
  a41=(a41+a45);
  if (res[0]!=0) res[0][3]=a41;
  a11=(a33*a11);
  a38=(a43*a38);
  a40=(a27*a40);
  a38=(a38+a40);
  a11=(a11-a38);
  if (res[0]!=0) res[0][4]=a11;
  a43=(a2*a43);
  a2=(a2*a27);
  a43=(a43+a2);
  a43=(a43-a33);
  if (res[0]!=0) res[0][5]=a43;
  a2=2.9999999999999999e-01;
  a46=(a46*a44);
  a46=(a2+a46);
  a50=(a50*a21);
  a15=(a15*a3);
  a50=(a50+a15);
  a50=(a46+a50);
  a53=(a53*a19);
  a55=(a55*a10);
  a53=(a53+a55);
  a53=(a50+a53);
  a57=(a57*a35);
  a59=(a59*a29);
  a57=(a57+a59);
  a57=(a53+a57);
  a61=(a61*a33);
  a63=(a63*a27);
  a61=(a61+a63);
  a61=(a57+a61);
  a65=(a65*a43);
  a65=(a61+a65);
  a63=(a65*a17);
  a33=(a2*a17);
  a63=(a63-a33);
  a33=(a68*a63);
  a2=(a2*a47);
  a59=(a65*a47);
  a2=(a2-a59);
  a59=(a42*a2);
  a49=(a49*a17);
  a48=(a48*a47);
  a49=(a49+a48);
  a48=(a66*a17);
  a35=(a69*a47);
  a48=(a48-a35);
  a49=(a49+a48);
  a48=(a41*a49);
  a59=(a59-a48);
  a33=(a33+a59);
  if (res[0]!=0) res[0][6]=a33;
  a33=(a5*a63);
  a59=(a34*a2);
  a48=(a11*a49);
  a59=(a59-a48);
  a33=(a33+a59);
  if (res[0]!=0) res[0][7]=a33;
  a63=(a58*a63);
  a2=(a0*a2);
  a49=(a43*a49);
  a2=(a2-a49);
  a63=(a63+a2);
  if (res[0]!=0) res[0][8]=a63;
  a63=(a68*a47);
  a2=(a42*a17);
  a63=(a63+a2);
  if (res[0]!=0) res[0][9]=a63;
  a63=(a5*a47);
  a2=(a34*a17);
  a63=(a63+a2);
  if (res[0]!=0) res[0][10]=a63;
  a47=(a58*a47);
  a17=(a0*a17);
  a47=(a47+a17);
  if (res[0]!=0) res[0][11]=a47;
  a47=(a6*a3);
  a17=(a46*a14);
  a47=(a47-a17);
  a17=(a69*a3);
  a63=(a65*a14);
  a17=(a17-a63);
  a47=(a47-a17);
  a17=(a68*a47);
  a46=(a46*a52);
  a63=(a7*a3);
  a46=(a46-a63);
  a63=(a65*a52);
  a2=(a66*a3);
  a63=(a63-a2);
  a46=(a46-a63);
  a63=(a42*a46);
  a7=(a7*a14);
  a6=(a6*a52);
  a7=(a7-a6);
  a6=(a66*a14);
  a2=(a69*a52);
  a6=(a6-a2);
  a7=(a7-a6);
  a6=(a41*a7);
  a63=(a63+a6);
  a17=(a17+a63);
  if (res[0]!=0) res[0][12]=a17;
  a17=(a5*a47);
  a63=(a34*a46);
  a6=(a11*a7);
  a63=(a63+a6);
  a17=(a17+a63);
  if (res[0]!=0) res[0][13]=a17;
  a47=(a58*a47);
  a46=(a0*a46);
  a7=(a43*a7);
  a46=(a46+a7);
  a47=(a47+a46);
  if (res[0]!=0) res[0][14]=a47;
  a47=(a68*a52);
  a46=(a42*a14);
  a7=(a41*a3);
  a46=(a46+a7);
  a47=(a47+a46);
  if (res[0]!=0) res[0][15]=a47;
  a47=(a5*a52);
  a46=(a34*a14);
  a7=(a11*a3);
  a46=(a46+a7);
  a47=(a47+a46);
  if (res[0]!=0) res[0][16]=a47;
  a52=(a58*a52);
  a14=(a0*a14);
  a3=(a43*a3);
  a14=(a14+a3);
  a52=(a52+a14);
  if (res[0]!=0) res[0][17]=a52;
  a52=(a22*a10);
  a14=(a50*a12);
  a52=(a52-a14);
  a14=(a69*a10);
  a3=(a65*a12);
  a14=(a14-a3);
  a52=(a52-a14);
  a14=(a68*a52);
  a50=(a50*a56);
  a3=(a1*a10);
  a50=(a50-a3);
  a3=(a65*a56);
  a47=(a66*a10);
  a3=(a3-a47);
  a50=(a50-a3);
  a3=(a42*a50);
  a1=(a1*a12);
  a22=(a22*a56);
  a1=(a1-a22);
  a22=(a66*a12);
  a47=(a69*a56);
  a22=(a22-a47);
  a1=(a1-a22);
  a22=(a41*a1);
  a3=(a3+a22);
  a14=(a14+a3);
  if (res[0]!=0) res[0][18]=a14;
  a14=(a5*a52);
  a3=(a34*a50);
  a22=(a11*a1);
  a3=(a3+a22);
  a14=(a14+a3);
  if (res[0]!=0) res[0][19]=a14;
  a52=(a58*a52);
  a50=(a0*a50);
  a1=(a43*a1);
  a50=(a50+a1);
  a52=(a52+a50);
  if (res[0]!=0) res[0][20]=a52;
  a52=(a68*a56);
  a50=(a42*a12);
  a1=(a41*a10);
  a50=(a50+a1);
  a52=(a52+a50);
  if (res[0]!=0) res[0][21]=a52;
  a52=(a5*a56);
  a50=(a34*a12);
  a1=(a11*a10);
  a50=(a50+a1);
  a52=(a52+a50);
  if (res[0]!=0) res[0][22]=a52;
  a56=(a58*a56);
  a12=(a0*a12);
  a10=(a43*a10);
  a12=(a12+a10);
  a56=(a56+a12);
  if (res[0]!=0) res[0][23]=a56;
  a56=(a20*a29);
  a12=(a53*a30);
  a56=(a56-a12);
  a12=(a69*a29);
  a10=(a65*a30);
  a12=(a12-a10);
  a56=(a56-a12);
  a12=(a68*a56);
  a53=(a53*a60);
  a10=(a54*a29);
  a53=(a53-a10);
  a10=(a65*a60);
  a52=(a66*a29);
  a10=(a10-a52);
  a53=(a53-a10);
  a10=(a42*a53);
  a54=(a54*a30);
  a20=(a20*a60);
  a54=(a54-a20);
  a20=(a66*a30);
  a52=(a69*a60);
  a20=(a20-a52);
  a54=(a54-a20);
  a20=(a41*a54);
  a10=(a10+a20);
  a12=(a12+a10);
  if (res[0]!=0) res[0][24]=a12;
  a12=(a5*a56);
  a10=(a34*a53);
  a20=(a11*a54);
  a10=(a10+a20);
  a12=(a12+a10);
  if (res[0]!=0) res[0][25]=a12;
  a56=(a58*a56);
  a53=(a0*a53);
  a54=(a43*a54);
  a53=(a53+a54);
  a56=(a56+a53);
  if (res[0]!=0) res[0][26]=a56;
  a56=(a68*a60);
  a53=(a42*a30);
  a54=(a41*a29);
  a53=(a53+a54);
  a56=(a56+a53);
  if (res[0]!=0) res[0][27]=a56;
  a56=(a5*a60);
  a53=(a34*a30);
  a54=(a11*a29);
  a53=(a53+a54);
  a56=(a56+a53);
  if (res[0]!=0) res[0][28]=a56;
  a60=(a58*a60);
  a30=(a0*a30);
  a29=(a43*a29);
  a30=(a30+a29);
  a60=(a60+a30);
  if (res[0]!=0) res[0][29]=a60;
  a60=(a36*a27);
  a30=(a57*a28);
  a60=(a60-a30);
  a30=(a69*a27);
  a29=(a65*a28);
  a30=(a30-a29);
  a60=(a60-a30);
  a30=(a68*a60);
  a57=(a57*a64);
  a29=(a51*a27);
  a57=(a57-a29);
  a29=(a65*a64);
  a56=(a66*a27);
  a29=(a29-a56);
  a57=(a57-a29);
  a29=(a42*a57);
  a51=(a51*a28);
  a36=(a36*a64);
  a51=(a51-a36);
  a36=(a66*a28);
  a56=(a69*a64);
  a36=(a36-a56);
  a51=(a51-a36);
  a36=(a41*a51);
  a29=(a29+a36);
  a30=(a30+a29);
  if (res[0]!=0) res[0][30]=a30;
  a30=(a5*a60);
  a29=(a34*a57);
  a36=(a11*a51);
  a29=(a29+a36);
  a30=(a30+a29);
  if (res[0]!=0) res[0][31]=a30;
  a60=(a58*a60);
  a57=(a0*a57);
  a51=(a43*a51);
  a57=(a57+a51);
  a60=(a60+a57);
  if (res[0]!=0) res[0][32]=a60;
  a60=(a68*a64);
  a57=(a42*a28);
  a51=(a41*a27);
  a57=(a57+a51);
  a60=(a60+a57);
  if (res[0]!=0) res[0][33]=a60;
  a60=(a5*a64);
  a57=(a34*a28);
  a51=(a11*a27);
  a57=(a57+a51);
  a60=(a60+a57);
  if (res[0]!=0) res[0][34]=a60;
  a64=(a58*a64);
  a28=(a0*a28);
  a27=(a43*a27);
  a28=(a28+a27);
  a64=(a64+a28);
  if (res[0]!=0) res[0][35]=a64;
  a64=(a18*a43);
  a28=(a61*a0);
  a64=(a64-a28);
  a28=(a69*a43);
  a27=(a65*a0);
  a28=(a28-a27);
  a64=(a64-a28);
  a28=(a68*a64);
  a61=(a61*a58);
  a27=(a62*a43);
  a61=(a61-a27);
  a65=(a65*a58);
  a27=(a66*a43);
  a65=(a65-a27);
  a61=(a61-a65);
  a65=(a42*a61);
  a62=(a62*a0);
  a18=(a18*a58);
  a62=(a62-a18);
  a66=(a66*a0);
  a69=(a69*a58);
  a66=(a66-a69);
  a62=(a62-a66);
  a66=(a41*a62);
  a65=(a65+a66);
  a28=(a28+a65);
  if (res[0]!=0) res[0][36]=a28;
  a28=(a5*a64);
  a65=(a34*a61);
  a66=(a11*a62);
  a65=(a65+a66);
  a28=(a28+a65);
  if (res[0]!=0) res[0][37]=a28;
  a64=(a58*a64);
  a61=(a0*a61);
  a62=(a43*a62);
  a61=(a61+a62);
  a64=(a64+a61);
  if (res[0]!=0) res[0][38]=a64;
  a68=(a68*a58);
  a42=(a42*a0);
  a41=(a41*a43);
  a42=(a42+a41);
  a68=(a68+a42);
  if (res[0]!=0) res[0][39]=a68;
  a5=(a5*a58);
  a34=(a34*a0);
  a11=(a11*a43);
  a34=(a34+a11);
  a5=(a5+a34);
  if (res[0]!=0) res[0][40]=a5;
  a58=casadi_sq(a58);
  a0=casadi_sq(a0);
  a43=casadi_sq(a43);
  a0=(a0+a43);
  a58=(a58+a0);
  if (res[0]!=0) res[0][41]=a58;
  return 0;
}

CASADI_SYMBOL_EXPORT int J_b(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int J_b_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int J_b_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void J_b_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int J_b_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void J_b_release(int mem) {
}

CASADI_SYMBOL_EXPORT void J_b_incref(void) {
}

CASADI_SYMBOL_EXPORT void J_b_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int J_b_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int J_b_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real J_b_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* J_b_name_in(casadi_int i) {
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* J_b_name_out(casadi_int i) {
  switch (i) {
    case 0: return "J_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* J_b_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* J_b_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int J_b_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
