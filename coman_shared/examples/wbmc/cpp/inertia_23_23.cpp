#include "wbmc_func.h" 
 
 float inertia_23_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
 float s4 = s(3); 
 float s5 = s(4); 
 float s6 = s(5); 
 float s7 = s(6); 
 float s8 = s(7); 
 float s9 = s(8); 
 float s10 = s(9); 
 float s11 = s(10); 
 float s12 = s(11); 
 float s13 = s(12); 
 float s14 = s(13); 
 float s15 = s(14); 
 float s16 = s(15); 
 float s17 = s(16); 
 float s18 = s(17); 
 float s19 = s(18); 
 float s20 = s(19); 
 float s21 = s(20); 
 float s22 = s(21); 
 float s23 = s(22); 
 float s24 = s(23); 
 float s25 = s(24); 
 float s26 = s(25); 
 float s27 = s(26); 
 float s28 = s(27); 
 float s29 = s(28); 
 float c4 = c(3); 
 float c5 = c(4); 
 float c6 = c(5); 
 float c7 = c(6); 
 float c8 = c(7); 
 float c9 = c(8); 
 float c10 = c(9); 
 float c11 = c(10); 
 float c12 = c(11); 
 float c13 = c(12); 
 float c14 = c(13); 
 float c15 = c(14); 
 float c16 = c(15); 
 float c17 = c(16); 
 float c18 = c(17); 
 float c19 = c(18); 
 float c20 = c(19); 
 float c21 = c(20); 
 float c22 = c(21); 
 float c23 = c(22); 
 float c24 = c(23); 
 float c25 = c(24); 
 float c26 = c(25); 
 float c27 = c(26); 
 float c28 = c(27); 
 float c29 = c(28); 
 float out; 
 
 out = 0.0425*s24*(0.10457619084891840128133540598867*s24 + c25*(0.00058939570649366366365397578102987*s24 + 0.00059169665394978836362911211630224*c24*s25 + 0.11395247939999999742255454293627*c25*s24) - 1.0*s25*(0.00041499848916429991050688688575256*s24 + 0.00059169665394978836362911211630224*c24*c25 - 0.11395247939999999742255454293627*s24*s25)) + s24*(0.0092106371309495243268739272172213*s24 - 0.00008359723848297011914675755752797*c24 + 0.00000065164199210122017919054511636817*c24*c25 + 0.0000025944060117971222222811573813394*c24*s25 + 0.00010609122716885945945771564058538*c25*s24 - 0.000074699728049573983891239639435462*s24*s25 + 0.1375*c25*(0.00058939570649366366365397578102987*s24 + 0.00059169665394978836362911211630224*c24*s25 + 0.11395247939999999742255454293627*c25*s24) - 0.1375*s25*(0.00041499848916429991050688688575256*s24 + 0.00059169665394978836362911211630224*c24*c25 - 0.11395247939999999742255454293627*s24*s25)) + 0.0425*c24*(0.21852867024891839870388994892494*c24 + 0.00058939570649366366365397578102987*c24*c25 - 0.00041499848916429991050688688575256*c24*s25) + c24*(0.024823247998124233967518721819267*c24 - 0.00008359723848297011914675755752797*s24 + 0.000081041909642878753752421669891607*c24*c25 - 0.000057062292260091237694696946790978*c24*s25 + c25*(0.00010609122716885945945771564058538*c24 + 0.00000065164199210122017919054511636817*s24 + 0.00022709251278659486730236644164728*c24*c25 - 0.000010187097442742489031127128148353*c24*s25 - 0.0001065053977109619054532401809344*s24*s25) + s25*(0.0000025944060117971222222811573813394*s24 - 0.000074699728049573983891239639435462*c24 - 0.000010187097442742489031127128148353*c24*c25 + 0.00021961499341858439193512819537217*c24*s25 + 0.0001065053977109619054532401809344*c25*s24)) + 0.00047648972239734937820893685924195; 
 
 return out; 
 }