#include "wbmc_func.h" 
 
 float inertia_6_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
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
 
 out = 0.0000054530187303678999148927651048564*s23*(c21*s19 + c19*s20*s21) - 0.00000023444970484002988571617039573524*c23*(c21*s19 + c19*s20*s21) + 0.0000017107633167905518821746068831764*c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0037766736756507600427650475134555*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) + 0.0037766736756507600427650475134555*s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + 0.0000017107633167905518821746068831764*s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - 0.0425*s24*(0.007437810626751680024242785364379*s23*(c21*s19 + c19*s20*s21) + c25*(0.63306932999999998568085857186816*c25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00059169665394978836362911211630224*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00058939570649366366365397578102987*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00059169665394978836362911211630224*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.63306932999999998568085857186816*s25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) + 0.00058939570649366366365397578102987*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + s25*(0.00059169665394978836362911211630224*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.63306932999999998568085857186816*s25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.00041499848916429991050688688575256*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00059169665394978836362911211630224*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) - 0.63306932999999998568085857186816*c25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) - 0.00041499848916429991050688688575256*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 1.0462304000000000048231640903396*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.060111398848918401076350932149239*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) - 0.007437810626751680024242785364379*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + 1.0462304000000000048231640903396*c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.060111398848918401076350932149239*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.0000054530187303678999148927651048564*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) - 1.0*s24*(0.00095397766331616627258039026033996*s23*(c21*s19 + c19*s20*s21) - 0.00000065164199210122017919054511636817*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.1375*c25*(0.63306932999999998568085857186816*c25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00059169665394978836362911211630224*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00058939570649366366365397578102987*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00059169665394978836362911211630224*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.63306932999999998568085857186816*s25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) + 0.00058939570649366366365397578102987*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.00058939570649366366365397578102987*c25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.1375*s25*(0.00059169665394978836362911211630224*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.63306932999999998568085857186816*s25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.00041499848916429991050688688575256*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00059169665394978836362911211630224*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) - 0.63306932999999998568085857186816*c25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) - 0.00041499848916429991050688688575256*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.0023764300152675201496816137303369*c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0000025944060117971222222811573813394*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.060111398848918401076350932149239*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.00041499848916429991050688688575256*s25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.00008359723848297011914675755752797*c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) - 0.0023764300152675201496816137303369*s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - 0.0066559026798704922811290126008786*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.0000025944060117971222222811573813394*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) - 0.00095397766331616627258039026033996*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) - 0.00000065164199210122017919054511636817*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.00041499848916429991050688688575256*c25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) + 0.060111398848918401076350932149239*c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.00058939570649366366365397578102987*s25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) + 0.0066559026798704922811290126008786*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.00008359723848297011914675755752797*s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00000023444970484002988571617039573524*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + 0.00047648972239734937650502439587826*c22*(s19*s21 - 1.0*c19*c21*s20) + c24*(0.000081041909642878753752421669891607*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00043876122171822873615274414215082*s23*(c21*s19 + c19*s20*s21) - 0.14715843172391839910746898578111*c24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 1.0*s25*(0.000010187097442742489031127128148353*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00041499848916429991050688688575256*c24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) + 0.00059169665394978836362911211630224*c25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00021961499341858439193512819537217*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.000057062292260091237694696946790978*c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) - 0.0000025944060117971222222811573813394*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00021961499341858439193512819537217*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.000010187097442742489031127128148353*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.00059169665394978836362911211630224*s25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) + 0.00041499848916429991050688688575256*s24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.0000025944060117971222222811573813394*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.000057062292260091237694696946790978*s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.007437810626751680024242785364379*c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.000057062292260091237694696946790978*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.01856901464985770200545128992357*c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) - 0.007437810626751680024242785364379*s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - 0.00008359723848297011914675755752797*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.000057062292260091237694696946790978*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.00043876122171822873615274414215082*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + c25*(0.00022709251278659486730236644164728*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00058939570649366366365397578102987*c24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.000010187097442742489031127128148353*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.00059169665394978836362911211630224*s25*(1.0*s24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.1375*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + c24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.1375*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.000081041909642878753752421669891607*c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00000065164199210122017919054511636817*s24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.000010187097442742489031127128148353*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.00022709251278659486730236644164728*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) - 0.00059169665394978836362911211630224*c25*(c23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) + s23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20)) + 0.00058939570649366366365397578102987*s24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) - 0.00000065164199210122017919054511636817*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.000081041909642878753752421669891607*s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.000081041909642878753752421669891607*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.14715843172391839910746898578111*s24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.00008359723848297011914675755752797*c24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.01856901464985770200545128992357*s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.0425*c24*(0.00058939570649366366365397578102987*c25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.0023764300152675201496816137303369*s23*(c21*s19 + c19*s20*s21) - 1.6792997299999999905040226622077*c24*(1.0*c23*(0.015*c19*c20 + 0.1554*s19*s21 - 1.0*c21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + s21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) - 0.1554*c19*c21*s20) - s23*(c22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*c22*(s19*s21 - 1.0*c19*c21*s20) + s22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) + 0.071*c19*c20*s22) - 0.0425*c22*(s19*s21 - 1.0*c19*c21*s20) + 0.0425*c19*c20*s22) - 0.00041499848916429991050688688575256*s25*(c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) + 0.14715843172391839910746898578111*c24*(c22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c19*c20*s22) + 0.00041499848916429991050688688575256*c25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 0.0023764300152675201496816137303369*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + 0.00058939570649366366365397578102987*s25*(s23*(c21*s19 + c19*s20*s21) - 1.0*c23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22)) + 1.6792997299999999905040226622077*s24*(0.0425*c23*(c21*s19 + c19*s20*s21) + 0.0425*s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22) + s22*(0.015*c21*s19 - 0.020281503999999998810732648735211*c20*s19 - 0.0825*s19*s21 + 0.0825*c19*c21*s20 + 0.015*c19*s20*s21) - 0.071*s22*(s19*s21 - 1.0*c19*c21*s20) - 1.0*c22*(0.0825*c19*c20 + 0.1554*c21*s19 + s21*(0.020281503999999998810732648735211*c19 + 0.0475*c19*s20) + c21*(0.0475*s19 + 0.020281503999999998810732648735211*s19*s20) + 0.1554*c19*s20*s21) - 0.071*c19*c20*c22) + 0.14715843172391839910746898578111*s24*(c23*(c21*s19 + c19*s20*s21) + s23*(s22*(s19*s21 - 1.0*c19*c21*s20) + c19*c20*c22))) - 0.00047648972239734937650502439587826*c19*c20*s22; 
 
 return out; 
 }