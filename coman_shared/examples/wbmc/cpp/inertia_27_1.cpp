#include "wbmc_func.h" 
 
 float inertia_27_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
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
 
 out = s28*(0.00058939570649366366365397578102987*c29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) - 0.00041499848916429991050688688575256*s29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.060111398848918401076350932149239*s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + 0.1375*c29*(0.63306932999999998568085857186816*c29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.63306932999999998568085857186816*s29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))))) + 0.00041499848916429991050688688575256*c29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 0.0023764300152675201496816137303369*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + 0.00058939570649366366365397578102987*s29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + 0.0023764300152675201496816137303369*c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))) + 0.1375*s29*(0.63306932999999998568085857186816*s29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) - 0.63306932999999998568085857186816*c29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))))) - 0.060111398848918401076350932149239*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.0425*c28*(1.6792997299999999905040226622077*c28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + 1.6792997299999999905040226622077*s28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.0037766736756507600427650475134555*c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + 0.0000017107633167905518821746068831764*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + 0.0425*s28*(1.0462304000000000048231640903396*s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + c29*(0.63306932999999998568085857186816*c29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.63306932999999998568085857186816*s29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))))) + s29*(0.63306932999999998568085857186816*s29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) - 0.63306932999999998568085857186816*c29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))))) - 1.0462304000000000048231640903396*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) - 0.0000017107633167905518821746068831764*c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))) + 0.0037766736756507600427650475134555*s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))) + c28*(0.14715843172391839910746898578111*c28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*s29*(0.00059169665394978836362911211630224*c29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.00041499848916429991050688688575256*c28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + 0.00059169665394978836362911211630224*s29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + 0.00041499848916429991050688688575256*s28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) - 0.007437810626751680024242785364379*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + c29*(0.00059169665394978836362911211630224*s29*(s28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 1.0*c28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.00058939570649366366365397578102987*c28*(c27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) + s27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) - 0.00059169665394978836362911211630224*c29*(1.0*s27*(s21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) + c21*(s19*s5 - 1.0*c19*c5*s6)) - c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)))) + 0.00058939570649366366365397578102987*s28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))) + 0.007437810626751680024242785364379*c27*(c26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20) - s26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6))) + 0.14715843172391839910746898578111*s28*(1.0*c26*(1.0*c21*(s20*(c19*s5 + c5*s19*s6) - 1.0*c20*c5*c6) - s21*(s19*s5 - 1.0*c19*c5*s6)) + s26*(c20*(c19*s5 + c5*s19*s6) + c5*c6*s20))); 
 
 return out; 
 }