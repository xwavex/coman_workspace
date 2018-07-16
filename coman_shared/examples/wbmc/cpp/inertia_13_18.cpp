#include "wbmc_func.h" 
 
 float inertia_13_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
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
 
 out = 0.000033955213987728512411656440362178*c18*(s14*s16 + c14*c16*s15) - 0.000016376033880822019816163783506662*s18*(s14*s16 + c14*c16*s15) + 0.006459466325711463602870241448052*c18*(s17*(0.2258*c14*s15 + 0.201*s14*s16 + 0.201*c14*c16*s15) + 0.2258*c14*c15*c17*s16) - 0.028699600335866720559966143990067*s18*(s17*(0.2258*c14*s15 + 0.201*s14*s16 + 0.201*c14*c16*s15) + 0.2258*c14*c15*c17*s16) + 0.000016376033880822019816163783506662*c18*(c17*(c16*s14 - 1.0*c14*s15*s16) + c14*c15*s17) + 0.000033955213987728512411656440362178*s18*(c17*(c16*s14 - 1.0*c14*s15*s16) + c14*c15*s17) + 0.028699600335866720559966143990067*c18*(0.201*c14*c15 + 0.2258*c14*c15*c16) + 0.006459466325711463602870241448052*s18*(0.201*c14*c15 + 0.2258*c14*c15*c16) - 0.0028836614524289694565534415788701*s17*(c16*s14 - 1.0*c14*s15*s16) + 0.0028836614524289694565534415788701*c14*c15*c17; 
 
 return out; 
 }