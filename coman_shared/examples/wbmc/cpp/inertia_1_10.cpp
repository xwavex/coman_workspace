#include "wbmc_func.h" 
 
 float inertia_1_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
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
 
 out = 0.26794369706496361237003851619818*s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8) - 0.26794369706496361237003851619818*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - 0.0034699377093521397805611681755122*s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - 0.201*s12*(0.66646684000000000480667949886993*s12*(1.0*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 1.0*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7)))) - 0.0034699377093521397805611681755122*c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8) - s11*(0.0006428218511565811421129092396358*s11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) + 0.00067316905954114113141677371531288*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c12*(0.006459466325711463602870241448052*s11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) + 0.0060288741634372678583132809560112*c12*(1.0*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) + 0.006459466325711463602870241448052*c11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7)) - 0.0060288741634372678583132809560112*s12*(c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 1.0*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7)))) + 0.0006428218511565811421129092396358*c11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7)) - s12*(0.028699600335866720559966143990067*s11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 0.0060288741634372678583132809560112*s12*(1.0*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 0.0060288741634372678583132809560112*c12*(c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 1.0*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7))) + 0.028699600335866720559966143990067*c11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7))) - 0.00067316905954114113141677371531288*s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - c11*(0.0006428218511565811421129092396358*c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 0.00016816274582323561604140563759858*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + 0.028699600335866720559966143990067*c12*(1.0*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) + 0.006459466325711463602870241448052*s12*(1.0*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) + 0.006459466325711463602870241448052*c12*(c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 1.0*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7))) - 0.028699600335866720559966143990067*s12*(c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 1.0*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7))) - 0.0006428218511565811421129092396358*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7)) + 0.00016816274582323561604140563759858*s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 0.201*c12*(0.66646684000000000480667949886993*c12*(1.0*c10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) - s10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) - 1.0*c9*(s5*s7 - 1.0*c5*c6*c7)) + c10*(c8*(c7*s5 + c5*c6*s7) + c5*s6*s8)) - 1.0*s11*(c9*(s8*(c7*s5 + c5*c6*s7) - 1.0*c5*c8*s6) + s9*(s5*s7 - 1.0*c5*c6*c7)))); 
 
 return out; 
 }