#include "wbmc_func.h" 
 
 float inertia_2_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
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
 
 out = s9*(0.201*c12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) + 0.26794369706496361237003851619818*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.0016783529345457719870290271073827*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + 0.0034699377093521397805611681755122*s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.12872721938999620977780059935753*s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + 0.1234*c10*(c12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) + 2.1397392100000000025872282094497*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 2.1397392100000000025872282094497*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)) + s12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + 0.1234*s10*(s11*(1.3963881500000000501771069139068*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.3963881500000000501771069139068*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) - c11*(1.0*s12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) - 0.72992131000000004537042741503683*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.72992131000000004537042741503683*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - c12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + 1.4098178999999999572168007944128*s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.4098178999999999572168007944128*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.0034699377093521397805611681755122*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)) + 0.12872721938999620977780059935753*c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7) - 0.26794369706496361237003851619818*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)) - 0.0016783529345457719870290271073827*s8*(c4*c6 - 1.0*s4*s5*s6) + s11*(0.00067316905954114113141677371531288*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + c12*(0.0060288741634372678583132809560112*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.0060288741634372678583132809560112*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) + 0.006459466325711463602870241448052*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.006459466325711463602870241448052*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) + s12*(0.0060288741634372678583132809560112*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.0060288741634372678583132809560112*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) - 0.028699600335866720559966143990067*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 0.028699600335866720559966143990067*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) + 0.0006428218511565811421129092396358*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.0006428218511565811421129092396358*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.00067316905954114113141677371531288*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + c11*(0.028699600335866720559966143990067*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.00016816274582323561604140563759858*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.006459466325711463602870241448052*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.006459466325711463602870241448052*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) - 0.028699600335866720559966143990067*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) + 0.0006428218511565811421129092396358*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.0006428218511565811421129092396358*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.00016816274582323561604140563759858*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.201*s12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + 0.1024*c9*(s11*(1.0*s12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) - 0.72992131000000004537042741503683*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.72992131000000004537042741503683*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - c12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + 1.0*c11*(1.3963881500000000501771069139068*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.3963881500000000501771069139068*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) + 3.1099312999999999540534645348089*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - 3.1099312999999999540534645348089*s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 1.0*c9*(c10*(0.0074860786300290602396147484368072*s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 0.201*c11*(1.3963881500000000501771069139068*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.3963881500000000501771069139068*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) - 0.12122951375496360325058260577578*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - 0.201*s11*(1.0*s12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) - 0.72992131000000004537042741503683*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.72992131000000004537042741503683*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - c12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + c12*(0.0060288741634372678583132809560112*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.0060288741634372678583132809560112*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) - 0.028699600335866720559966143990067*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 0.028699600335866720559966143990067*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) - 1.0*s12*(0.0060288741634372678583132809560112*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.0060288741634372678583132809560112*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) + 0.006459466325711463602870241448052*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.006459466325711463602870241448052*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) + 0.00067316905954114113141677371531288*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.00016816274582323561604140563759858*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.00016816274582323561604140563759858*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.0074860786300290602396147484368072*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)) - 0.00067316905954114113141677371531288*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.12122951375496360325058260577578*s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 0.1234*s11*(1.0*s12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) - 0.72992131000000004537042741503683*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.72992131000000004537042741503683*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - c12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) - 0.1234*c11*(1.3963881500000000501771069139068*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.3963881500000000501771069139068*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) - 0.30269874824999620449835381738808*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - 0.0011397094742551080683066271559816*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + 0.0011397094742551080683066271559816*s8*(c4*c6 - 1.0*s4*s5*s6) + 0.30269874824999620449835381738808*s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7) + 1.0*s10*(1.0*s11*(0.028699600335866720559966143990067*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.00016816274582323561604140563759858*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.006459466325711463602870241448052*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.006459466325711463602870241448052*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) - 0.028699600335866720559966143990067*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) + 0.0006428218511565811421129092396358*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.0006428218511565811421129092396358*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.00016816274582323561604140563759858*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.0074860786300290602396147484368072*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 0.0034699377093521397805611681755122*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - c11*(0.00067316905954114113141677371531288*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + c12*(0.0060288741634372678583132809560112*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.0060288741634372678583132809560112*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) + 0.006459466325711463602870241448052*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.006459466325711463602870241448052*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) + s12*(0.0060288741634372678583132809560112*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.0060288741634372678583132809560112*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) - 0.028699600335866720559966143990067*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 0.028699600335866720559966143990067*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) + 0.0006428218511565811421129092396358*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.0006428218511565811421129092396358*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.00067316905954114113141677371531288*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.0074860786300290602396147484368072*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)) + 0.0034699377093521397805611681755122*s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))) - 0.017312193404557201111736519789375*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + 0.060352952989293605258579673206043*s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + 0.1024*s9*(1.700113399999999996836663740396*s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c10*(c12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) + 2.1397392100000000025872282094497*c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - 2.1397392100000000025872282094497*s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)) + s12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + s10*(s11*(1.3963881500000000501771069139068*c11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.3963881500000000501771069139068*s11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6)))) - c11*(1.0*s12*(0.66646684000000000480667949886993*c12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - 0.66646684000000000480667949886993*s12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)))) - 0.72992131000000004537042741503683*c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.72992131000000004537042741503683*s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - c12*(0.66646684000000000480667949886993*s12*(c10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) - s10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 0.66646684000000000480667949886993*c12*(c11*(s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.0*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) - s11*(1.0*c9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) - s9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7))))) + 1.4098178999999999572168007944128*s10*(s9*(s8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) + c8*(c4*c6 - 1.0*s4*s5*s6)) + c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 1.4098178999999999572168007944128*c10*(1.0*c8*(s7*(c4*s6 + c6*s4*s5) - 1.0*c5*c7*s4) - s8*(c4*c6 - 1.0*s4*s5*s6))) + 1.700113399999999996836663740396*c9*(c7*(c4*s6 + c6*s4*s5) + c5*s4*s7)) + 0.060352952989293605258579673206043*c8*(c4*c6 - 1.0*s4*s5*s6) + 0.017312193404557201111736519789375*s8*(c4*c6 - 1.0*s4*s5*s6); 
 
 return out; 
 }