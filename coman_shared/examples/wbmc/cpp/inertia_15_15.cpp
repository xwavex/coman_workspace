#include "wbmc_func.h" 
 
 float inertia_15_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) { 
 
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
 
 out = c16*(0.0012896966303509281543384489859745*c16 - 0.00031063912613933249055548809383066*s16 + s17*(0.000008227471987271154009540863537302*s16 + 0.0000035338397077781886056462916043931*c16*c17 + 0.0032501098902887943385665519970068*c16*s17 - 0.000033955213987728512411656440362178*c18*s16 - 0.0001292071920824728095646947571668*s16*s17 + 0.000016376033880822019816163783506662*s16*s18 - 0.000016376033880822019816163783506662*c16*c17*c18 - 0.000033955213987728512411656440362178*c16*c17*s18 - 0.0012983527314680041841769185310585*c18*s16*s17 + 0.0057686196675092108325531949420035*s16*s17*s18) - 1.0*c17*(0.00000010620522209853533430366130845567*s16 - 0.00036183850509007811668861261148573*c16*c17 + c18*(0.000016376033880822019816163783506662*c16*s17 + 0.0012983527314680041841769185310585*c17*s16 + 0.00047896057843341537583402940050324*c18*s16 + 0.0014498197558268702777442704852665*s16*s18 - 0.0014498197558268702777442704852665*c16*c17*c18 + 0.00047896057843341537583402940050324*c16*c17*s18 - 0.0012118037068508908395209694721583*s16*s17*s18) - 0.0000035338397077781886056462916043931*c16*s17 + 0.0001292071920824728095646947571668*c17*s16 - 1.0*s18*(0.0057686196675092108325531949420035*c17*s16 - 0.000033955213987728512411656440362178*c16*s17 + 0.0025848891616778932518883508409438*c18*s16 + 0.00047896057843341537583402940050324*s16*s18 - 0.00047896057843341537583402940050324*c16*c17*c18 + 0.0025848891616778932518883508409438*c16*c17*s18 - 0.0012118037068508908395209694721583*c18*s16*s17))) - 1.0*s16*(0.00031063912613933249055548809383066*c16 - 0.014845618343420970269680735222189*s16 + 0.201*s17*(0.00067316905954114113141677371531288*s16 + 0.0006428218511565811421129092396358*c16*s17 - 0.1467141833100000091194559104224*s16*s17 - 1.0*s18*(0.028699600335866720559966143990067*c16*s17 - 0.0060288741634372678583132809560112*s16*s18 + 0.0060288741634372678583132809560112*c16*c17*c18 + 0.13395983484000000096614257927286*s16*s17*s18) + c18*(0.006459466325711463602870241448052*c16*s17 + 0.0060288741634372678583132809560112*c18*s16 + 0.0060288741634372678583132809560112*c16*c17*s18 - 0.13395983484000000096614257927286*c18*s16*s17)) + 0.00000010620522209853533430366130845567*c16*c17 - 0.000008227471987271154009540863537302*c16*s17 + 0.000033800711910470358824322533157314*c17*s16 - 1.0*s18*(0.000016376033880822019816163783506662*c16*s17 + 0.0012983527314680041841769185310585*c17*s16 + 0.00047896057843341537583402940050324*c18*s16 + 0.0014498197558268702777442704852665*s16*s18 - 0.0014498197558268702777442704852665*c16*c17*c18 + 0.00047896057843341537583402940050324*c16*c17*s18 - 0.0012118037068508908395209694721583*s16*s17*s18) - 1.0*c18*(0.0057686196675092108325531949420035*c17*s16 - 0.000033955213987728512411656440362178*c16*s17 + 0.0025848891616778932518883508409438*c18*s16 + 0.00047896057843341537583402940050324*s16*s18 - 0.00047896057843341537583402940050324*c16*c17*c18 + 0.0025848891616778932518883508409438*c16*c17*s18 - 0.0012118037068508908395209694721583*c18*s16*s17) + 0.00013530698096776936741477151677789*s16*s17 - 0.201*c17*(0.28067401815000001008559848969526*c17*s16 - 0.0006428218511565811421129092396358*c16*c17 - 0.00016816274582323561604140563759858*s16 + 0.028699600335866720559966143990067*c18*s16 + 0.006459466325711463602870241448052*s16*s18 - 0.006459466325711463602870241448052*c16*c17*c18 + 0.028699600335866720559966143990067*c16*c17*s18)) + 0.00090460801101672354279348536431477; 
 
 return out; 
 }