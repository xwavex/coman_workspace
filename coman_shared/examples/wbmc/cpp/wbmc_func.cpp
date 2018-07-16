#include "wbmc_func.h"

static float spin(void) {
    return Eigen::Matrix<float,1,10>::Random() * Eigen::Matrix<float,10,1>::Random();
}


#if 0

#if 1
void V(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c, Eigen::Matrix<float,29,17>& res) {
    res = s * c.transpose().head(17);
}

void J_lm(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c, Eigen::Matrix<float,3,29>& res) {
    res.Random();
}

void J_am(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c, Eigen::Matrix<float,3,29>& res) {
    res.Random();
}

void J_RHand(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c, Eigen::Matrix<float,3,29>& res) {
    res.Random();
}

void J_LHand(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c, Eigen::Matrix<float,3,29>& res) {
    res.Random();
}

#endif


#if 0
Eigen::Vector3f P_RHand(Eigen::Vector3f & q, Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
    return q;
}

Eigen::Vector3f P_LHand(Eigen::Vector3f & q, Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
    return q;
}
#endif

//float tau_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
//        return spin();
//}

float tau_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_24(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_25(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_26(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_27(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_28(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float tau_29(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}


//float inertia_1_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
//        return spin();
//}

float inertia_1_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_1_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_2_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_3_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_4_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_5_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_6_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_7_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_8_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_9_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_10_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_11_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_12_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_13_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_14_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_15_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_16_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_17_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_18_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_19_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_20_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_21_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_22_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_1(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_2(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_3(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_4(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_5(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_6(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_7(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_8(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_9(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_10(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_11(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_12(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_13(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_14(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_15(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_16(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_17(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_18(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_19(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_20(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_21(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_22(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

float inertia_23_23(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}

#endif

func_prt tau_func_ptr[29] = {
tau_1, tau_2, tau_3, tau_4, tau_5, tau_6, tau_7, tau_8, tau_9, tau_10, tau_11, tau_12, tau_13, tau_14, tau_15, tau_16, tau_17, tau_18, tau_19, tau_20, tau_21, tau_22, tau_23, tau_24, tau_25, tau_26, tau_27, tau_28, tau_29, };


func_prt inertia_func_ptr[23][23] = {
inertia_1_1, inertia_1_2, inertia_1_3, inertia_1_4, inertia_1_5, inertia_1_6, inertia_1_7, inertia_1_8, inertia_1_9, inertia_1_10, inertia_1_11, inertia_1_12, inertia_1_13, inertia_1_14, inertia_1_15, inertia_1_16, inertia_1_17, inertia_1_18, inertia_1_19, inertia_1_20, inertia_1_21, inertia_1_22, inertia_1_23, inertia_2_1, inertia_2_2, inertia_2_3, inertia_2_4, inertia_2_5, inertia_2_6, inertia_2_7, inertia_2_8, inertia_2_9, inertia_2_10, inertia_2_11, inertia_2_12, inertia_2_13, inertia_2_14, inertia_2_15, inertia_2_16, inertia_2_17, inertia_2_18, inertia_2_19, inertia_2_20, inertia_2_21, inertia_2_22, inertia_2_23, inertia_3_1, inertia_3_2, inertia_3_3, inertia_3_4, inertia_3_5, inertia_3_6, inertia_3_7, inertia_3_8, inertia_3_9, inertia_3_10, inertia_3_11, inertia_3_12, inertia_3_13, inertia_3_14, inertia_3_15, inertia_3_16, inertia_3_17, inertia_3_18, inertia_3_19, inertia_3_20, inertia_3_21, inertia_3_22, inertia_3_23, inertia_4_1, inertia_4_2, inertia_4_3, inertia_4_4, inertia_4_5, inertia_4_6, inertia_4_7, inertia_4_8, inertia_4_9, inertia_4_10, inertia_4_11, inertia_4_12, inertia_4_13, inertia_4_14, inertia_4_15, inertia_4_16, inertia_4_17, inertia_4_18, inertia_4_19, inertia_4_20, inertia_4_21, inertia_4_22, inertia_4_23, inertia_5_1, inertia_5_2, inertia_5_3, inertia_5_4, inertia_5_5, inertia_5_6, inertia_5_7, inertia_5_8, inertia_5_9, inertia_5_10, inertia_5_11, inertia_5_12, inertia_5_13, inertia_5_14, inertia_5_15, inertia_5_16, inertia_5_17, inertia_5_18, inertia_5_19, inertia_5_20, inertia_5_21, inertia_5_22, inertia_5_23, inertia_6_1, inertia_6_2, inertia_6_3, inertia_6_4, inertia_6_5, inertia_6_6, inertia_6_7, inertia_6_8, inertia_6_9, inertia_6_10, inertia_6_11, inertia_6_12, inertia_6_13, inertia_6_14, inertia_6_15, inertia_6_16, inertia_6_17, inertia_6_18, inertia_6_19, inertia_6_20, inertia_6_21, inertia_6_22, inertia_6_23, inertia_7_1, inertia_7_2, inertia_7_3, inertia_7_4, inertia_7_5, inertia_7_6, inertia_7_7, inertia_7_8, inertia_7_9, inertia_7_10, inertia_7_11, inertia_7_12, inertia_7_13, inertia_7_14, inertia_7_15, inertia_7_16, inertia_7_17, inertia_7_18, inertia_7_19, inertia_7_20, inertia_7_21, inertia_7_22, inertia_7_23, inertia_8_1, inertia_8_2, inertia_8_3, inertia_8_4, inertia_8_5, inertia_8_6, inertia_8_7, inertia_8_8, inertia_8_9, inertia_8_10, inertia_8_11, inertia_8_12, inertia_8_13, inertia_8_14, inertia_8_15, inertia_8_16, inertia_8_17, inertia_8_18, inertia_8_19, inertia_8_20, inertia_8_21, inertia_8_22, inertia_8_23, inertia_9_1, inertia_9_2, inertia_9_3, inertia_9_4, inertia_9_5, inertia_9_6, inertia_9_7, inertia_9_8, inertia_9_9, inertia_9_10, inertia_9_11, inertia_9_12, inertia_9_13, inertia_9_14, inertia_9_15, inertia_9_16, inertia_9_17, inertia_9_18, inertia_9_19, inertia_9_20, inertia_9_21, inertia_9_22, inertia_9_23, inertia_10_1, inertia_10_2, inertia_10_3, inertia_10_4, inertia_10_5, inertia_10_6, inertia_10_7, inertia_10_8, inertia_10_9, inertia_10_10, inertia_10_11, inertia_10_12, inertia_10_13, inertia_10_14, inertia_10_15, inertia_10_16, inertia_10_17, inertia_10_18, inertia_10_19, inertia_10_20, inertia_10_21, inertia_10_22, inertia_10_23, inertia_11_1, inertia_11_2, inertia_11_3, inertia_11_4, inertia_11_5, inertia_11_6, inertia_11_7, inertia_11_8, inertia_11_9, inertia_11_10, inertia_11_11, inertia_11_12, inertia_11_13, inertia_11_14, inertia_11_15, inertia_11_16, inertia_11_17, inertia_11_18, inertia_11_19, inertia_11_20, inertia_11_21, inertia_11_22, inertia_11_23, inertia_12_1, inertia_12_2, inertia_12_3, inertia_12_4, inertia_12_5, inertia_12_6, inertia_12_7, inertia_12_8, inertia_12_9, inertia_12_10, inertia_12_11, inertia_12_12, inertia_12_13, inertia_12_14, inertia_12_15, inertia_12_16, inertia_12_17, inertia_12_18, inertia_12_19, inertia_12_20, inertia_12_21, inertia_12_22, inertia_12_23, inertia_13_1, inertia_13_2, inertia_13_3, inertia_13_4, inertia_13_5, inertia_13_6, inertia_13_7, inertia_13_8, inertia_13_9, inertia_13_10, inertia_13_11, inertia_13_12, inertia_13_13, inertia_13_14, inertia_13_15, inertia_13_16, inertia_13_17, inertia_13_18, inertia_13_19, inertia_13_20, inertia_13_21, inertia_13_22, inertia_13_23, inertia_14_1, inertia_14_2, inertia_14_3, inertia_14_4, inertia_14_5, inertia_14_6, inertia_14_7, inertia_14_8, inertia_14_9, inertia_14_10, inertia_14_11, inertia_14_12, inertia_14_13, inertia_14_14, inertia_14_15, inertia_14_16, inertia_14_17, inertia_14_18, inertia_14_19, inertia_14_20, inertia_14_21, inertia_14_22, inertia_14_23, inertia_15_1, inertia_15_2, inertia_15_3, inertia_15_4, inertia_15_5, inertia_15_6, inertia_15_7, inertia_15_8, inertia_15_9, inertia_15_10, inertia_15_11, inertia_15_12, inertia_15_13, inertia_15_14, inertia_15_15, inertia_15_16, inertia_15_17, inertia_15_18, inertia_15_19, inertia_15_20, inertia_15_21, inertia_15_22, inertia_15_23, inertia_16_1, inertia_16_2, inertia_16_3, inertia_16_4, inertia_16_5, inertia_16_6, inertia_16_7, inertia_16_8, inertia_16_9, inertia_16_10, inertia_16_11, inertia_16_12, inertia_16_13, inertia_16_14, inertia_16_15, inertia_16_16, inertia_16_17, inertia_16_18, inertia_16_19, inertia_16_20, inertia_16_21, inertia_16_22, inertia_16_23, inertia_17_1, inertia_17_2, inertia_17_3, inertia_17_4, inertia_17_5, inertia_17_6, inertia_17_7, inertia_17_8, inertia_17_9, inertia_17_10, inertia_17_11, inertia_17_12, inertia_17_13, inertia_17_14, inertia_17_15, inertia_17_16, inertia_17_17, inertia_17_18, inertia_17_19, inertia_17_20, inertia_17_21, inertia_17_22, inertia_17_23, inertia_18_1, inertia_18_2, inertia_18_3, inertia_18_4, inertia_18_5, inertia_18_6, inertia_18_7, inertia_18_8, inertia_18_9, inertia_18_10, inertia_18_11, inertia_18_12, inertia_18_13, inertia_18_14, inertia_18_15, inertia_18_16, inertia_18_17, inertia_18_18, inertia_18_19, inertia_18_20, inertia_18_21, inertia_18_22, inertia_18_23, inertia_19_1, inertia_19_2, inertia_19_3, inertia_19_4, inertia_19_5, inertia_19_6, inertia_19_7, inertia_19_8, inertia_19_9, inertia_19_10, inertia_19_11, inertia_19_12, inertia_19_13, inertia_19_14, inertia_19_15, inertia_19_16, inertia_19_17, inertia_19_18, inertia_19_19, inertia_19_20, inertia_19_21, inertia_19_22, inertia_19_23, inertia_20_1, inertia_20_2, inertia_20_3, inertia_20_4, inertia_20_5, inertia_20_6, inertia_20_7, inertia_20_8, inertia_20_9, inertia_20_10, inertia_20_11, inertia_20_12, inertia_20_13, inertia_20_14, inertia_20_15, inertia_20_16, inertia_20_17, inertia_20_18, inertia_20_19, inertia_20_20, inertia_20_21, inertia_20_22, inertia_20_23, inertia_21_1, inertia_21_2, inertia_21_3, inertia_21_4, inertia_21_5, inertia_21_6, inertia_21_7, inertia_21_8, inertia_21_9, inertia_21_10, inertia_21_11, inertia_21_12, inertia_21_13, inertia_21_14, inertia_21_15, inertia_21_16, inertia_21_17, inertia_21_18, inertia_21_19, inertia_21_20, inertia_21_21, inertia_21_22, inertia_21_23, inertia_22_1, inertia_22_2, inertia_22_3, inertia_22_4, inertia_22_5, inertia_22_6, inertia_22_7, inertia_22_8, inertia_22_9, inertia_22_10, inertia_22_11, inertia_22_12, inertia_22_13, inertia_22_14, inertia_22_15, inertia_22_16, inertia_22_17, inertia_22_18, inertia_22_19, inertia_22_20, inertia_22_21, inertia_22_22, inertia_22_23, inertia_23_1, inertia_23_2, inertia_23_3, inertia_23_4, inertia_23_5, inertia_23_6, inertia_23_7, inertia_23_8, inertia_23_9, inertia_23_10, inertia_23_11, inertia_23_12, inertia_23_13, inertia_23_14, inertia_23_15, inertia_23_16, inertia_23_17, inertia_23_18, inertia_23_19, inertia_23_20, inertia_23_21, inertia_23_22, inertia_23_23, };
