

static register double s4;
register double s5;
double s6;
double s7;
double s8;
double s9;
double s1;
double s11;
double s12;
double s13;
double s14;
double s15;
double s16;
double s17;
double s18;
double s19;
double s20;
double s21;
double s22;
double s23;
double s24;
double s25;
double s26;
double s27;
double s28;
double s29;
double c4;
double c5;
double c6;
double c7;
double c8;
double c9;
double c10;
double c11;
double c12;
double c13;
double c14;
double c15;
double c16;
double c17;
double c18;
double c19;
double c20;
double c21;
double c22;
double c23;
double c24;
double c25;
double c26;
double c27;
double c28;
double c29;

typedef float (*func_prt)(void);

static func_prt tau_func_ptr[29] = {
    tau1, tau2, tau3, tau4, tau5, tau6, tau7, tau8, tau9,
    tau10, tau11, tau12, tau13, tau14, tau15, tau16, tau17, tau18, tau19,
    tau20, tau21, tau22, tau23, tau24, tau25, tau26, tau27, tau28, tau29
};

static func_prt inertia_func_ptr[23][2] = {
    {inertia1_1, inertia1_2, inertia1_3, inertia1_4, inertia1_5, inertia1_6, inertia1_7, inertia1_8, inertia1_9, 
        inertia1_10, inertia1_11, inertia1_12, inertia1_13, inertia1_14, inertia1_15, inertia1_16, inertia1_17, inertia1_18, inertia1_19, 
        inertia1_20, inertia1_21, inertia1_22, inertia1_23 },

    {inertia2_1, inertia2_2, inertia2_3, inertia2_4, inertia2_5, inertia2_6, inertia2_7, inertia2_8, inertia2_9, 
        inertia2_10, inertia2_11, inertia2_12, inertia2_13, inertia2_14, inertia2_15, inertia2_16, inertia2_17, inertia2_18, inertia2_19, 
        inertia2_20, inertia2_21, inertia2_22, inertia2_23 }

};


/* Control loop */
void control_loop(void) {

    float q[29], qd[29]
        float RFootPos[3], LFootPos[3]
float Vc[29][17], SqV[23][17], SqVpinv[17][23], proj[23][29]
float Vc_eps[29][17], SqV_eps[23][17], SqVpinv_eps[17][23], proj_eps[23][29]

float h[29], M[23][23], J_LMcom[6][29], LMom[3], NormLM, J_AMcom[6][29], AMom[3], NormAM, qd_eps[29], LMom_eps[6], AMom_eps[6], Effort, q_eps[29], h_eps[29], tau_GC_eps[29], Effort_eps, tau_JLimP[29], tau_JLimD[29], RHand_act[3], J_RH[6][29], RHandd_act[3], LHand_act[3], J_LH[6][29], LHandd_act[3]

    /* Joints and base positions and velocities are loaded */
    // q position 
    // q 1..3 compute later with dirDyn
    // q 4..6 imu data
    // q 7..29 mc board
    input q[4:29]
    // qd velocity
    // qd 1..6 imu data
    // qd 7..29 mc board
    input qd[1:29]


    s4 = sin(q[4]);
    s5 = sin(q[5]);
    s6 = sin(q[6]);
    s7 = sin(q[7]);
    s8 = sin(q[8]);
    s9 = sin(q[9]);
    s10 = sin(q[10]);
    s11 = sin(q[11]);
    s12 = sin(q[12]);
    s13 = sin(q[13]);
    s14 = sin(q[14]);
    s15 = sin(q[15]);
    s16 = sin(q[16]);
    s17 = sin(q[17]);
    s18 = sin(q[18]);
    s19 = sin(q[19]);
    s20 = sin(q[20]);
    s21 = sin(q[21]);
    s22 = sin(q[22]);
    s23 = sin(q[23]);
    s24 = sin(q[24]);
    s25 = sin(q[25]);
    s26 = sin(q[26]);
    s27 = sin(q[27]);
    s28 = sin(q[28]);
    s29 = sin(q[29]);
    c4 = cos(q[4]);
    c5 = cos(q[5]);
    c6 = cos(q[6]);
    c7 = cos(q[7]);
    c8 = cos(q[8]);
    c9 = cos(q[9]);
    c10 = cos(q[10]);
    c11 = cos(q[11]);
    c12 = cos(q[12]);
    c13 = cos(q[13]);
    c14 = cos(q[14]);
    c15 = cos(q[15]);
    c16 = cos(q[16]);
    c17 = cos(q[17]);
    c18 = cos(q[18]);
    c19 = cos(q[19]);
    c20 = cos(q[20]);
    c21 = cos(q[21]);
    c22 = cos(q[22]);
    c23 = cos(q[23]);
    c24 = cos(q[24]);
    c25 = cos(q[25]);
    c26 = cos(q[26]);
    c27 = cos(q[27]);
    c28 = cos(q[28]);
    c29 = cos(q[29]);


    q[1:3] = dirDyn(RFootPos, LFootPos) // ,s4 .. c29   

    // tutte matrici
    Vc = V(q)
    SqV = Vc[7:29][1:17]
    //  pseude inversa
    SqVpinv = pinv(SqV )
    proj = SqVpinv' * SqVpinv * SqV * Vc'

    /* Gravity compensation */
    //h[1] = tau1(s4,...,s29,c4,...,c29)
    //...
    //h[29] = tau29(s4,...,s29,c4,...,c29)
    // >>> PARALLEL REGION
    for (int i=1; i<=29; i++ ) {
        h[i] = tau_func_ptr[i](); //(s4,...,s29,c4,...,c29);
    }
    // <<< PARALLEL REGION

    tau_GC[1:6] = 0
    tau_GC[7:29] = proj * h

    /* Mom@Joints */
    //M[1][1] = inertia7_7(s4,...,s29,c4,...,c29)
    //...
    //M[23][23] = inertia29_29(s4,...,s29,c4,...,c29)
    for (int i=1; i<=23; i++ ) {
        for (int j=1; j<=23; j++ ) {
            M[i][j] = inertia_func_ptr[i](); //(s4,...,s29,c4,...,c29);
    }

    tau_MomJ[1:6] = 0
    tau_MomJ[7:29] = k[1] * (qd[7:29])' * M

    /* LMom@COM and AMom@COM */
    J_LMcom = J_lm(s4,...,s29,c4,...,c29)
    LMom = J_LMcom * qd
    NormLM = norm(Ref_LMom - LMom)
    J_AMcom = J_am(s4,...,s29,c4,...,c29)
    AMom = J_AMcom * qd
    NormAM = norm(Ref_AMom - AMom)

    for(i==7, i<=29, i++)
        qd_eps = qd
        qd_eps[i] = qd[i] + epsqd
        // i-esima riga
        LMom_eps[:][i] = J_LMcom * qd_eps
        tau_LMomCOM[i] = k[2] * ((norm(Ref_LMom - LMom_eps[:][i]) - NormLM) / epsqd)
        AMom_eps[:][i] = J_AMcom * qd_eps
        tau_AMomCOM[i] = k[3] * ((norm(Ref_AMom - AMom_eps[:][i]) - NormAM) / epsqd)

    /* MinEff */
    Effort = tau_GC' * tau_GC
    for(i==7, i<=29, i++)
        q_eps = q
        q_eps[i] = q[i] + epsq
        if(i==7||i==10||i==12||i==13||i==16||i==18)
            q_eps[5] = q_eps[5] - epsq
        end
        if(i==8||i==11||i==14||i==17)
            q_eps[4] = q_eps[4] - epsq
        end
        if(i==9||i==15)
            q_eps[6] = q_eps[6] - epsq
        end

        Vc_eps = V(q_eps)
        SqV_eps = Vc_eps[7:29][1:17]
        SqVpinv_eps = pinv(SqV_eps)
        proj_eps = SqVpinv_eps' * SqVpinv_eps * SqV_eps * Vc_eps'
 
        // !!!! ricalcolo di s4 .... c29 
        // s4 = sin(q_esp[4])
        // update_tau_esp() {
        //  s4_eps = sin(q_esp[4])
        // }
        h_eps[1] = tau1(s4,...,s29,c4,...,c29)
        ...
        h_eps[29] = tau1(s4,...,s29,c4,...,c29)

        tau_GC_eps[1:6] = 0
        tau_GC_eps[7:29] = proj_eps * h_eps
        
        Effort_eps = tau_GC_eps' * tau_GC_eps
        tau_MEff[i] = - k[4] * sign(Effort_eps - Effort) * (tau_GC[i])^2
    end

    /* Joint limits */
    for(i==7, i<=29, i++)
        if(q[i]<thNeg)
            tau_JLimP[i] = k[5] / (limNeg[i] - thNeg[i])^2 * (q[i] - thNeg[i])^2
            tau_JLimD[i] = -k[6] * qd[i]
            tau_JLimPD[i] = tau_JLimP + tau_JLimD
            if(tau_JLimPD[i] < 0)
                tau_JLimPD[i] = 0
            end
        end
        if(q[i]>thPos)
            tau_JLimP[i] = -k[5] / (limPos[i] - thPos[i])^2 * (q[i] - thPos[i])^2
            tau_JLimD[i] = -k[6] * qd[i]
            tau_JLimPD[i] = tau_JLimP + tau_JLimD
            if(tau_JLimPD[i] > 0)
                tau_JLimPD[i] = 0
            end
        end
        if((q[i]>=thNeg)&&(q[i]<=thPos))
        	tau_JLimPD[i] = 0
        end
    end
    
    /* Right end-effector */
    if (time >=ti_ree && time<=tf_ree)
        RHand_act = P_RHand(s4,...,s29,c4,...,c29)
        J_RH = J_RHank(s4,...,s29,c4,...,c29)
        RHandd_act = J_RH * qd
    
        if(time ==ti_ree)
            xi_ree = RHand_act[1]
            yi_ree = RHand_act[2]
            zi_ree = RHand_act[3]
        end

        xref_ree = -2 / T_ree^3 * (xf_ree - xi_ree) * ((time - ti_ree) * Ts)^3 + 3 / T_ree^2 * (xf_ree - xi_ree) * ((time - ti_ree) * Ts)^2 + xi_ree;
        yref_ree = -2 / T_ree^3 * (yf_ree - yi_ree) * ((time - ti_ree) * Ts)^3 + 3/T_ree^2 * (yf_ree - 	yi_ree) * ((time - ti_ree) * Ts)^2 + yi_ree;
        zref_ree = -2 / T_ree^3 * (zf_ree - zi_ree) * ((time - ti_ree) * Ts)^3 + 3/T_ree^2 * (zf_ree - 	zi_ree) * ((time - ti_ree) * Ts)^2 + zi_ree;

        xdref_ree = -6 / T_ree^3 * (xf_ree - xi_ree) * ((k - ti_ree/Ts) * Ts)^2 + 6/T_ree^2 * (xf_ree - xi_ree) * ((k - ti_ree/Ts) * Ts);
        ydref_ree = -6 / T_ree^3 * (yf_ree - yi_ree) * ((k - ti_ree/Ts) * Ts)^2 + 6/T_ree^2 * (yf_ree - yi_ree) * ((k - ti_ree/Ts) * Ts);
        zdref_ree = -6 / T_ree^3 * (zf_ree - zi_ree) * ((k - ti_ree/Ts) * Ts)^2 + 6/T_ree^2 * (zf_ree - zi_ree) * ((k - ti_ree/Ts) * Ts);

        F_ree[1] = k[7] * (xref_ree - RHand_act[1]) + k[8] * (xdref_ree - RHandd_act[1])
        F_ree[2] = k[7] * (yref_ree - RHand_act[2]) + k[8] * (ydref_ree - RHandd_act[2])
        F_ree[3] = k[7] * (zref_ree - RHand_act[3]) + k[8] * (zdref_ree - RHandd_act[3])
    end

    if((time>tf_ree)&&(time<=tf2_ree))
        F_ree[1] = k[7] * (xe_ree - RHand_act[1]) + k[8] * (- RHandd_act[1])
        F_ree[2] = k[7] * (ye_ree - RHand_act[2]) + k[8] * (- RHandd_act[2])
        F_ree[3] = k[7] * (ze_ree - RHand_act[3]) + k[8] * (- RHandd_act[3])
    end

    if((time>tf2_ree)&&(time<=tf3_ree))
        F_ree[1] =  (tf3_ree - time) / (tf3_ree - tf2_ree) * (k[7] * (xe_ree - RHand_act[1]) + k[8] * (- RHandd_act[1]))
        F_ree[2] = (tf3_ree - time) / (tf3_ree - tf2_ree) * (k[7] * (ye_ree - RHand_act[2]) + k[8] * (- RHandd_act[2]))
        F_ree[3] = (tf3_ree - time) / (tf3_ree - tf2_ree) * (k[7] * (ze_ree - RHand_act[3]) + k[8] * (- RHandd_act[3]))
    end

    tau_ree = J_RH' * F_ree

    if((time<ti_ree)||(time>tf3_ree))
        tau_ree = zeros(29)
    end

    tau_RH = proj * tau_ree

    /* Left end-effector */
    if (time >=ti_lee && time<=tf_lee)
        LHand_act = P_LHand(s4,...,s29,c4,...,c29)
        J_LH = J_LHank(s4,...,s29,c4,...,c29)
        LHandd_act = J_LH * qd
        
        if(time ==ti_lee)
            xi_lee = LHand_act[1]
            yi_lee = LHand_act[2]
            zi_lee = LHand_act[3]
        end

        xref_lee = -2 / T_lee^3 * (xf_lee - xi_lee) * ((time - ti_lee) * Ts)^3 + 3 / T_lee^2 * (xf_lee - xi_lee) * ((time - ti_lee) * Ts)^2 + xi_lee
        yref_lee = -2 / T_lee^3 * (yf_lee - yi_lee) * ((time - ti_lee) * Ts)^3 + 3/T_lee^2 * (yf_lee - 	yi_lee) * ((time - ti_lee) * Ts)^2 + yi_lee;
        zref_lee = -2 / T_lee^3 * (zf_lee - zi_lee) * ((time - ti_lee) * Ts)^3 + 3/T_lee^2 * (zf_lee - 	zi_lee) * ((time - ti_lee) * Ts)^2 + zi_lee

        xdref_lee = -6 / T_lee^3 * (xf_lee - xi_lee) * ((k - ti_lee/Ts) * Ts)^2 + 6/T_lee^2 * (xf_lee - xi_lee) * ((k - ti_lee/Ts) * Ts);
        ydref_lee = -6 / T_lee^3 * (yf_lee - yi_lee) * ((k - ti_lee/Ts) * Ts)^2 + 6/T_lee^2 * (yf_lee - yi_lee) * ((k - ti_lee/Ts) * Ts);
        zdref_lee = -6 / T_lee^3 * (zf_lee - zi_lee) * ((k - ti_lee/Ts) * Ts)^2 + 6/T_lee^2 * (zf_lee - zi_lee) * ((k - ti_lee/Ts) * Ts);

        F_lee[1] = k[7] * (xref_lee - LHand_act[1]) + k[8] * (xdref_lee - LHandd_act[1])
        F_lee[2] = k[7] * (yref_lee - LHand_act[2]) + k[8] * (ydref_lee - LHandd_act[2])
        F_lee[3] = k[7] * (zref_lee - LHand_act[3]) + k[8] * (zdref_lee - LHandd_act[3])
    end

    if((time>tf_lee)&&(time<=tf2_lee))
        F_lee[1] = k[7] * (xe_lee - LHand_act[1]) + k[8] * (- LHandd_act[1])
        F_lee[2] = k[7] * (ye_lee - LHand_act[2]) + k[8] * (- LHandd_act[2])
        F_lee[3] = k[7] * (ze_lee - LHand_act[3]) + k[8] * (- LHandd_act[3])
    end

    if((time>tf2_lee)&&(time<=tf3_lee))
        F_lee[1] =  (tf3_lee - time) / (tf3_lee - tf2_lee) * (k[7] * (xe_lee - LHand_act[1]) + k[8] * (- LHandd_act[1]))
        F_lee[2] = (tf3_lee - time) / (tf3_lee - tf2_lee) * (k[7] * (ye_lee - LHand_act[2]) + k[8] * (- LHandd_act[2]))
        F_lee[3] = (tf3_lee - time) / (tf3_lee - tf2_lee) * (k[7] * (ze_lee - LHand_act[3]) + k[8] * (- LHandd_act[3]))
    end

    tau_lee = J_LH' * F_lee

    if((time<ti_lee)||(time>tf3_lee))
        tau_lee = zeros(29)
    end

    tau_LH = proj * tau_lee

    tau = tauGC + tau_MomJ + tau_LMomCOM + tau_AMomCOM + tau_MEff + tau_JLimPD + tau_RH + tau_LH

}

