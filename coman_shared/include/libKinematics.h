void InvKHIP(float HipP[3], float HipO[3], float LeftFootP[3], float LFtO[4], float RightFootP[3], float RFtO[4], float *traj);
void ArmIKTrajectory(float BodyP[3], float BodyO[3],float LeftHandP[3],float yawArmL, float RightHandP[3],float yawArmR,float *traj);
void ArmFWTrajectory(float q[], float armL[6], float armR[6]);
void LegFWTrajectory(float q[], float legL[6], float legR[6]);

