#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME PMSM_Plant // s-Function Block Name
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumContStates(S, 4);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 4); // Number of Inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 7); // Number of Outputs
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S) {

    real_T *X0 = ssGetContStates(S);
    int_T nStates = ssGetNumContStates(S);
    int_T i;

    /* Initialize the states to 0.0 */
    for (i = 0; i < nStates; i++) X0[i] = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    real_T *X = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);

    real_T Wm, Theta_e, Isd, Isq, Te, Ia, Ib, Ic, Vsd, Vsq;
    real_T I_Alfa, I_Beta;

    // Transformation Constants
    real_T Constant = 0.816497;
    real_T Constant2 = 0.866025;

    // PMSM Parameters
    real_T N = 4;
    
    // State Variables
    Wm = X[0];
    Theta_e = X[1];
    Isq = X[2];
    Isd = X[3];
    Te = X[4];
    
    // dq to AlphaBeta (Inverse Park)
    I_Alfa = Isd * cos(Theta_e) - Isq * sin(Theta_e);
    I_Beta = Isd * sin(Theta_e) + Isq * cos(Theta_e);
    
    // AlphaBeta to ABC (Inverse Clarke)
    Ia = Constant * I_Alfa;
    Ib = Constant * (-0.5 * I_Alfa + (Constant2) * I_Beta);
    Ic = Constant * (-0.5 * I_Alfa - (Constant2) * I_Beta);

    Y[0] = Ia;
    Y[1] = Ib;
    Y[2] = Ic;
    Y[3] = Wm;
    Y[4] = Te;
    Y[5] = Theta_e / N;
    Y[6] = Theta_e;
}

#define MDL_DERIVATIVES
static void mdlDerivatives(SimStruct *S) {
    real_T *dX = ssGetdX(S);
    real_T *X = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    
    real_T Va, Vb, Vc, Vsalfa, Vsbeta, Vsd, Vsq;
    real_T Te, Tl, Wm, Wm_dot, Isd_dot, Isq_dot;
    real_T Theta_e, Theta_e_dot, Isd, Isq;
    real_T Ia, Ib, Ic, I_Alfa, I_Beta;

    // Transformation Constants
    real_T Constant = 0.816497; // sqrt(2/3)
    real_T Constant2 = 0.866025; // sqrt(3)/2

    // PMSM Parameters
    real_T N = 4;
    real_T psi = 0.121;
    real_T Lsd = 16.61e-3;
    real_T Lsq = 16.22e-3;
    real_T Rs = 0.55;
    real_T J = 0.01;

    // Input Ports
    Tl = U(0);
    Va = U(1);
    Vb = U(2);
    Vc = U(3);
    
    // State Variables
    Wm = X[0];
    Theta_e = X[1];
    Isd = X[2];
    Isq = X[3];
    
    // ABC to AlphaBeta (Clarke)
    Vsalfa = Constant * (Va - 0.5 * Vb - 0.5 * Vc);
    Vsbeta = Constant * Constant2 * (Vb - Vc);

    // AlphaBeta to dq (Park)
    Vsd = Vsalfa * cos(Theta_e) + Vsbeta * sin(Theta_e);
    Vsq = -Vsalfa * sin(Theta_e) + Vsbeta * cos(Theta_e);
    
    // PMSM dq Model
    Te = N * (psi + (Lsd - Lsq) * Isd) * Isq;
    Wm_dot = (Te - Tl) / J;
    Theta_e_dot = N * Wm;
    Isd_dot = (Vsd - Rs * Isd + N * Wm * Lsq * Isq) / Lsd;
    Isq_dot = (Vsq - Rs * Isq - N * Wm * (Lsd * Isd + psi)) / Lsq;
    
    // State Variable Derivatives
    dX[0] = Wm_dot;
    dX[1] = Theta_e_dot;
    dX[2] = Isd_dot;
    dX[3] = Isq_dot;
    
    X[4] = Te;
}

static void mdlTerminate(SimStruct *S) 
{} /* Keep this function empty since no memory is allocated */

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif