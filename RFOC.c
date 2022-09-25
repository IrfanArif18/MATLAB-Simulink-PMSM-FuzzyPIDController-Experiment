#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME RFOC // s-Function Block Name
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumDiscStates(S, 8);

    if (!ssSetNumInputPorts(S, 1))
        return;
    ssSetInputPortWidth(S, 0, 5);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1))
        return;
    ssSetOutputPortWidth(S, 0, 3);
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE
    | SS_OPTION_DISCRETE_VALUED_OUTPUT));
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, 1e-3);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S) {
    real_T *X0 = ssGetRealDiscStates(S);
    int_T nXStates = ssGetNumDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    int_T i;

    /* Initialize the states to 0.0 */
    for (i = 0; i < nXStates; i++) X0[i] = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    real_T *X = ssGetRealDiscStates(S);

    real_T Va, Vb, Vc;

    Va = X[5];
    Vb = X[6];
    Vc = X[7];
    
    Y[0] = Va;
    Y[1] = Vb;
    Y[2] = Vc;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);

    real_T dt = 1e-3;

    real_T iq, id, theta_e, ia, ib, ic, isalfa, isbeta, isd, isq, we;
    real_T Vsd, Vsq, V_Alfa, V_Beta, Va, Vb, Vc;
    real_T integral_old_q, integral_q, error_q, pi_q;
    real_T integral_old_d, integral_d, error_d, pi_d;
    real_T theta_e_old, wm, iq_old, id_old, iq_new, id_new;
    real_T Kpd, Ki, Kpq;

    // Input Ports
    iq = U(0);
    theta_e = U(1);
    ia = U(2);
    ib = U(3);
    ic = U(4);
    id = 0.00;

    // Transformation Constants
    real_T Constant = 0.816497; // sqrt(2/3)
    real_T Constant2 = 0.866025; // sqrt(3)/2

    // PMSM Parameters
    real_T N = 4;
    real_T psi = 0.121;
    real_T Lsd = 16.61e-3;
    real_T Lsq = 16.22e-3;
    real_T Rs = 0.55;
    real_T Td = 0.01;

    // ABC to AlphaBeta (Clarke)
    isalfa = Constant * (ia - 0.5 * ib - 0.5 * ic);
    isbeta = Constant * Constant2 * (ib - ic);

    // AlphaBeta to dq (Park)
    isd = isalfa * cos(theta_e) + isbeta * sin(theta_e);
    isq = -isalfa * sin(theta_e) + isbeta * cos(theta_e);

    // PI Parameters
    Kpd = (Lsd / Td);
    Kpq = (Lsq / Td);
    Ki = (Rs / Td);

    // PI Iq
    integral_old_q = X[0];
    error_q = iq - isq;
    integral_q = integral_old_q + error_q * dt;
    pi_q = Kpq * error_q + Ki * integral_q;

    // PI Id
    integral_old_d = X[1];
    error_d = id - isd;
    integral_d = integral_old_d + error_d * dt;
    pi_d = Kpd * error_d + Ki * integral_d;

    // Output Vsd and Vsq
    theta_e_old = X[2];
    iq_old = X[3];
    id_old = X[4];
    wm = (theta_e - theta_e_old) / (dt * N);
    iq_new = iq_old + (iq - iq_old) * dt / Td;
    id_new = id_old + (id - id_old) * dt / Td;
    Vsd = pi_d - N * wm * Lsq * iq_new;
    Vsq = pi_q + N * wm * (Lsd * id_new + psi);

    // dq to AlphaBeta (Inverse Park)
    V_Alfa = Vsd * cos(theta_e) - Vsq * sin(theta_e);
    V_Beta = Vsd * sin(theta_e) + Vsq * cos(theta_e);
    
    // AlphaBeta to ABC (Inverse Clarke)
    Va = Constant * V_Alfa;
    Vb = Constant * (-0.5 * V_Alfa + (Constant2)*V_Beta);
    Vc = Constant * (-0.5 * V_Alfa - (Constant2)*V_Beta);

    // State Variables
    X[0] = integral_q;
    X[1] = integral_d;
    X[2] = theta_e;
    X[3] = iq_new;
    X[4] = id_new;
    X[5] = Va;
    X[6] = Vb;
    X[7] = Vc;
}

static void mdlTerminate(SimStruct *S)
{} /* Keep this function empty since no memory is allocated */

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif