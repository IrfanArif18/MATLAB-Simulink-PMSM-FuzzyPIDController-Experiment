#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME Speed_Controller_PID // s-Function Block Name
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumDiscStates(S, 3);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 5); // Number of Inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3); // Number of Outputs
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
    for(i = 0; i < nXStates; i++) X0[i] = 0.0;
} 

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    real_T *X = ssGetRealDiscStates(S);

    Y[0] = X[2]; // Control Signal
    Y[1] = X[3]; // Error
    Y[2] = X[4]; // Change of Error
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);

    real_T dt = 1e-3;

    real_T Kp, Ki, Kd, pid;
    real_T W_ref, W_act, error, theta_m, theta_old;
    real_T integral_error, integral_error_old;
    real_T derivative_error, error_old;

    // Input Ports
    W_ref = U(0);
    theta_m = U(1);
    Kp = U(2);
    Ki = U(3);
    Kd = U(4);
    
    // PID Controller
    integral_error_old = X[0];
    theta_old = X[1];
    error_old = X[3];

    W_act = (theta_m - theta_old) / dt;
    error = W_ref - W_act;
    integral_error = integral_error_old + error * dt;
    derivative_error = error - error_old;

    // derivative_error = (error - error_old) / dt;

    pid = (Kp * error) +
        (Ki * integral_error) +
        (Kd * derivative_error);
    
    // State Variables
    X[0] = integral_error;
    X[1] = theta_m;
    X[2] = pid;
    X[3] = error;
    X[4] = derivative_error;
}

static void mdlTerminate(SimStruct *S)
{} /* Keep this function empty since no memory is allocated */ 

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif