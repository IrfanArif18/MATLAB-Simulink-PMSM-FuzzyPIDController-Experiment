#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME Fuzzy_Logic  // s-Function Block Name
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumDiscStates(S, 5);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 2);   // Number of Inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 5);  // Number of Outputs
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE
    | SS_OPTION_DISCRETE_VALUED_OUTPUT));
} 

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, 1e-4);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS 

static void mdlInitializeConditions(SimStruct *S) {
    real_T *X0 = ssGetRealDiscStates(S);
    int_T nXStates = ssGetNumDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int_T i;

    /* Initialize the states to 0.0 */
    for (i = 0; i < nXStates; i++) X0[i] = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    real_T *X = ssGetRealDiscStates(S);

    real_T delta_Kp = X[1];
    real_T delta_Ki = X[2];
    real_T delta_Kd = X[3];

    Y[0] = delta_Kp;
    Y[1] = delta_Ki;
    Y[2] = delta_Kd;
    Y[3] = X[0]; // Error
    Y[4] = X[5]; // Change of Error
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T dt = 1e-4;

    // Input Ports
    real_T w_ref = U(0);
    real_T theta_m = U(1);
    
    real_T theta_old = X[4];

    real_T w_act = (theta_m - theta_old)/dt;
    real_T e_old = X[0];
    
    // Scaling
    real_T e_scale = 10;
    real_T ec_scale = 5;

    // Linguistic Variables
    real_T NL, NM, NS, Z, PS, PM, PL;
    
    // Fuzzification Logic
    real_T e_MFleft, e_DOMleft, e_MFright, e_DOMright; 
    real_T e_MFleft_1, e_MFleft_2, e_MFright_1, e_MFright_2;
    
    real_T ec_MFleft, ec_DOMleft, ec_MFright, ec_DOMright; 
    real_T ec_MFleft_1, ec_MFleft_2, ec_MFright_1, ec_MFright_2;

    // Error
    real_T e = w_ref - w_act;
    // Change of Error
    real_T ec = (e - e_old);

    // Normalization
    e = e / e_scale;
    ec = ec / ec_scale;
    
    // Degree of Membership for Error
    NL = -0.75 * e_scale;
    NM = -0.50 * e_scale;
    NS = -0.25 * e_scale;
	Z = 0.00 * e_scale;
    PS = 0.25 * e_scale;
    PM = 0.50 * e_scale;
	PL = 0.75 * e_scale;

    // Fuzzification for Error
    if (e <= NL) {
        e_MFleft = NL; // Lower NL Upper NL
        e_MFright = NL; // Lower NL Upper NL
        e_DOMleft = 1;        
        e_DOMright = 1;
    }
    else if (e >= PL) {
        e_MFleft = PL; // Lower PL Upper PL
        e_MFright = PL; // Lower PL Upper PL
        e_DOMleft = 1;
        e_DOMright = 1;
    }
    else if (e == NM) {
        e_MFleft = NM; // Lower NM Upper NM
        e_MFright = NM; // Lower NM Upper NM
        e_DOMleft = 1;
        e_DOMright = 1;
    }
    else if (e == NS) {
        e_MFleft = NS; // Lower NS Upper NS
        e_MFright = NS; // Lower NS Upper NS
        e_DOMleft = 1;
        e_DOMright = 1;
    }
    else if (e == Z) {
        e_MFleft = Z; // Lower Z Upper Z
        e_MFright = Z; // Lower Z Upper Z
        e_DOMleft = 1;
        e_DOMright = 1;
    }
    else if (e == PS) {
        e_MFleft = PS; // Lower PS Upper PS
        e_MFright = PS; // Lower PS Upper PS
        e_DOMleft = 1;
        e_DOMright = 1;
    }
    else if (e == PM) {
        e_MFleft = PM; // Lower PM Upper PM
        e_MFright = PM; // Lower PM Upper PM
        e_DOMleft = 1;
        e_DOMright = 1;
    }
    // Negative Side
    else if (NL < e && e < Z) {
        if (NL < e && e < NM) {
            e_MFleft = NL; // Lower NL Upper NL
            e_MFright = NM; // Lower NL Upper NS

            e_DOMleft = (NM - e) / (NM - NL);         
            
            e_MFright_1 = (e - NL) / (NM - NL);
            e_MFright_2 = (NS - e) / (NS - NM);

            if (e_MFright_1 > e_MFright_2) e_DOMright = e_MFright_2;
            else if (e_MFright_1 < e_MFright_2) e_DOMright = e_MFright_1;
            if (e_DOMright > 0) e_DOMright = e_DOMright;
            else if (e_DOMright < 0) e_DOMright = 0;
        }
        else if (NM < e && e < NS) {
            e_MFleft = NM; // Lower NL Upper NS
            e_MFright = NS; // Lower NM Upper Z

            e_MFleft_1 = (e - NL) / (NM - NL);
            e_MFleft_2 = (NS - NL) / (NS-NM);

            e_MFright_1 = (e - NM) / (NS - NM);
            e_MFright_2 = (Z - e) / (Z - NS);

            if (e_MFleft_1 > e_MFleft_2) e_DOMleft = e_MFleft_2;
            else if (e_MFleft_1 < e_MFleft_2) e_DOMleft = e_MFleft_1;
            if (e_DOMleft > 0) e_DOMleft = e_DOMleft;
            else if (e_DOMleft < 0) e_DOMleft = 0;

            if (e_MFright_1 > e_MFright_2) e_DOMright = e_MFright_2;
            else if (e_MFright_1 < e_MFright_2) e_DOMright = e_MFright_1;
            if (e_DOMright > 0) e_DOMright = e_DOMright;
            else if (e_DOMright < 0) e_MFright = 0;
        }
        else if (NS < e && e < Z) {
            e_MFleft = NS; // Lower NM Upper Z
            e_MFright = Z; // Lower NS Upper PS

            e_MFleft_1 = (e - NM) / (NS - NM);
            e_MFleft_2 = (Z - e) / (Z - NS);

            e_MFright_1 = (e - NS) / (Z - NS);
            e_MFright_2 = (PS - e) / (PS - Z);

            if (e_MFleft_1 > e_MFleft_2) e_DOMleft = e_MFleft_2;
            else if (e_MFleft_1 < e_MFleft_2) e_DOMleft = e_MFleft_1;
            if (e_DOMleft > 0) e_DOMleft = e_DOMleft;
            else if (e_DOMleft < 0) e_DOMleft = 0;

            if (e_MFright_1 > e_MFright_2) e_DOMright = e_MFright_2;
            else if (e_MFright_1 < e_MFright_2) e_DOMright = e_MFright_1;
            if (e_DOMright > 0) e_DOMright = e_DOMright;
            else if (e_DOMright < 0) e_DOMright = 0;
        }
    }
    // Positive Side
    else if (Z < e && e < PL) {
        if (Z < e && e < NS) {
            e_MFleft = Z; // Lower NS Upper PS
            e_MFright = PS; // Lower Z Upper PM
            
            e_MFleft_1 = (e - NS) / (Z - NS);
            e_MFleft_2 = (PS - e) / (PS-Z);

            e_MFright_1 = (e - Z) / (PS - Z);
            e_MFright_2 = (PM - e) / (PM - PS);

            if (e_MFleft_1 > e_MFleft_2) e_DOMleft = e_MFleft_2;
            else if (e_MFleft_1 < e_MFleft_2) e_DOMleft = e_MFleft_1;
            if (e_DOMleft > 0) e_DOMleft = e_DOMleft;
            else if (e_DOMleft < 0) e_DOMleft = 0;

            if (e_MFright_1 > e_MFright_2) e_DOMright = e_MFright_2;
            else if (e_MFright_1 < e_MFright_2) e_DOMright = e_MFright_1;
            if (e_DOMright > 0) e_DOMright = e_DOMright;
            else if (e_DOMright < 0) e_DOMright = 0;
        }
        else if (NS < e && e < PM) {
            e_MFleft = PS; // Lower Z Upper PM
            e_MFright = PM; // Lower PS Upper PL
            
            e_MFleft_1 = (e - Z) / (PS - Z);
            e_MFleft_2 = (PM - e) / (PM - PS);

            e_MFright_1 = (e - PS) / (PM - PS);
            e_MFright_2 = (PL - e) / (PL - PM);

            if (e_MFleft_1 > e_MFleft_2) e_DOMleft = e_MFleft_2;
            else if (e_MFleft_1 < e_MFleft_2) e_DOMleft = e_MFleft_1;
            if (e_DOMleft > 0) e_DOMleft = e_DOMleft;
            else if (e_DOMleft < 0) e_DOMleft = 0;

            if (e_MFright_1 > e_MFright_2) e_DOMright = e_MFright_2;
            else if (e_MFright_1 < e_MFright_2) e_DOMright = e_MFright_1;
            if (e_DOMright > 0) e_DOMright = e_DOMright;
            else if (e_DOMright < 0) e_DOMright = 0;
        }
        else if (PM < e < PL) {
            e_MFleft = PM; // Lower PS Upper PL
            e_MFright = PL; // Lower PL Upper PL
            
            e_MFleft_1 = (e - PS) / (PM - PS);
            e_MFleft_2 = (PL - e) / (PL - PM);

            e_DOMright = (PL - e) / (PL - PM);

            if (e_MFleft_1 > e_MFleft_2) e_DOMleft = e_MFleft_2;
            else if (e_MFleft_1 < e_MFleft_2) e_DOMleft = e_MFleft_1;
            if (e_DOMleft > 0) e_DOMleft = e_DOMleft;
            else if (e_DOMleft < 0) e_DOMleft = 0;
        }  
    }

    // Degree of Membership for Change of Error
    NL	= -0.75 * ec_scale;
    NM = -0.50 * ec_scale;
    NS = -0.25 * ec_scale;
	Z = 0.00 * ec_scale;
    PS = 0.25 * ec_scale;
    PM = 0.50 * ec_scale;
	PL = 0.75 * ec_scale;

    // Fuzzification for Change of Error
    if (ec <= NL) {
        ec_MFleft = NL; // Lower NL Upper NL
        ec_MFright = NL; // Lower NL Upper NL
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    else if (ec >= PL) {
        ec_MFleft = PL; // Lower PL Upper PL
        ec_MFright = PL; // Lower PL Upper PL
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    else if (ec == NM) {
        ec_MFleft = NM; // Lower NM Upper NM
        ec_MFright = NM; // Lower NM Upper NM
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    else if (ec == NS) {
        ec_MFleft = NS; // Lower NS Upper NS
        ec_MFright = NS; // Lower NS Upper NS
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    else if (ec == Z) {
        ec_MFleft = Z; // Lower Z Upper Z
        ec_MFright = Z; // Lower Z Upper Z
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    else if (ec == PS) {
        ec_MFleft = PS; // Lower PS Upper PS
        ec_MFright = PS; // Lower PS Upper PS
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    else if (ec == PM) {
        ec_MFleft = PM; // Lower PM Upper PM
        ec_MFright = PM; // Lower PM Upper PM
        ec_DOMleft = 1;
        ec_DOMright = 1;
    }
    // Negative Side
    else if (NL < ec && ec < Z) {
        if (NL < ec && ec < NM) {
            ec_MFleft = NL; // Lower NL Upper NL
            ec_MFright = NM; // Lower NL Upper NS

            ec_DOMleft = (NM - ec) / (NM - NL);
            
            ec_MFright_1 = (ec - NL) / (NM - NL);
            ec_MFright_2 = (NS - ec) / (NS - NM);

            if (ec_MFright_1 > ec_MFright_2) ec_DOMright = ec_MFright_2;
            else if (ec_MFright_1 < ec_MFright_2) ec_DOMright = ec_MFright_1;
            if (ec_DOMright > 0) ec_DOMright = ec_DOMright;
            else if (ec_DOMright < 0) ec_DOMright = 0;
        }
        else if (NM < ec && ec < NS) {
            ec_MFleft = NM; // Lower NL Upper NS
            ec_MFright = NS; // Lower NM Upper Z

            ec_MFleft_1 = (ec - NL) / (NM - NL);
            ec_MFleft_2 = (NS - NL) / (NS - NM);

            ec_MFright_1 = (ec - NM) / (NS - NM);
            ec_MFright_2 = (Z - ec) / (Z - NS);

            if (ec_MFleft_1 > ec_MFleft_2) ec_DOMleft = ec_MFleft_2;
            else if (ec_MFleft_1 < ec_MFleft_2) ec_DOMleft = ec_MFleft_1;
            if (ec_DOMleft > 0) ec_DOMleft = ec_DOMleft;
            else if (ec_DOMleft < 0) ec_DOMleft = 0;

            if (ec_MFright_1 > ec_MFright_2) ec_DOMright = ec_MFright_2;
            else if (ec_MFright_1 < ec_MFright_2) ec_DOMright = ec_MFright_1;
            if (ec_DOMright > 0) ec_DOMright = ec_DOMright;
            else if (ec_DOMright < 0) ec_MFright = 0;
        }
        else if (NS < ec && ec < Z) {
            ec_MFleft = NS; // Lower NM Upper Z
            ec_MFright = Z; // Lower NS Upper PS

            ec_MFleft_1 = (ec - NM) / (NS - NM);
            ec_MFleft_2 = (Z - ec) / (Z - NS);

            ec_MFright_1 = (ec - NS) / (Z - NS);
            ec_MFright_2 = (PS - ec) / (PS-Z);

            if (ec_MFleft_1 > ec_MFleft_2) ec_DOMleft = ec_MFleft_2;
            else if (ec_MFleft_1 < ec_MFleft_2) ec_DOMleft = ec_MFleft_1;
            if (ec_DOMleft > 0) ec_DOMleft = ec_DOMleft;
            else if (ec_DOMleft < 0) ec_DOMleft = 0;

            if (ec_MFright_1 > ec_MFright_2) ec_DOMright = ec_MFright_2;
            else if (ec_MFright_1 < ec_MFright_2) ec_DOMright = ec_MFright_1;
            if (ec_DOMright > 0) ec_DOMright = ec_DOMright;
            else if (ec_DOMright < 0) ec_DOMright = 0;
        }
    }
    // Positive Side
    else if (Z < ec && ec < PL) {
        if (Z < ec && ec < NS) {
            ec_MFleft = Z; // Lower NS Upper PS
            ec_MFright = PS; // Lower Z Upper PM
            
            ec_MFleft_1 = (ec - NS) / (Z - NS);
            ec_MFleft_2 = (PS - ec) / (PS - Z);

            ec_MFright_1 = (ec - Z) / (PS - Z);
            ec_MFright_2 = (PM - ec) / (PM - PS); 

            if (ec_MFleft_1 > ec_MFleft_2) ec_DOMleft = ec_MFleft_2;
            else if (ec_MFleft_1 < ec_MFleft_2) ec_DOMleft = ec_MFleft_1;
            if (ec_DOMleft > 0) ec_DOMleft = ec_DOMleft;
            else if (ec_DOMleft < 0) ec_DOMleft = 0;

            if (ec_MFright_1 > ec_MFright_2) ec_DOMright = ec_MFright_2;
            else if (ec_MFright_1 < ec_MFright_2) ec_DOMright = ec_MFright_1;
            if (ec_DOMright > 0) ec_DOMright = ec_DOMright;
            else if (ec_DOMright < 0) ec_DOMright = 0;
        }
        else if (NS < ec && ec < PM) {
            ec_MFleft = PS; // Lower Z Upper PM
            ec_MFright = PM; // Lower PS Upper PL
            
            ec_MFleft_1 = (ec - Z) / (PS - Z);
            ec_MFleft_2 = (PM - ec) / (PM - PS);

            ec_MFright_1 = (ec - PS) / (PM - PS);
            ec_MFright_2 = (PL - ec) / (PL - PM);

            if (ec_MFleft_1 > ec_MFleft_2) ec_DOMleft = ec_MFleft_2;
            else if (ec_MFleft_1 < ec_MFleft_2) ec_DOMleft = ec_MFleft_1;
            if (ec_DOMleft > 0) ec_DOMleft = ec_DOMleft;
            else if (ec_DOMleft < 0) ec_DOMleft = 0;

            if (ec_MFright_1 > ec_MFright_2) ec_DOMright = ec_MFright_2;
            else if (ec_MFright_1 < ec_MFright_2) ec_DOMright = ec_MFright_1;
            if (ec_DOMright > 0) ec_DOMright = ec_DOMright;
            else if (ec_DOMright < 0) ec_DOMright = 0;  
        }
        else if (PM < ec && ec < PL) {
            ec_MFleft = PM; // Lower PS Upper PL
            ec_MFright = PL; // Lower PL Upper PL
            
            ec_MFleft_1 = (ec - PS) / (PM - PS);
            ec_MFleft_2 = (PL - ec) / (PL - PM);

            ec_DOMright = (PL - ec)/(PL - PM);

            if (ec_MFleft_1 > ec_MFleft_2) ec_DOMleft = ec_MFleft_2;
            else if (ec_MFleft_1 < ec_MFleft_2) ec_DOMleft = ec_MFleft_1;
            if (ec_DOMleft > 0) ec_DOMleft = ec_DOMleft;
            else if (ec_DOMleft < 0) ec_DOMleft = 0;
        }  
    }

    // Output of Fuzzy Variables
    real_T KP_OMF, KI_OMF, KD_OMF;
    real_T e_OMF, e_ODOM, ec_OMF, ec_ODOM;

    // Weighted Average
    e_OMF = (e_DOMleft * e_MFleft + e_DOMright * e_MFright) / (e_DOMleft + e_DOMright);
    ec_OMF = (ec_DOMleft * ec_MFleft + ec_DOMright * ec_MFright) / (ec_DOMleft + ec_DOMright);
    
    // Inference Engine
    // Base Rule for KP KI KD
    if (ec_OMF == NL) {
        if (e_OMF == NL) {KP_OMF = PL; KI_OMF = NL; KD_OMF = PS;}
        else if (e_OMF == NM) {KP_OMF = PL; KI_OMF = NL; KD_OMF = PS;}
        else if (e_OMF == NS) {KP_OMF = PM; KI_OMF = NL; KD_OMF = Z;}
        else if (e_OMF == Z) {KP_OMF = PM; KI_OMF = NM; KD_OMF = Z;}
        else if (e_OMF == PS) {KP_OMF = PS; KI_OMF = NM; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = PS; KI_OMF = Z; KD_OMF = PL;}
        else if (e_OMF == PL) {KP_OMF = Z; KI_OMF = Z; KD_OMF = PL;}
    }
    else if (ec_OMF == NM) {
        if (e_OMF == NL) {KP_OMF = PL; KI_OMF = NL; KD_OMF = NS;}
        else if (e_OMF == NM) {KP_OMF = PL; KI_OMF = NL; KD_OMF = NS;}
        else if (e_OMF == NS) {KP_OMF = PM; KI_OMF = NM; KD_OMF = NS;}
        else if (e_OMF == Z) {KP_OMF = PM; KI_OMF = NM; KD_OMF = NS;}
        else if (e_OMF == PS) {KP_OMF = PS; KI_OMF = NS; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = Z; KI_OMF = Z; KD_OMF = NS;}
        else if (e_OMF == PL) {KP_OMF = Z; KI_OMF = Z; KD_OMF = PM;}
    }
    else if (ec_OMF == NS) {
        if (e_OMF == NL) {KP_OMF = PM; KI_OMF = NM; KD_OMF = NL;}
        else if (e_OMF == NM) {KP_OMF = PM; KI_OMF = NM; KD_OMF = NL;}
        else if (e_OMF == NS) {KP_OMF = PM; KI_OMF = NS; KD_OMF = NM;}
        else if (e_OMF == Z) {KP_OMF = PS; KI_OMF = NS; KD_OMF = NS;}
        else if (e_OMF == PS) {KP_OMF = Z; KI_OMF = Z; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = NS; KI_OMF = PS; KD_OMF = PS;}
        else if (e_OMF == PL) {KP_OMF = NM; KI_OMF = PS; KD_OMF = PM;}
    }
    else if (ec_OMF == Z) {
        if (e_OMF == NL) {KP_OMF = PS; KI_OMF = NS; KD_OMF = NL;}
        else if (e_OMF == NM) {KP_OMF = PS; KI_OMF = NS; KD_OMF = NM;}
        else if (e_OMF == NS) {KP_OMF = PS; KI_OMF = NS; KD_OMF = NM;}
        else if (e_OMF == Z) {KP_OMF = Z; KI_OMF = Z; KD_OMF = NS;}
        else if (e_OMF == PS) {KP_OMF = NS; KI_OMF = PS; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = NM; KI_OMF = PS; KD_OMF = PS;}
        else if (e_OMF == PL) {KP_OMF = NM; KI_OMF = PM; KD_OMF = PM;}
    }
    else if (ec_OMF == PS) {
        if (e_OMF == NL) {KP_OMF = PS; KI_OMF = NS; KD_OMF = NL;}
        else if (e_OMF == NM) {KP_OMF = PS; KI_OMF = NS; KD_OMF = NM;}
        else if (e_OMF == NS) {KP_OMF = Z; KI_OMF = Z; KD_OMF = NS;}
        else if (e_OMF == Z) {KP_OMF = NS; KI_OMF = PS; KD_OMF = NS;}
        else if (e_OMF == PS) {KP_OMF = NM; KI_OMF = PS; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = NM; KI_OMF = PM; KD_OMF = PS;}
        else if (e_OMF == PL) {KP_OMF = NM; KI_OMF = PM; KD_OMF = PS;}
    }
    else if (ec_OMF == PM) {
        if (e_OMF == NL) {KP_OMF = Z; KI_OMF = Z; KD_OMF = NM;}
        else if (e_OMF == NM) {KP_OMF = Z; KI_OMF = Z; KD_OMF = NS;}
        else if (e_OMF == NS) {KP_OMF = NS; KI_OMF = PS; KD_OMF = NS;}
        else if (e_OMF == Z) {KP_OMF = NM; KI_OMF = PM; KD_OMF = NS;}
        else if (e_OMF == PS) {KP_OMF = NM; KI_OMF = PM; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = NM; KI_OMF = PL; KD_OMF = PS;}
        else if (e_OMF == PL) {KP_OMF = NL; KI_OMF = PL; KD_OMF = PS;}
    }
    else if (ec_OMF == PL) {
        if (e_OMF == NL) {KP_OMF = Z; KI_OMF = Z; KD_OMF = PS;}
        else if (e_OMF == NM) {KP_OMF = NS; KI_OMF = Z; KD_OMF = Z;}
        else if (e_OMF == NS) {KP_OMF = NS; KI_OMF = PS; KD_OMF = Z;}
        else if (e_OMF == Z) {KP_OMF = NM; KI_OMF = PM; KD_OMF = Z;}
        else if (e_OMF == PS) {KP_OMF = NM; KI_OMF = PL; KD_OMF = Z;}
        else if (e_OMF == PM) {KP_OMF = NL; KI_OMF = PL; KD_OMF = PL;}
        else if (e_OMF == PL) {KP_OMF = NL; KI_OMF = PL; KD_OMF = PL;}
    }
    
    // Crisp Output Variables
    real_T delta_Kp, delta_Ki, delta_Kd;

    // Defuzzification
    // Crisp Value for Output
    if (KP_OMF == NL) {delta_Kp = -3;}
    else if (KP_OMF == NM) {delta_Kp = -2;}
    else if (KP_OMF == NS) {delta_Kp = -1;}
    else if (KP_OMF == Z) {delta_Kp = 0;}
    else if (KP_OMF == PS) {delta_Kp = 1;}
    else if (KP_OMF == PM) {delta_Kp = 2;}
    else if (KP_OMF == PL) {delta_Kp = 3;}

    if (KI_OMF == NL) {delta_Ki = -3;}
    else if (KI_OMF == NM) {delta_Ki = -2;}
    else if (KI_OMF == NS) {delta_Ki = -1;}
    else if (KI_OMF == Z) {delta_Ki = 0;}
    else if (KI_OMF == PS) {delta_Ki = 1;}
    else if (KI_OMF == PM) {delta_Ki = 2;}
    else if (KI_OMF == PL) {delta_Ki = 3;}

    if (KD_OMF == NL) {delta_Ki = -3;}
    else if (KD_OMF == NM) {delta_Kd = -2;}
    else if (KD_OMF == NS) {delta_Kd = -1;}
    else if (KD_OMF == Z) {delta_Kd = 0;}
    else if (KD_OMF == PS) {delta_Kd = 1;}
    else if (KD_OMF == PM) {delta_Kd = 2;}
    else if (KD_OMF == PL) {delta_Kd = 3;}

    X[0] = e;
    X[1] = delta_Kp;
    X[2] = delta_Ki;
    X[3] = delta_Kd;
    X[4] = theta_m;
    X[5] = ec;
}

static void mdlTerminate(SimStruct *S)
{} /* Keep this function empty since no memory is allocated */ 

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif