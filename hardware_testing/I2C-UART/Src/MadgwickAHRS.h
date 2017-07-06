#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickSetBeta(float _beta);
void MadgwickSetDelta(float _deltat);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *angle);

#endif
