#include "arm_math.h"
#include "math_helper.h"
#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include <driverlib/sysctl.h>
#include <driverlib/fpu.h>
#include <math.h>
#include "IQmath/IQmathLib.h"

volatile int timeBef;
volatile int timeAft;
volatile int clkVal = 0;
volatile float l1 = 0.25;
volatile float l2 = 0.25;
volatile float32_t  theta1 = 0.3232;
volatile float32_t  theta2 = 0.3598;
volatile float32_t  cosOutput1;
volatile float32_t  cosOutput2;
volatile float32_t  cosOutput12;
volatile float32_t  sinOutput1;
volatile float32_t  sinOutput2;
volatile float32_t  sinOutput12;
volatile float32_t  cosSquareOutput1;
volatile float32_t  cosSquareOutput2;
volatile float32_t  sinSquareOutput1;
volatile float32_t  sinSquareOutput2;
volatile float32_t  Pxx;
volatile float32_t  Pyy;
volatile float32_t  r;
float32_t  theta;
volatile _iq24 thetaiqresult;
volatile _iq24 Pxiq;
volatile _iq24 Pyiq;
volatile float32_t Trial_Real_theta1;
volatile float32_t Trial_Real_theta2;
volatile float32_t Trial_Real_theta3;

float ForwardKin(float thet1, float thet2){

    cosOutput1 = arm_cos_f32(thet1);
    cosOutput12 = arm_cos_f32(thet2+thet1);
    sinOutput1 = arm_sin_f32(thet1);
    sinOutput12 = arm_sin_f32(thet2+thet1);
    arm_mult_f32(&cosOutput1, &cosOutput1, &cosSquareOutput1, 1);
    arm_mult_f32(&sinOutput1, &sinOutput1, &sinSquareOutput1, 1);
    //arm_mult_f32(&cosOutput2, &cosOutput12, &cosSquareOutput2, 1);
    //arm_mult_f32(&sinOutput2, &sinOutput12, &sinSquareOutput2, 1);

    Pxx = l1*cosOutput1 + l2*cosOutput12;
    Pyy = l1*sinOutput1 + l2*sinOutput12;

    return Pyy;
}

float PolarTrans(float32_t Px, float32_t Py){

    volatile float32_t sum = ((Px*Px) + (Py*Py));
    arm_sqrt_f32(sum, &r);
    theta = atan2(Py, Px);
//    Pxiq = _IQ24(Px);
//    Pyiq = _IQ24(Py);
//    thetaiqresult = _IQ24atan2(Pyiq,Pxiq);
//   theta = _IQ24toF(thetaiqresult);

    return theta;
}

float Jacobian(float32_t force, float32_t torque, float32_t thet2 ){
    cosOutput2 = arm_cos_f32(thet2);
    sinOutput2 = arm_sin_f32(thet2);
    float32_t sqrt_denmult_cos;
    float32_t Torque1;
    float32_t Torque2;
    float32_t denmultcos = (l1*l1)+(l2*l2)+(2*l1*l2*cosOutput2);
    arm_sqrt_f32(denmultcos, &sqrt_denmult_cos);
    float32_t denmultsin = (l1*l1)+(l2*l2)+(2*l1*l2*sinOutput2);
    Torque1 = (-1)*l1*l2*sinOutput2*force/sqrt_denmult_cos;
    Torque2 = force + ((l2*l2)+l1*l1*cosOutput2)/denmultsin;
    return Torque1;
}

int32_t main(void)
{

  FPUEnable();
  FPULazyStackingEnable();
  SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5);
  clkVal = SysCtlClockGet();
  NVIC_ST_RELOAD_R = 0xFFFFFF;    /* reload reg. with max value */
  NVIC_ST_CTRL_R = 5;             /* enable it, no interrupt, use system clock */

  /* calculation of AT Multiply with A */
  timeBef = NVIC_ST_CURRENT_R;
  Trial_Real_theta1 = ForwardKin(theta1,theta2);
  Trial_Real_theta2 = PolarTrans(theta1,theta2);
  Trial_Real_theta3 = Jacobian(theta1,theta2, theta2);
  timeAft = NVIC_ST_CURRENT_R;

  while(1);                             /* main function does not return */
}

 /** \endlink */
