/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * Initialize PID coefficients (and errors, if needed)
   **/
   mKp = Kpi;
   mKi = Kii;
   mKd = Kdi;
   mMaxOutput = output_lim_maxi;
   mMinOutput = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/
   mIErr =  mIErr + cte * mDt; 
   mDErr = (cte - mPErr) / mDt;
   mPErr = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = mKp * mPErr + mKd * mDErr + mKi * mIErr;
    return std::min(std::max(control, mMinOutput), mMaxOutput);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
   mDt = new_delta_time;
}