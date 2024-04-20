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
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
      this->p = Kpi;
      this->i = Kii;
      this->d = Kdi;
      this->max = output_lim_maxi;
      this->min = output_lim_mini;
      this->delta = 0;
      this->e_p = 0;
      this->e_i = 0;
      this->e_d = 0;
   
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   this->e_p = cte;
  
  	if (this->delta > 0.0) {

    	this->e_d = (cte - e_p) / this->delta;
  	}
  
  	else {
    // Divide by zero: set resulting derivative error to 0.0
    	this->e_d = 0.0; 
  	}
  
  	this->e_i += cte * this->delta;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control  = ((this->p * this->e_p) + (this->i * this->e_i) + (this->d * this->e_d));

   if(control > this->max)
    {
      return this->max;
    }
  
  	else if(control < this->min)
    {
      return this->min;
    }
    else
    {
      return control;
    }
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta = new_delta_time;
  	return this->delta;
}