/*
 * configuration.hpp
 *
 *  Created on: Oct 18, 2020
 *      Author: Joe
 */

#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

// ==== Gradient Descent Config ====
struct params_t{

//	double ALPHA 	= -0.01; 	// step size, Negative because TakeStep() assumes new_pos = old_pos + step
//	double EPSILON 	= 0.25;		// Threshold for completion
//	double K_ATT 	= 14.5;		// gain for U_att  14.5;
//	double K_REP	= 0.0135; 		// gain for U_rep
//	double Q_STAR 	= 3.7;		// Dist from obstacle threshold, only use U_rep when q is within this threshold
//	double D_STAR 	= 1.25;		// Dist from goal threshold, switch to quadratic U_att when q is within this threshold 1.25;


//	double ALPHA 	= -0.01; 	// step size, Negative because TakeStep() assumes new_pos = old_pos + step
//	double EPSILON 	= 0.25;		// Threshold for completion
//	double K_ATT 	= 3.0;		// gain for U_att
//	double K_REP	= 0.5; 		// gain for U_rep
//	double Q_STAR 	= 0.3;		// Dist from obstacle threshold, only use U_rep when q is within this threshold
//	double D_STAR 	= 5.0;		// Dist from goal threshold, switch to quadratic U_att when q is within this threshold

	/// PART a VALUES
//	double ALPHA 	= -0.01; 	// step size, Negative because TakeStep() assumes new_pos = old_pos + step
//	double EPSILON 	= 0.25;		// Threshold for completion
//	double K_ATT 	= 5.0;		// gain for U_att
//	double K_REP	= 0.1; 		// gain for U_rep
//	double Q_STAR 	= 1.0;		// Dist from obstacle threshold, only use U_rep when q is within this threshold
//	double D_STAR 	= 5.0;		// Dist from goal threshold, switch to quadratic U_att when q is within this threshold

	/// PART b WORKSPACE 1 VALUES
//	double ALPHA 	= -0.01; 	// step size, Negative because TakeStep() assumes new_pos = old_pos + step
//	double EPSILON 	= 0.25;		// Threshold for completion
//	double K_ATT 	= 2.0;		// gain for U_att
//	double K_REP	= 0.015; 		// gain for U_rep 0.015
//	double Q_STAR 	= 4.0;		// Dist from obstacle threshold, only use U_rep when q is within this threshold
//	double D_STAR 	= 5;		// Dist from goal threshold, switch to quadratic U_att when q is within this threshold

	double ALPHA 	= -0.01; 	// step size, Negative because TakeStep() assumes new_pos = old_pos + step
	double EPSILON 	= 0.25;		// Threshold for completion
	double K_ATT 	= 0.75;		// gain for U_att
	double K_REP	= 5.0; 		// gain for U_rep
	double Q_STAR 	= 4.0;		// Dist from obstacle threshold, only use U_rep when q is within this threshold
	double D_STAR 	= 5;		// Dist from goal threshold, switch to quadratic U_att when q is within this threshold



};

extern params_t PLANNER_PARAMS;	// make it global :(



#endif /* CONFIGURATION_HPP_ */
