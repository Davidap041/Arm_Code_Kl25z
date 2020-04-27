/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Kalman.h"

/* We will set the variables like so, these can also be tuned by the user */
float Q_angle = 0.00008f;
float Q_bias = 0.004f;
float R_measure = 0.03f;

float angle = 0.0f; // Reset the angle
float bias = 0.0f; // Reset bias

float P[1][1];

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(Kalman_data *kalman, float newAngle, float newRate, float dt) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	kalman->rate = newRate - bias;
	angle += dt * kalman->rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = P[0][0] + R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - angle; // Angle difference
	/* Step 6 */
	angle += K[0] * y;
	bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return angle;
}

void setAngle(Kalman_data *kalman, float angle) {
	kalman->angle = angle;
}
// Used to set angle, this should be set as the starting angle
float getRate(Kalman_data *kalman) {
	return kalman->rate;
}

// Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(Kalman_data *kalman, float Q_angle) {
	kalman->Q_angle = Q_angle;
}

void setQbias(Kalman_data *kalman, float Q_bias) {
	kalman->Q_bias = Q_bias;
}

void setRmeasure(Kalman_data *kalman, float R_measure) {
	kalman-> R_measure = R_measure;
}


float getQangle(Kalman_data *kalman) {
	return kalman->Q_angle;
}

float getQbias(Kalman_data *kalman) {
	return kalman->Q_bias;
}

float getRmeasure(Kalman_data *kalman) {
	return kalman->R_measure;
}

