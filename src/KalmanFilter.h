/*
KalmanFilter.h - Header file for the Kalman Filter
Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KalmanFilter_h
#define KalmanFilter_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class KalmanFilter
{
    public:

	KalmanFilter(float angle = 0.001, float bias = 0.003, float measure = 0.03);
	float update(float newValue, float newRate);

    private:

	float Q_angle, Q_bias, R_measure;
	float K_angle, K_bias, K_rate;
	float P[2][2], K[2];
	float S, y;
	float dt, kt;
};

#endif