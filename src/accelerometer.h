/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef QS_SENSOR_ACCELEROMETER_H
#define QS_SENSOR_ACCELEROMETER_H

#include <random>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class accelerometer
{
	public:
	
		accelerometer();
		~accelerometer();
		
		// sensor calibration
		void read_calibration_value(Vector3d xdotdot_ideal, Vector3d attitude_ideal);
		void calibrate();

		// chip tilt calibration
		void take_chip_off_quadcopter();
		void place_chip_on_quadcopter();

		// sensor reading
		void get_corrupted_accelerations(Vector3d *xdotdot_bf_corrupted, Vector3d xdotdot_ideal, Vector3d attitude_ideal);
		
	private:

		Vector3d calibrated_offsets;
		Vector3d calibrated_offsets_sum;

		Vector3d chip_tilt;

		std::default_random_engine mag_randomGen;
};

#endif