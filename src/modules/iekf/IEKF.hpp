#include "ros/ros.hpp"
#include "matrix/math.hpp"

using namespace matrix;

/**
 * Note that structs are used instead of enums
 * to allow use in arrays without casting
 * and to keep size small
 */

/**
 * State enum
 */
struct X {
	static const uint8_t q_nb_0 = 0;
	static const uint8_t q_nb_1 = 1;
	static const uint8_t q_nb_2 = 2;
	static const uint8_t q_nb_3 = 3;
	static const uint8_t vel_N = 4;
	static const uint8_t vel_E = 5;
	static const uint8_t vel_D = 6;
	static const uint8_t gyro_bias_bx = 7;
	static const uint8_t gyro_bias_by = 8;
	static const uint8_t gyro_bias_bz = 9;
	static const uint8_t accel_scale = 10;
	static const uint8_t pos_N = 11;
	static const uint8_t pos_E = 12;
	static const uint8_t pos_D = 13;
	static const uint8_t terrain_alt = 14;
	static const uint8_t baro_bias = 15;
	static const uint8_t n = 16;
};

/**
 * Error state enum
 * used for linearization
 */
struct Xe {
	static const uint8_t rot_bx = 0;
	static const uint8_t rot_by = 1;
	static const uint8_t rot_bz = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t gyro_bias_bx = 6;
	static const uint8_t gyro_bias_by = 7;
	static const uint8_t gyro_bias_bz = 8;
	static const uint8_t accel_scale = 9;
	static const uint8_t pos_N = 10;
	static const uint8_t pos_E = 11;
	static const uint8_t pos_D = 12;
	static const uint8_t terrain_alt = 13;
	static const uint8_t baro_bias = 14;
	static const uint8_t n = 15;
};

/**
 * Input enum
 */
struct U {
	static const uint8_t omega_nb_bx = 0;
	static const uint8_t omega_nb_by = 1;
	static const uint8_t omega_nb_bz = 2;
	static const uint8_t accel_bx = 3;
	static const uint8_t accel_by = 4;
	static const uint8_t accel_bz = 5;
	static const uint8_t n = 6;
};

/**
 * Accel measurement enum
 */
struct Y_accel {
	static const uint8_t bx = 0;
	static const uint8_t by = 1;
	static const uint8_t bz = 2;
	static const uint8_t n = 3;
};

/**
 * GPS measurement
 */
struct Y_gps {
	static const uint8_t pos_N = 0;
	static const uint8_t pos_E = 1;
	static const uint8_t pos_D = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t n;
};

/**
 * Baro measurement
 */
struct Y_baro {
	static const uint8_t asl = 0;
	static const uint8_t n = 1;
};

/**
 * Magnetometer measurement
 */
struct Y_mag {
	static const uint8_t bx = 0;
	static const uint8_t by = 1;
	static const uint8_t bz = 2;
	static const uint8_t n = 3;
};

/**
 * Main class for invariant extended kalman filter
 *
 * inspired by: https://hal.archives-ouvertes.fr/hal-00494342/document
 *
 * Also see python directory for simulation and python version.
 */
class IEKF
{
public:
	IEKF();
	Vector<float, X::n> dynamics(const Vector<float, X::n> &x, const Vector<float, U::n> &u);
	bool ok() { return _nh.ok(); }
	void callback_gyro(const sensor_gyro_s *msg);
	void callback_accel(const sensor_accel_s *msg);
	void callback_mag(const sensor_mag_s *msg);
	void callback_baro(const sensor_baro_s *msg);
	void callback_attitude(const vehicle_attitude_s *msg);
	void predict(float dt);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub_gyro;
	ros::Subscriber _sub_accel;
	ros::Subscriber _sub_mag;
	ros::Subscriber _sub_baro;
	ros::Subscriber _sub_attitude;
	ros::Publisher _pub_attitude;
	ros::Publisher _pub_local_position;
	ros::Publisher _pub_global_position;
	Vector<float, X::n> _x; // state
	Matrix<float, Xe::n, Xe::n> _P; // covariance
	Vector<float, U::n> _u;
};
