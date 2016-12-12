#include "IEKF.hpp"

IEKF::IEKF() :
	_nh(), // node handle
	_sub_gyro(_nh.subscribe("sensor_gyro", 0, &IEKF::callback_gyro, this)),
	_sub_accel(_nh.subscribe("sensor_accel", 0, &IEKF::callback_accel, this)),
	_sub_mag(_nh.subscribe("sensor_mag", 0, &IEKF::callback_mag, this)),
	_sub_baro(_nh.subscribe("sensor_baro", 0, &IEKF::callback_baro, this)),
	_sub_attitude(_nh.subscribe("vehicle_attitude", 0, &IEKF::callback_attitude, this)),
	_pub_attitude(_nh.advertise<vehicle_attitude_s>("vehicle_attitude", 0)),
	_pub_local_position(_nh.advertise<vehicle_local_position_s>("vehicle_local_position", 0)),
	_pub_global_position(_nh.advertise<vehicle_global_position_s>("vehicle_global_position", 0)),
	_x(),
	_P()
{
	_x.setZero();
	_x(X::q_nb_0) = 1;
	_x(X::accel_scale) = 1;
	_P.setIdentity();
}

Vector<float, X::n> IEKF::dynamics(const Vector<float, X::n> &x, const Vector<float, U::n> &u)
{
	Quaternion<float> q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3<float> a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3<float> g_n(0, 0, -9.8);
	Vector3<float> as_n = q_nb.conjugate(a_b / _x(X::accel_scale)) - g_n;
	Vector3<float> gyro_bias_b(_x(X::gyro_bias_bx), _x(X::gyro_bias_by), _x(X::gyro_bias_bz));
	Vector3<float> omega_nb_b(_u(U::omega_nb_bx), _u(U::omega_nb_by), _u(U::omega_nb_bz));
	Quaternion<float> dq_nb = q_nb.derivative(omega_nb_b - gyro_bias_b);

	Vector<float, X::n> dx;
	dx.setZero();
	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);
	dx(X::vel_N) = as_n(0);
	dx(X::vel_E) = as_n(1);
	dx(X::vel_D) = as_n(2);
	dx(X::gyro_bias_bx) = 0;
	dx(X::gyro_bias_by) = 0;
	dx(X::gyro_bias_bz) = 0;
	dx(X::accel_scale) = 0;
	dx(X::pos_N) = x(X::vel_N);
	dx(X::pos_E) = x(X::vel_E);
	dx(X::pos_D) = x(X::vel_D);
	dx(X::terrain_alt) = 0;
	dx(X::baro_bias) = 0;
	return dx;
}

void IEKF::callback_gyro(const sensor_gyro_s *msg)
{
	//ROS_INFO("gyro callback %10.4f %10.4f %10.4f",
	//double(msg->x), double(msg->y), double(msg->x));
	_u(U::omega_nb_bx) = msg->x;
	_u(U::omega_nb_by) = msg->y;
	_u(U::omega_nb_bz) = msg->z;

	// predict driven by gyro callback
	if (msg->integral_dt > 0) {
		predict(msg->integral_dt / 1.0e6f);
	};
}

void IEKF::callback_accel(const sensor_accel_s *msg)
{
	//ROS_INFO("accel callback %10.4f %10.4f %10.4f",
	//double(msg->x), double(msg->y), double(msg->x));
	_u(U::accel_bx) = msg->x;
	_u(U::accel_by) = msg->y;
	_u(U::accel_bz) = msg->z;
}

void IEKF::callback_mag(const sensor_mag_s *msg)
{
	//ROS_INFO("mag callback %10.4f %10.4f %10.4f",
	//double(msg->x), double(msg->y), double(msg->x));
}

void IEKF::callback_baro(const sensor_baro_s *msg)
{
	//ROS_INFO("baro callback %10.4f", double(msg->altitude));

	// calculate residual
	Vector<float, Y_baro::n> y;
	y(Y_baro::asl) = msg->altitude;
	Vector<float, Y_baro::n> yh;
	yh(Y_baro::asl)	= -_x(X::pos_D) + _x(X::baro_bias);
	Vector<float, Y_baro::n> r = y - yh;

	// define R
	Matrix<float, Y_baro::n, Y_baro::n> R;
	R(Y_baro::asl, Y_baro::asl) = 1.0;

	// define H
	Matrix<float, Y_baro::n, Xe::n> H;
	H(Y_baro::asl, Xe::pos_D) = -1;
	H(Y_baro::asl, Xe::baro_bias) = 1;

	bool fault = correct<Y_baro::n>(r, H, R);

	if (fault) {
		ROS_WARN("baro fault");
	}
}

void IEKF::callback_attitude(const vehicle_attitude_s *msg)
{
	//ROS_INFO("attitude callback q_nb: %10.4f %10.4f %10.4f %10.4f",
	//double(msg->q[0]), double(msg->q[1]),
	//double(msg->q[2]), double(msg->q[3]));
}



void IEKF::predict(float dt)
{
	// define process noise matrix
	Matrix<float, Xe::n, Xe::n> Q;
	Q.setIdentity();
	Q *= 0.1;

	// define A matrix
	Matrix<float, Xe::n, Xe::n> A;
	A.setZero();

	// derivative of position is velocity
	A(Xe::pos_N, Xe::vel_N) = 1;
	A(Xe::pos_E, Xe::vel_E) = 1;
	A(Xe::pos_D, Xe::vel_D) = 1;

	// propgate state using euler integration
	Vector<float, X::n> dx = dynamics(_x, _u) * dt;
	_x += dx;

	// propgate covariance using euler integration
	Matrix<float, Xe::n, Xe::n> dP = (A * _P + _P * A.T() + Q) * dt;
	_P += dP;

	// publish attitude
	{
		vehicle_attitude_s msg = {};
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		_pub_attitude.publish(msg);
	}

	// publish local position
	{
		vehicle_local_position_s msg = {};
		msg.xy_valid = true;
		msg.z_valid = true;
		msg.v_xy_valid = true;
		msg.v_z_valid = true;
		msg.x = _x(X::pos_N);
		msg.y = _x(X::pos_E);
		msg.z = _x(X::pos_D);
		msg.vx = _x(X::vel_N);
		msg.vy = _x(X::vel_E);
		msg.vz = _x(X::vel_D);
		_pub_local_position.publish(msg);
	}

	// publish global position
	{
		vehicle_global_position_s msg = {};
		msg.lat = 0;
		msg.lon = 0;
		msg.alt = 0;
		msg.vel_n = 0;
		msg.vel_e = 0;
		msg.vel_d = 0;
		msg.yaw = 0;
		msg.eph = 0;
		msg.epv = 0;
		msg.terrain_alt = 0;
		msg.terrain_alt_valid = true;
		msg.dead_reckoning = false;
		msg.pressure_alt = 0;
		_pub_global_position.publish(msg);
	}
}
