#include "omni_3wheel_control.h"
#include "math.h"


omni_3wheel_control::omni_3wheel_control() : 
	wheel_1(0),
	wheel_2(0),
	wheel_3(0),
	wheel_1_velocity(0),
	wheel_2_velocity(0),
	wheel_3_velocity(0),
	global_velocity_x(0),
	global_velocity_y(0),
	local_velocity_x(0),
	local_velocity_y(0),
	robot_angle(0)
{

	cos_theta_1 = 1 / 2;
	sin_theta_1 = sqrt_3 / 2;

	cos_theta_2 = 1 / 2;
	sin_theta_2 = -(sqrt_3) / 2;

	cos_theta_3 = -1;
	sin_theta_3 = 0;


	return;
}

void omni_3wheel_control::local_control(float local_velocity_x, float local_velocity_y){

	this->local_velocity_x = local_velocity_x;
	this->local_velocity_y = local_velocity_y;

	wheel_1 = this->local_velocity_x * cos_theta_1 + this->local_velocity_y * sin_theta_1;
	wheel_2 = this->local_velocity_x * cos_theta_2 + this->local_velocity_y * sin_theta_2;
	wheel_3 = this->local_velocity_x * cos_theta_3 + this->local_velocity_y * sin_theta_3;

	return;
}

void omni_3wheel_control::global_control(float global_velocity_x, float global_velocity_y) {
	
	this->global_velocity_x = global_velocity_x;
	this->global_velocity_y = global_velocity_y;

	float angle = 0;
	angle = this->get_robot_angle();

	this->local_control(
		this->global_velocity_x * cosf(angle)+global_velocity_y * sinf(angle),
		this->global_velocity_y * cosf(angle)-this->global_velocity_x * sinf(angle)
	);

	return;
}


float omni_3wheel_control::get_wheel_velocity_1() {
	this->wheel_1_velocity = this->wheel_1;
	return this->wheel_1_velocity;
}

float omni_3wheel_control::get_wheel_velocity_2() {
	this->wheel_2_velocity = this->wheel_2;
	return this->wheel_2_velocity;
}

float omni_3wheel_control::get_wheel_velocity_3() {
	this->wheel_3_velocity = this->wheel_3;
	return this->wheel_3_velocity;
}


void omni_3wheel_control::set_robot_angle(float angle) {
	this->robot_angle = angle;
	return;
}

float omni_3wheel_control::get_robot_angle() {
	return this->robot_angle;
}