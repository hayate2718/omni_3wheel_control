#pragma once

/*
�O��Ƃ��ăI���j�z�C�[���̈ʒu�֌W�y�ьv�Z�͉��LURL�Q��
"https://qiita.com/hayate2718/items/9f5db2bfb46d39a7b330"
*/

/*
MCU���̌v�Z�����̏��Ȃ����Z���u��z�肵�Ė������̒萔���ɒ�`����B
32bitsMCU���v�Z�������Ȃ����́A�c�_�̗]�n�����邩...
*/

#define PI 3.141592f //�~�������`
#define sqrt_3 1.732051f //3^(0.5)���`

class omni_3wheel_control
{
private:

	float sin_theta_1; //�e�z�C�[���̉�]���������[�J�����W�n��x���ƂȂ��p�ɂ��x,y�������`����
	float cos_theta_1;

	float sin_theta_2;
	float cos_theta_2;

	float sin_theta_3;
	float cos_theta_3;
	
	float wheel_1; //�o�b�t�@
	float wheel_2;
	float wheel_3;

	float wheel_1_velocity; //�e�I���j�z�C�[���̖ڕW���x(��]���ł͂Ȃ�)[m/s]
	float wheel_2_velocity;
	float wheel_3_velocity;


	float local_velocity_x; //���[�J�����W�n�̖ڕW���x[m/s]
	float local_velocity_y;

	float global_velocity_x; //�O���[�o�����W�n�̖ڕW���x[m/s]
	float global_velocity_y;

	float robot_angle; //���[�J�����W�n�ƃO���[�o�����W�n���Ȃ��p

	float robot_range; //�I���j�z�C�[���Əd�S�܂ł̋���[m]
	float robot_angular_velocity; //���{�b�g�̖ڕW�p���x[rad/s]
	float robot_angular_wheel; //���{�b�g�̉�]�����̃z�C�[�����x[m/s]

public:

	omni_3wheel_control();

	void local_control(float local_velocity_x, float local_velocity_y); //���[�J�����W�n�̐���֐�

	void global_control(float global_velocity_x,float global_velocity_y); //�O���[�o�����W�n�̐���֐�

	void angular_velocity_control(float robot_angular_velocity);

	float get_wheel_velocity_1(); //�z�C�[���̖ڕW���x���擾����

	float get_wheel_velocity_2();

	float get_wheel_velocity_3();

	void set_sin_theta_1(float theta);
	void set_cos_theta_1(float theta);

	void set_sin_theta_2(float theta);
	void set_cos_theta_2(float theta);
	
	void set_sin_theta_3(float theta);
	void set_cos_theta_3(float theta);

	void set_robot_range(float range);

	void set_robot_angle(float angle); //���[�J�����W�n�ƃO���[�o�����W�n���Ȃ��p����͂���[rad]

	float get_robot_angle(); //���[�J�����W�n�ƃO���[�o�����W�n���Ȃ��p���擾����[rad]

};

