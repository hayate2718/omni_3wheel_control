#pragma once

/*
前提としてオムニホイールの位置関係及び計算は下記URL参照
"https://qiita.com/hayate2718/items/9f5db2bfb46d39a7b330"
*/

/*
MCU等の計算資源の少ない演算装置を想定して無理数の定数を先に定義する。
32bitsMCUが計算資源少ないかは、議論の余地があるか...
*/

#define PI 3.141592f //円周率を定義
#define sqrt_3 1.732051f //3^(0.5)を定義

class omni_3wheel_control
{
private:

	float sin_theta_1; //各ホイールの回転方向がローカル座標系のx軸となす角によるx,y成分を定義する
	float cos_theta_1;

	float sin_theta_2;
	float cos_theta_2;

	float sin_theta_3;
	float cos_theta_3;
	
	float wheel_1; //バッファ
	float wheel_2;
	float wheel_3;

	float wheel_1_velocity; //各オムニホイールの目標速度(回転数ではない)[m/s]
	float wheel_2_velocity;
	float wheel_3_velocity;


	float local_velocity_x; //ローカル座標系の目標速度[m/s]
	float local_velocity_y;

	float global_velocity_x; //グローバル座標系の目標速度[m/s]
	float global_velocity_y;

	float robot_angle; //ローカル座標系とグローバル座標系がなす角

	float robot_range; //オムニホイールと重心までの距離[m]
	float robot_angular_velocity; //ロボットの目標角速度[rad/s]
	float robot_angular_wheel; //ロボットの回転方向のホイール速度[m/s]

public:

	omni_3wheel_control();

	void local_control(float local_velocity_x, float local_velocity_y); //ローカル座標系の制御関数

	void global_control(float global_velocity_x,float global_velocity_y); //グローバル座標系の制御関数

	void angular_velocity_control(float robot_angular_velocity);

	float get_wheel_velocity_1(); //ホイールの目標速度を取得する

	float get_wheel_velocity_2();

	float get_wheel_velocity_3();

	void set_sin_theta_1(float theta);
	void set_cos_theta_1(float theta);

	void set_sin_theta_2(float theta);
	void set_cos_theta_2(float theta);
	
	void set_sin_theta_3(float theta);
	void set_cos_theta_3(float theta);

	void set_robot_range(float range);

	void set_robot_angle(float angle); //ローカル座標系とグローバル座標系がなす角を入力する[rad]

	float get_robot_angle(); //ローカル座標系とグローバル座標系がなす角を取得する[rad]

};

