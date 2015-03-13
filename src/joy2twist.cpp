#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>

#define FAIXA_MAX_SPEED 500




double speed_limits[2];//{max_speed_foward, max_speed_reverse};
double angle_limits[4];//{max_steerAngle_right, max_steerAngle_left, max_steerAngle_right_at_max_speed, max_steerAngle_left_at_max_speed};
double accel_limits[4];//{max_accel_speed_foward, max_accel_speed_reverse, max_accel_steerAngle_right, max_accel_steerAngle_left};


using namespace std;

class Joy2Twist
{
public:
  Joy2Twist();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void AtualizaComandoJoystick (double raw_linear, double raw_angle);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  geometry_msgs::Twist cmd_vel;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber joy_sub_;
  
};



int Sinal (double a){
	if (a < 0) return -1;
	else return 1;
}// Sinal

double Modulo (double a){
	if (a < 0) return -a;
	else return a;
}// Modulo

double Min (double a, double b){
	if (a < b) return a;
	else return b;
}// Min

bool AtMaxSpeed (double current_speed){
	if (current_speed > 0 && current_speed >= FAIXA_MAX_SPEED * speed_limits[0]) return true;
	else if (current_speed < 0 && current_speed <= -FAIXA_MAX_SPEED * speed_limits[1]) return true;
	return false;
}// AtMaxSpeed

double VelMax (double joystick_speed){
	// Calcula a velocidade máxima permitida dependendo da orientação.
	if (joystick_speed > 0) return speed_limits[0];
	else return speed_limits[1];
}// VelMax

double AngleMax (double &joystick_angle, double current_speed, bool keyboard = false){
	// Calcula o steerAngle maximo permitido dependendo da orientação e da velocidade.
	int incremento = 0;
	bool corrigeAngle = false;
	if (AtMaxSpeed(current_speed)) {
		if (!keyboard)incremento = 2;
		corrigeAngle = true; // No caso do keyboard é necessária uma correção do angulo para evitar saltos.
	}

	if (joystick_angle > 0) {
		if (keyboard && corrigeAngle)joystick_angle = Sinal (joystick_angle) * Min (Modulo(angle_limits[2] / angle_limits[0]), Modulo(joystick_angle));
		return angle_limits[0+incremento];
		}
	else {
		if (keyboard && corrigeAngle)joystick_angle = Sinal (joystick_angle) * Min (Modulo(angle_limits[3] / angle_limits[1]), Modulo(joystick_angle));
		return angle_limits[1+incremento];
	}
}// AngleMax

double AccelSpeedMax (double joystick_speed){
	// Calcula a aceleração máxima permitida dependendo da orientação.
	if (joystick_speed > 0) return accel_limits[0];
	else return accel_limits[1];
}// AccelSpeedMax

double AccelAngleMax (double joystick_angle){
	// Calcula a aceleracao maxima do steerAngle dependendo da orientação.
	if (joystick_angle > 0) return accel_limits[2];
	else return accel_limits[3];
}// AccelAngleMax

void LimitsInit (){
	ros::NodeHandle nh("~");
	// Init speed limits.
  
} // LimitsInit

void Joy2Twist::AtualizaComandoJoystick (double raw_linear, double raw_angle){
	cmd_vel.linear.x=raw_linear;
	cmd_vel.linear.y=0.0;
	cmd_vel.linear.z=0.0;
	cmd_vel.angular.x=0.0; 
	cmd_vel.angular.y=0.0;
	cmd_vel.angular.z=raw_angle;

} // AtualizaComandoJoystick


Joy2Twist::Joy2Twist():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  // Alterar para publicar mensagens do tipo CarCommand
 //vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
 cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy2Twist::joyCallback, this);

}

void Joy2Twist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
// Declara mensagens do tipo CarCommand e preencher com os ados do leitor


  ROS_INFO_STREAM("\nLinear: " << joy->axes[linear_] << "\nAngular: " << joy->axes[angular_]);
  AtualizaComandoJoystick(l_scale_*joy->axes[linear_],a_scale_*joy->axes[angular_]);
  cmd_vel_pub_.publish(cmd_vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy2twist");
  Joy2Twist joy2twist;
  LimitsInit();

  ros::spin();
}
