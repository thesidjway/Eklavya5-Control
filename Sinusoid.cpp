#include <iostream>
#include "vx_pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctime>

int manualmode=0;
using namespace std;
int state=1;
int bias=1400;
struct timeval t1, t2;

int difftimer()
{
	gettimeofday(&t2, NULL);
	return (t2.tv_sec - t1.tv_sec) * 1000 + (t2.tv_usec - t1.tv_usec)/1000;
}
VxPid::VxPid() : Vx_t_lock(), Alpha_lock(), Vl_Vr_a_lock(),manualmode_lock() {

	Vr_a=0;
	Vl_a=0;
	Vx_t=0;
	Vy_t=+0.0;
	Vz_t=0.0; 
	Alpha_a=0;
	Kp_Vx=0;
	Ki_Vx=0;
	Kd_Vx=0;
	Vx_error_diff=0;
	Vx_error_old=0;
	Vx_error_integral=0;
	PWM_Duty_Cycle=0;
	vx_pid_loop_rate=0;
	//int    Vs_PID_loop_rate;

	PWM_min_percent=0; PWM_max_percent=0;
	PWM_PERIOD_TIME=0;     // in ns
	
	
}

double VxPid::getMinMax(int Cur_Var, int max, int min) {

    if (Cur_Var > max) {
		return max;
	} else {
		if (Cur_Var < min) {
			return min;
		} else {
			return Cur_Var;
		}
	}
}

void VxPid::vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    Vy_t_lock.lock();
    Vx_t_lock.lock();
    manualmode_lock.lock();
	Alpha_lock.lock();
	Vx_t = (msg->linear.x);
	Vy_t=(msg->linear.y);
	Vz_t=(msg->linear.z);
	//	Vs_t= (msg->linear.x )/( cos( (Alpha_a * PI)/180));
	if(std::signbit(Vz_t)) 
			manualmode = 1;
    else
			manualmode=0;
	Alpha_lock.unlock();
	Vx_t_lock.unlock();
	Vy_t_lock.unlock();
	manualmode_lock.unlock();
	

}


void VxPid::encoderCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	Vl_Vr_a_lock.lock();
	Vl_a = msg->linear.x;
	Vr_a = msg->linear.y;
	Vl_Vr_a_lock.unlock();

}

void VxPid::implementPid(int argc, char** argv)
{
	gettimeofday(&t1, NULL);
	
	time_t start;
  	time(&start);

	ros::init(argc, argv, "vxpid_node");

	ros::NodeHandle nh_;

	ros::Publisher va_pub = nh_.advertise<std_msgs::Float64>("va", 20);			
	ros::Publisher vt_pub = nh_.advertise<std_msgs::Float64>("vt", 20);		
	ros::Publisher dac_pub = nh_.advertise<std_msgs::Float64>("dac", 20);
	ros::Publisher pub=nh_.advertise<geometry_msgs::Twist>("twist_msg", 20); 
	ros::Subscriber Override_Subscriber = nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,&VxPid::vxTargetUpdateCallback, this);
	ros::Subscriber Encoder_Subscriber = nh_.subscribe<geometry_msgs::Twist>("encoders", 5, &VxPid::encoderCallback, this);
																	


	nh_.getParam("/vxpid_node/Kp_Vx", Kp_Vx);
	nh_.getParam("/vxpid_node/Ki_Vx", Ki_Vx);
	nh_.getParam("/vxpid_node/Kd_Vx", Kd_Vx);
	nh_.getParam("/vxpid_node/PWM_min", PWM_min_percent);
	nh_.getParam("/vxpid_node/bias", bias);
	nh_.getParam("/vxpid_node/PWM_max", PWM_max_percent);
	nh_.getParam("/vxpid_node/PWM_PERIOD_TIME", PWM_PERIOD_TIME);
	nh_.getParam("/vxpid_node/vx_pid_loop_rate", vx_pid_loop_rate);

	
	ros::Rate loop_rate(vx_pid_loop_rate);
	
std_msgs::Float64 va_msg;
std_msgs::Float64 dac_msg;		
std_msgs::Float64 vt_msg;

	while (ros::ok()) {
		
		time_t now;		
		time(&now);
		Vl_Vr_a_lock.lock();
		Vx_t_lock.lock();
		Alpha_lock.lock();
		double Vx_error = Vx_t - Vx_a;

		float vxtprinter,vxaprinter,vxerrorprinter; 
		vxtprinter=Vx_t;
		vxaprinter=Vx_a;
		vxerrorprinter=Vx_error;
		vt_msg.data=vxtprinter;
		va_msg.data=vxaprinter;
		Alpha_lock.unlock();
		Vx_t_lock.unlock();
		Vl_Vr_a_lock.unlock();
		
		
		va_pub.publish(va_msg);
		vt_pub.publish(vt_msg);

		float percerror=(vxerrorprinter/vxtprinter)*100.0;
		if(vxtprinter<0.01 && vxtprinter >-0.01)
		{
			percerror=100;
		}
		Vx_error_diff = Vx_error_old - Vx_error;
		Vx_error_integral +=  Vx_error*Ki_Vx;
		Vx_error_old = Vx_error;

		if (Vx_error_integral >= PWM_max_percent) {
			Vx_error_integral = PWM_max_percent;
		} else if (Vx_error_integral <= - PWM_max_percent) {
			Vx_error_integral = - PWM_max_percent;
		}

		if(difftimer()>0 && difftimer()<18000)
		{
			PWM_Duty_Cycle=1000;

		}
		else if(difftimer()>23000 && difftimer()<25000)
		{	

			PWM_Duty_Cycle=1900;

		}
		else if(difftimer()>25000 && difftimer()<27000)
		{	

			PWM_Duty_Cycle=1000+(difftimer()-20000)*0.1;

		}
		else if(difftimer()>27000 && difftimer()<31000)
		{

			PWM_Duty_Cycle=1700+200*sin(((difftimer()-27000)/4000.0)*6.28318);

		}
		else if(difftimer()>31000 && difftimer()<35000)
		{
			PWM_Duty_Cycle=1700+200*sin(2*((difftimer()-31000)/4000.0)*6.28318);

		}
		else if(difftimer()>35000 && difftimer()<39000)
		{

			PWM_Duty_Cycle=1700+200*sin(4*((difftimer()-35000)/4000.0)*6.28318);

		}
		else if(difftimer()>39000 && difftimer()<43000)
		{

			PWM_Duty_Cycle=1700+200*sin(8*((difftimer()-39000)/4000.0)*6.28318);

		}
		else if(difftimer()>45)
		{
			PWM_Duty_Cycle=1300;
			state=8;
		}
		

		if(std::signbit(Vy_t)) {
			Vx_error_integral = 0;
        } 

		PWM_Duty_Cycle = getMinMax(PWM_Duty_Cycle, PWM_max_percent, PWM_min_percent);
		printf("DAC Val=%lf",PWM_Duty_Cycle);
		dac_msg.data=PWM_Duty_Cycle;
		dac_pub.publish(dac_msg);
		int t,bit;

		

		printf(" G_PWM: %3.3f \n ", PWM_Duty_Cycle); 
		
		
		int value;		
		manualmode_lock.lock();
		if(manualmode==1)
		{
			value=1;
		}
		else if(manualmode==0)
		{
			value=0;
		}
		manualmode_lock.unlock();
		//fclose(file0);*/
		
		geometry_msgs::Twist msg;
		
		msg.linear.x=(int)PWM_Duty_Cycle;  // adding pwm value to x 
		msg.angular.y=value;// adding manual or automatic value to y

		pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
}

int main(int argc, char** argv) {
  
	VxPid * vxPid = new VxPid();

	vxPid->implementPid(argc, argv);
	
	delete vxPid;
}
