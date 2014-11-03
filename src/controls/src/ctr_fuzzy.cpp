#include "ctr_fuzzy.h"

bool start=true;
state::state init_state;
t_gainfuzzy gfuz;
geometry_msgs::Wrench F;

#include "fuzzy_core.cpp"

void initctr(const state::state state)
{
  init_state=state;
  I.pos[0]=0.0;
I.pos[1]=0.0;
I.pos[2]=0.0;
I.theta_x=0.0;
I.theta_y=0.0;
I.theta_z=0.0;
oE.pos[0]=0.0;
oE.pos[1]=0.0;
oE.pos[2]=0.0;
oE.theta_x=0.0;
oE.theta_y=0.0;
oE.theta_z=0.0;

 gfuz = {((double)FUZZY_TIME)/1000000, (double)MAX_ERROR_FORCE_XY, (double)MAX_ERROR_FORCE_Z, (double)MAX_ERROR_FORCE_TZ, (double)MAX_ERROR_FORCE_TXY, (double)ERROR_RANGE_XY, (double)ERROR_RANGE_Z, (double)ERROR_RANGE_TZ, (double)ERROR_RANGE_TXY, (double)MAX_INTEGRAL_FORCE_XY, (double)MAX_INTEGRAL_FORCE_Z, (double)MAX_INTEGRAL_FORCE_TZ, (double)MAX_INTEGRAL_FORCE_TXY, (double)INTEGRAL_RANGE_XY, (double)INTEGRAL_RANGE_Z, (double)INTEGRAL_RANGE_TZ, (double)INTEGRAL_RANGE_TXY, (double)MAX_INC_FORCE_XY, (double)MAX_INC_FORCE_Z, (double)MAX_INC_FORCE_TZ, (double)MAX_INC_FORCE_TXY, (double)INC_RANGE_XY, (double)INC_RANGE_Z, (double)INC_RANGE_TZ, (double)INC_RANGE_TXY, (double)INC_SLOPE_XY, (double)INC_SLOPE_Z, (double)INC_SLOPE_TZ, (double)INC_SLOPE_TXY, (double)MAX_DEC_FORCE_XY, (double)MAX_DEC_FORCE_Z, (double)MAX_DEC_FORCE_TZ, (double)MAX_DEC_FORCE_TXY, (double)DEC_RANGE_XY, (double)DEC_RANGE_Z, (double)DEC_RANGE_TZ, (double)DEC_RANGE_TXY, (double)DEC_SLOPE_XY, (double)DEC_SLOPE_Z, (double)DEC_SLOPE_TZ, (double)DEC_SLOPE_TXY, (double)ANTI_WINDUP_XY, (double)ANTI_WINDUP_Z, (double)ANTI_WINDUP_TZ, (double)ANTI_WINDUP_TXY};
}

void subState(const state::state state)
{

	// Zero compass//
	if(start){
    	rz0=rz;
	initctr(state);
    	start=false;}

	fuzzy_control(state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle node;
    ros::Publisher Controle_node = node.advertise<geometry_msgs::Wrench>("command_control",1);
    ros::Rate loop_rate(5);

	//ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
	ros::Subscriber subS = node.subscribe("state", 1, subState);
	while (ros::ok())
	{
/*        	geometry_msgs::Wrench wrenchMsg;
        	////////////////////////////////////
        	////       Controller           ////
        	////////////////////////////////////

		wrenchMsg.force.y=0;
		wrenchMsg.force.z=0.0012*(errorn+1.2*deriv);
      	  	wrenchMsg.torque.x=0;
        	wrenchMsg.torque.y=0;
    		wrenchMsg.torque.z=-0.012*errorz;
*/
		ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",wrenchMsg.force.x,wrenchMsg.force.y,wrenchMsg.force.z,wrenchMsg.torque.x,wrenchMsg.torque.y,wrenchMsg.torque.z);
       		 /////////////////////////////////
        	ROS_INFO("error z",errorz);
        	Controle_node.publish(wrenchMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



