#include "ctr_fuzzy.h"


bool start=true;
state::state init_state;
geometry_msgs::Wrench F;

double Ls=(double)CUBE_LENGTH;				// Cube lengh
state::state E;		// The error
state::state oE;		// The old error
state::state I;		// Integral of error
state::state D;		// Derivative of error
double wlimit=((double)DISTANCE_SECURITE)/100;	// The collision limit
double zlimit=((double)DISTANCE_SECURITEZ)/100;
t_gainfuzzy gfuz;

void initctr(const state::state state)
{
  init_state=state;
  I.pos[0]=0.0;
I.pos[1]=0.0;
I.pos[2]=0.0;
//I.theta_x=0.0;
//I.theta_y=0.0;
I.quat[0]=0.0;
oE.pos[0]=0.0;
oE.pos[1]=0.0;
oE.pos[2]=0.0;
//oE.theta_x=0.0;
//oE.theta_y=0.0;
oE.quat[0]=0.0;
 gfuz = {((double)1/5), (double)MAX_ERROR_FORCE_XY, (double)MAX_ERROR_FORCE_Z, (double)MAX_ERROR_FORCE_TZ, (double)MAX_ERROR_FORCE_TXY, (double)ERROR_RANGE_XY, (double)ERROR_RANGE_Z, (double)ERROR_RANGE_TZ, (double)ERROR_RANGE_TXY, (double)MAX_INTEGRAL_FORCE_XY, (double)MAX_INTEGRAL_FORCE_Z, (double)MAX_INTEGRAL_FORCE_TZ, (double)MAX_INTEGRAL_FORCE_TXY, (double)INTEGRAL_RANGE_XY, (double)INTEGRAL_RANGE_Z, (double)INTEGRAL_RANGE_TZ, (double)INTEGRAL_RANGE_TXY, (double)MAX_INC_FORCE_XY, (double)MAX_INC_FORCE_Z, (double)MAX_INC_FORCE_TZ, (double)MAX_INC_FORCE_TXY, (double)INC_RANGE_XY, (double)INC_RANGE_Z, (double)INC_RANGE_TZ, (double)INC_RANGE_TXY, (double)INC_SLOPE_XY, (double)INC_SLOPE_Z, (double)INC_SLOPE_TZ, (double)INC_SLOPE_TXY, (double)MAX_DEC_FORCE_XY, (double)MAX_DEC_FORCE_Z, (double)MAX_DEC_FORCE_TZ, (double)MAX_DEC_FORCE_TXY, (double)DEC_RANGE_XY, (double)DEC_RANGE_Z, (double)DEC_RANGE_TZ, (double)DEC_RANGE_TXY, (double)DEC_SLOPE_XY, (double)DEC_SLOPE_Z, (double)DEC_SLOPE_TZ, (double)DEC_SLOPE_TXY, (double)ANTI_WINDUP_XY, (double)ANTI_WINDUP_Z, (double)ANTI_WINDUP_TZ, (double)ANTI_WINDUP_TXY};
}

double isig(double value, double peak)			// Membership function, inverse triangle
{
double isigma;

   isigma=value/peak;

if (isigma>=0)
   isigma=1-isigma;
if (isigma<0)
   isigma=0; 
return isigma;
}

 
double sig(double value, double peak)			// Membership function Triangle zero at zero
{
double sigma;

   sigma=value/peak;
   
if (sigma<0)
   sigma=0;
   
if (sigma>1)
   sigma=1;
 
return sigma;
}

void fuzzy_control(const state::state state)
{
		// Update old error
		oE.pos[0]=E.pos[0];
		oE.pos[1]=E.pos[1];
		oE.pos[2]=E.pos[2];
		//oE.theta_x=E.theta_x;
		//oE.theta_y=E.theta_y;
		oE.quat[0]=E.quat[0];

		// Calculate the error
		E.pos[0]=init_state.pos[0]-state.pos[0];
		E.pos[1]=init_state.pos[1]-state.pos[1];
		E.pos[2]=init_state.pos[2]-state.pos[2];
		E.quat[0]=init_state.quat[0]-state.quat[0];
		
		/*...Integral of Error.................................................................*/

		I.pos[0]=I.pos[0]+E.pos[0]*gfuz.time;
		I.pos[1]=I.pos[1]+E.pos[1]*gfuz.time;
		I.pos[2]=I.pos[2]+E.pos[2]*gfuz.time;
		I.quat[0]=I.quat[0]+E.quat[0]*gfuz.time;
		
		// Anti-Windup
		if (I.pos[0]>ANTI_WINDUP_XY)
		   I.pos[0]=ANTI_WINDUP_XY;
		if (I.pos[0]<-ANTI_WINDUP_XY)
		   I.pos[0]=-ANTI_WINDUP_XY;
	   	if (I.pos[1]>ANTI_WINDUP_XY)
		   I.pos[1]=ANTI_WINDUP_XY;
		if (I.pos[1]<-ANTI_WINDUP_XY)
		   I.pos[1]=-ANTI_WINDUP_XY;
		if (I.pos[2]>ANTI_WINDUP_Z)
		   I.pos[2]=ANTI_WINDUP_Z;
		if (I.pos[2]<-ANTI_WINDUP_Z)
		   I.pos[2]=-ANTI_WINDUP_Z;   
/*		if (I.theta_x>gfuz->awinup_txy)
		   I.theta_x=gfuz->awinup_txy;
		if (I.theta_x<-gfuz->awinup_txy)
		   I.theta_x=-gfuz->awinup_txy;
		if (I.theta_y>gfuz->awinup_txy)
		   I.theta_y=gfuz->awinup_txy;
		if (I.theta_y<-gfuz->awinup_txy)
		   I.theta_y=-gfuz->awinup_txy;*/
		 if (I.quat[0]>ANTI_WINDUP_TZ)
		   I.quat[0]=ANTI_WINDUP_TZ;
		if (I.quat[0]<-ANTI_WINDUP_TZ)
		   I.quat[0]=-ANTI_WINDUP_TZ;     

		/*...Derivative of Error................................................................*/
		D.pos[0]=(E.pos[0]-oE.pos[0])/gfuz.time;
		D.pos[1]=(E.pos[1]-oE.pos[1])/gfuz.time;
		D.pos[2]=(E.pos[2]-oE.pos[2])/gfuz.time;
/*		D.theta_x=(E.theta_x-oE.theta_x)/time;
		D.theta_y=(E.theta_y-oE.theta_y)/time;*/
		D.quat[0]=(E.quat[0]-oE.quat[0])/gfuz.time;


		// Calculate force command, fuzzy style
		F.force.x=0;
		F.force.y=0;
		F.force.z=0;
		F.torque.x=0;
		F.torque.y=0;
		F.torque.z=0;
F.force.x=gfuz.mef_xy*(sig(E.pos[0], gfuz.exy)-sig(E.pos[0], -gfuz.exy))+gfuz.mixy*(sig(E.pos[0], gfuz.ixy)*sig(I.pos[0], gfuz.awinup_xy)-sig(E.pos[0], -gfuz.ixy)*sig(I.pos[0], -gfuz.awinup_xy))+gfuz.minc_xy*(sig(E.pos[0], gfuz.incxy)*sig(D.pos[0], gfuz.incsxy)-sig(E.pos[0], -gfuz.incxy)*sig(D.pos[0], -gfuz.incsxy))-gfuz.mdec_xy*(isig(E.pos[0], gfuz.decxy)*sig(D.pos[0], -gfuz.decsxy)-isig(E.pos[0], -gfuz.decxy)*sig(D.pos[0], gfuz.decsxy));
F.force.y=gfuz.mef_xy*(sig(E.pos[1], gfuz.exy)-sig(E.pos[1], -gfuz.exy))+gfuz.mixy*(sig(E.pos[1], gfuz.ixy)*sig(I.pos[1], gfuz.awinup_xy)-sig(E.pos[1], -gfuz.ixy)*sig(I.pos[1], -gfuz.awinup_xy))+gfuz.minc_xy*(sig(E.pos[1], gfuz.incxy)*sig(D.pos[1], gfuz.incsxy)-sig(E.pos[1], -gfuz.incxy)*sig(D.pos[1], -gfuz.incsxy))-gfuz.mdec_xy*(isig(E.pos[1], gfuz.decxy)*sig(D.pos[1], -gfuz.decsxy)-isig(E.pos[1], -gfuz.decxy)*sig(D.pos[1], gfuz.decsxy));
F.force.z=gfuz.mef_z*(sig(E.pos[2], gfuz.ez)-sig(E.pos[2], -gfuz.ez))+gfuz.miz*(sig(E.pos[2], gfuz.iz)*sig(I.pos[2], gfuz.awinup_z)-sig(E.pos[2], -gfuz.iz)*sig(I.pos[2], -gfuz.awinup_z))+gfuz.minc_z*(sig(E.pos[2], gfuz.incz)*sig(D.pos[2], gfuz.incsz)-sig(E.pos[2], -gfuz.incz)*sig(D.pos[2], -gfuz.incsz))-gfuz.mdec_z*(isig(E.pos[2], gfuz.decz)*sig(D.pos[2], -gfuz.decsz)-isig(E.pos[2], -gfuz.decz)*sig(D.pos[2], gfuz.decsz));
//F.Tx=-2*gfuz->mef_txy*(sig(E.theta_x, gfuz->etxy)-sig(E.theta_x, -gfuz->etxy));
//F.Tx=gfuz->mef_txy*(sig(E.theta_x, gfuz->etxy)-sig(E.theta_x, -gfuz->etxy))+gfuz->mitxy*(sig(E.theta_x, gfuz->itxy)*sig(I.theta_x, gfuz->awinup_txy)-sig(E.theta_x, -gfuz->itxy)*sig(I.theta_x, -gfuz->awinup_txy))+gfuz->minc_txy*(sig(E.theta_x, gfuz->inctxy)*sig(D.theta_x, gfuz->incstxy)-sig(E.theta_x, -gfuz->inctxy)*sig(D.theta_x, -gfuz->incstxy))-gfuz->mdec_txy*(isig(E.theta_x, gfuz->dectxy)*sig(D.theta_x, -gfuz->decstxy)-isig(E.theta_x, -gfuz->dectxy)*sig(D.theta_x, gfuz->decstxy));
//F.Ty=-2*gfuz->mef_txy*(sig(E.theta_y, etxy)-sig(E.theta_y, -etxy));
//F.Ty=gfuz->mef_txy*(sig(E.theta_y, gfuz->etxy)-sig(E.theta_y, -gfuz->etxy))+gfuz->mitxy*(sig(E.theta_y, gfuz->itxy)*sig(I.theta_y, gfuz->awinup_txy)-sig(E.theta_y, -gfuz->itxy)*sig(I.theta_y, -gfuz->awinup_txy))+gfuz->minc_txy*(sig(E.theta_y, gfuz->inctxy)*sig(D.theta_y, gfuz->incstxy)-sig(E.theta_y, -gfuz->inctxy)*sig(D.theta_y, -gfuz->incstxy))-gfuz->mdec_txy*(isig(E.theta_y, gfuz->dectxy)*sig(D.theta_y, -gfuz->decstxy)-isig(E.theta_y, -gfuz->dectxy)*sig(D.theta_y, gfuz->decstxy));
F.torque.z=gfuz.mef_tz*(sig(E.quat[0], gfuz.etz)-sig(E.quat[0], -gfuz.etz))+gfuz.mitz*(sig(E.quat[0], gfuz.itz)*sig(I.quat[0], gfuz.awinup_tz)-sig(E.quat[0], -gfuz.itz)*sig(I.quat[0], -gfuz.awinup_tz))+gfuz.minc_tz*(sig(E.quat[0], gfuz.inctz)*sig(D.quat[0], gfuz.incstz)-sig(E.quat[0], -gfuz.inctz)*sig(D.quat[0], -gfuz.incstz))-gfuz.mdec_tz*(isig(E.quat[0], gfuz.dectz)*sig(D.quat[0], -gfuz.decstz)-isig(E.quat[0], -gfuz.dectz)*sig(D.quat[0], gfuz.decstz));
	
} 

void subState(const state::state state)
{

	if(start){
//    	rz0=rz;
	initctr(state);
    	start=false;}

	fuzzy_control(state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tryphon241_control");
    ros::NodeHandle node;
    ros::Publisher Controle_node = node.advertise<geometry_msgs::Wrench>("/tryphon241/command_control",1);
    ros::Rate loop_rate(5); //CHANGE TIME OF FUZZY CONTROL IF DIFF zOF 5H

	ros::Subscriber subS = node.subscribe("/tryphon241/state", 1, subState);
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
		ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",F.force.x, F.force.y, F.force.z, F.torque.x, F.torque.y, F.torque.z);
       		 /////////////////////////////////
        	Controle_node.publish(F);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


