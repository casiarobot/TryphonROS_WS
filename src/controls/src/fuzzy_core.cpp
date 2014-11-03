/************************************************************************************
File:			fuzzy_core.c
Description:	The fuzzy control thread
Author:			Stefan Bracher
History:		14.01.2008	Created by Stefan Bracher & David St-Onge
History:		10.14.2014	Adapted for ROS by David St-Onge
*************************************************************************************/

double Ls=(double)CUBE_LENGTH;				// Cube lengh
state::state E;		// The error
state::state oE;		// The old error
state::state I;		// Integral of error
state::state D;		// Derivative of error
double wlimit=((double)DISTANCE_SECURITE)/100;	// The collision limit
double zlimit=((double)DISTANCE_SECURITEZ)/100;

//int count=0;								// A counter variable

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
// Get the pointers to the variables transmitted through ptr
//t_thread_control_arguments *arg=(t_thread_control_arguments *)ptr;
//t_position *position_now=arg->position_now;
//t_position *position_des=arg->position_des;
//int *control=arg->central_control;
//t_gainfuzzy *gfuz=arg->gfuz;

// "Global" variables for this thread
//t_position position_des_save;
//t_position E;		// The error
//t_position oE;		// The old error
//t_position I;		// Integral of error
//t_position D;		// Derivative of error
//t_forces F;

//double time=gfuz->time;//((double)FUZZY_TIME)/1000000;			// Loop time in s

//double mef_xy=(double)MAX_ERROR_FORCE_XY;
//double mef_z=(double)MAX_ERROR_FORCE_Z;
//double mef_tz=(double)MAX_ERROR_FORCE_TZ;
//double mef_txy=(double)MAX_ERROR_FORCE_TXY;
//double exy=(double)ERROR_RANGE_XY;
//double ez=(double)ERROR_RANGE_Z;
//double etz=(double)ERROR_RANGE_TZ;
//double etxy=(double)ERROR_RANGE_TXY;
//
//double mixy=(double)MAX_INTEGRAL_FORCE_XY;
//double miz=(double)MAX_INTEGRAL_FORCE_Z;
//double mitz=(double)MAX_INTEGRAL_FORCE_TZ;
//double mitxy=(double)MAX_INTEGRAL_FORCE_TXY;
//double ixy=(double)INTEGRAL_RANGE_XY;
//double iz=(double)INTEGRAL_RANGE_Z;
//double itz=(double)INTEGRAL_RANGE_TZ;
//double itxy=(double)INTEGRAL_RANGE_TXY;
//
//
//double minc_xy=(double)MAX_INC_FORCE_XY;
//double minc_z=(double)MAX_INC_FORCE_Z;
//double minc_tz=(double)MAX_INC_FORCE_TZ;
//double minc_txy=(double)MAX_INC_FORCE_TXY;
//double incxy=(double)INC_RANGE_XY;
//double incz=(double)INC_RANGE_Z;
//double inctz=(double)INC_RANGE_TZ;
//double inctxy=(double)INC_RANGE_TXY;
//double incsxy=(double)INC_SLOPE_XY;
//double incsz=(double)INC_SLOPE_Z;
//double incstz=(double)INC_SLOPE_TZ;
//double incstxy=(double)INC_SLOPE_TXY;
//
//double mdec_xy=(double)MAX_DEC_FORCE_XY;
//double mdec_z=(double)MAX_DEC_FORCE_Z;
//double mdec_tz=(double)MAX_DEC_FORCE_TZ;
//double mdec_txy=(double)MAX_DEC_FORCE_TXY;
//double decxy=(double)DEC_RANGE_XY;
//double decz=(double)DEC_RANGE_Z;
//double dectz=(double)DEC_RANGE_TZ;
//double dectxy=(double)DEC_RANGE_TXY;
//double decsxy=(double)DEC_SLOPE_XY;
//double decsz=(double)DEC_SLOPE_Z;
//double decstz=(double)DEC_SLOPE_TZ;
//double decstxy=(double)DEC_SLOPE_TXY;

//*position_now=sensors2position(*position_des, Ls);	// Get actual position

// Initialisation of the variables
//I.x=0.0;
//I.y=0.0;
//I.z=0.0;
//I.theta_x=0.0;
//I.theta_y=0.0;
//I.theta_z=0.0;
//oE.x=0.0;
//oE.y=0.0;
//oE.z=0.0;
//oE.theta_x=0.0;
//oE.theta_y=0.0;
//oE.theta_z=0.0;

//usleep(1000000);
//printf("\nLaunch Fuzzy controller!");

//while (1)
//{
// 	if (*control==1)
// 	{
		//printf("\nCentral control started!");
		// Update old error
		oE.pos[0]=E.pos[0];
		oE.pos[1]=E.pos[1];
		oE.pos[2]=E.pos[2];
		//oE.theta_x=E.theta_x;
		//oE.theta_y=E.theta_y;
		oE.quat[0]=E.quat[0];
		   		
	
		// Get actual position
		//*position_now=sensors2position(*position_des, Ls);
	 	// Check for collisions
		//position_des_save=safety(*position_des);
		
		//position_des_save=*position_des;

		// Calculate the error
		//E.x=position_des_save.x-position_now->x;
		//E.y=position_des_save.y-position_now->y;
		//E.z=position_des_save.z-position_now->z;
		//E.theta_x=position_des_save.theta_x-position_now->theta_x;
		//E.theta_y=position_des_save.theta_y-position_now->theta_y;
		//E.theta_z=position_des_save.theta_z-position_now->theta_z;
		E.pos[0]=init_state.pos[0]-state.pos[0];
		E.pos[1]=init_state.pos[1]-state.pos[1];
		E.pos[2]=init_state.pos[2]-state.pos[2];
		E.quat[0]=init_state.quat[0]-state.quat[0];
		
		//Disable wall if orientation isn't good.
/*		if(abs(E.theta_z)>0.7){
		  E.x=0;
		  E.y=0;
		}
*/		  
/*				
		// Partially disable controller if needed
		if (position_des->x_ref==0)
		{   		
		E.x=0;
		oE.x=0;
		I.x=0;
		D.x=0;   				
		} 
	
		if (position_des->y_ref==0)
		{   		
		E.y=0;
		oE.y=0;
		I.y=0;
		D.y=0;   				
		}

		if (position_des->z_ref==0)
		{   		
		E.z=0;
		oE.z=0;
		I.z=0;
		D.z=0;   				
		}
		*/
		/*...Integral of Error.................................................................*/
		/*I.x=I.x+E.x*time;
		I.y=I.y+E.y*time;
		I.z=I.z+E.z*time;
		I.theta_x=I.theta_x+E.theta_x*time;
		I.theta_y=I.theta_y+E.theta_y*time;
		I.theta_z=I.theta_z+E.theta_z*time;*/
		I.pos[0]=I.pos[0]+E.pos[0]*time;
		I.pos[1]=I.pos[1]+E.pos[1]*time;
		I.pos[2]=I.pos[2]+E.pos[2]*time;
		I.quat[0]=I.quat[0]+E.quat[0]*time;
		
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
		D.pos[0]=(E.pos[0]-oE.pos[0])/time;
		D.pos[1]=(E.pos[1]-oE.pos[1])/time;
		D.pos[2]=(E.pos[2]-oE.pos[2])/time;
/*		D.theta_x=(E.theta_x-oE.theta_x)/time;
		D.theta_y=(E.theta_y-oE.theta_y)/time;*/
		D.quat[0]=(E.quat[0]-oE.quat[0])/time;

		// Eliminate error due to unwanted states
/*		if (position_des->x_ref==0)
		{
		E.x=0;	
		oE.x=0;
		I.x=0;	
		D.x=0;	
		}

		if (position_des->y_ref==0)
		{
		E.y=0;	
		oE.y=0;
		I.y=0;	
		D.y=0;	
		}

		if (position_des->z_ref==0)
		{
		E.z=0;	double isig(double value, double peak)			// Membership function, inverse triangle
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
		oE.z=0;
		I.z=0;	
		D.z=0;	
		}

		if (position_des->theta_x_ref==0)
		{
		E.theta_x=0;	
		oE.theta_x=0;
		I.theta_x=0;	
		D.theta_x=0;	
		}

		if (position_des->theta_y_ref==0)
		{
		E.theta_y=0;	
		oE.theta_y=0;
		I.theta_y=0;	
		D.theta_y=0;	
		}

		if (position_des->theta_z_ref==0)
		{
		E.theta_z=0;	
		oE.theta_z=0;
		I.theta_z=0;	
		D.theta_z=0;	
		}*/


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


//printf("\n F.Tz %f", F.Tz);
//		forces2motor(F, Ls);					// Translate force commands to motor commands
	
	/*...Screen outputs.......................................................................*/
//	count++;
//	if (count>10)
//		{
//		count=0;
		//if(debug) printf("\nEx: %f \t Fx:  %f \t Ey: %f \t Fy: %f \t Ez: %f \t Fz: %f \t Etz: %f \t Tz: %f", E.x, F.Fx, E.y, F.Fy, E.z, F.Fz, E.theta_z, F.Tz);
		//printf("\n%3.3f;%3.3f;%3.3f;%3.3f;%3.3f;%3.3f;%3.3f;%3.3f", position_now->x, position_des->x, position_now->y, position_des->y, position_now->z, position_des->z, position_now->theta_z, position_des->theta_z);
//		}	
	
	
//	}
//	usleep(time*1000000);//FUZZY_TIME);
	
//}
	
} 
