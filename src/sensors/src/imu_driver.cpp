#include "imu_robo/imu_driver.h"

ImuDriver::ImuDriver(bool useIMU):
    useIMU(useIMU),
    loopRate(this->LOOP_RATE){
    this->createPublishers();
}

void ImuDriver::createPublishers(){
    this->pubImu = this->nodeHandle.advertise<sensor_msgs::Imu>("/imu", 1);
    this->pubMag = this->nodeHandle.advertise<sensor_msgs::Imu>("/imuMag", 1);
    this->pubPose = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/pose", 1);
}


void ImuDriver::init(){
    if(robovero.Init()==-1){
        exit(0);
    }

    robovero.GYROzeroCalibrate(128, 2);
    robovero.mag_calibration();

    this->updateSensorData();

    uimu_ahrs_init(a, m);
    uimu_ahrs_set_beta(2.5);

    usleep(200);

    this->imuZeroCalibration();
}

void ImuDriver::updateSensorData(){
    robovero.updateIMUdata();
    this->a = Robovero.a;
    this->g = Robovero.g;
    this->m = Robovero.m;
}

void ImuDriver::imuZeroCalibration(){
    for(int i =0; i < LOOP_RATE && ros::ok(); i++){
        robovero.updateIMUdata();
        uimu_ahrs_iterate(this->g, this->a, this->m, this->useIMU);

        ros::spinOnce();
        this->loopRate.sleep();
    }
    ROS_WARN("Set new beta");
    uimu_ahrs_set_beta(0.2);
    uimu_ahrs_set_offset(uimu_ahrs_get_imu_quaternion());
}

void ImuDriver::loop(){
    //int g =0;
    while(ros::ok()){
       // Robovero.test_motor(38);
       //g++;
        //if(g > 100){
            //ROS_INFO("Test I2C:");
            //Robovero.scanAllI2C();
            //Robovero.setMotor((int)(0xB0)/2,70,112);
       // }
        //else
       //    Robovero.setMotor((int)(0xB0)/2,0,112);
        //if(g > 200)
        //    g=0;
        this->updateSensorData();
        uimu_ahrs_iterate(this->g, this->a, this->m, this->useIMU);

        this->rotation = uimu_ahrs_get_quaternion();

        this->pubImu.publish(this->generateImuMessage());
        this->pubPose.publish(this->generatePoseMessage());
        this->pubMag.publish(this->generateMagMessage());

        ros::spinOnce();
        this->loopRate.sleep();
    }
}

sensor_msgs::Imu ImuDriver::generateImuMessage(){
    sensor_msgs::Imu msg;
    msg.header.frame_id = "base_link";
    msg.orientation = this->orientationConvertToROSMessage();

    msg.angular_velocity.x = this->g[0];
    msg.angular_velocity.y = this->g[1];
    msg.angular_velocity.z = this->g[2];

    msg.linear_acceleration.x = this->a[0];
    msg.linear_acceleration.y = this->a[1];
    msg.linear_acceleration.z = this->a[2];
    return msg;
}

geometry_msgs::PoseStamped ImuDriver::generatePoseMessage(){
    geometry_msgs::PoseStamped msg;
    msg.pose.orientation = this->orientationConvertToROSMessage();
    msg.header.frame_id = "base_link";
    return msg;
}

geometry_msgs::Quaternion ImuDriver::orientationConvertToROSMessage(){
    geometry_msgs::Quaternion q;
    q.x = this->rotation.x();
    q.y = this->rotation.y();
    q.z = this->rotation.z();
    q.w = this->rotation.w();
    return q;
}


sensor_msgs::Imu ImuDriver::generateMagMessage(){
    sensor_msgs::Imu msg;
    msg.header.frame_id = "base_link";
    msg.linear_acceleration.x = this->m[0];
    msg.linear_acceleration.y = this->m[1];
    msg.linear_acceleration.z = this->m[2];

    return msg;
}

