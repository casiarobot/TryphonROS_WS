#include "imu_robo/ahrs.h"

imu::Quaternion q;
imu::Quaternion offset;
imu::Quaternion body;

volatile float beta = 0.1;
unsigned long last_millis = 0;

void MadgwickAHRSupdate(imu::Vector<3> g, imu::Vector<3> a, imu::Vector<3> m, float dt);


void uimu_ahrs_init(imu::Vector<3> acc, imu::Vector<3> mag)
{
    imu::Vector<3> down = acc;
    //imu::Vector<3> east = down.cross(mag);
    //imu::Vector<3> north = east.cross(down);
    imu::Vector<3> north(sqrt(mag.x()*mag.x() + mag.y()*mag.y()),
                         0,
                         mag.z());
    imu::Vector<3> east = down.cross(north);
    //imu::Vector<3> north = east.cross(down);

    down.normalize();
    east.normalize();
    north.normalize();

    imu::Matrix<3> m;
    m.vector_to_row(north, 0);
    m.vector_to_row(east, 1);
    m.vector_to_row(down, 2);

    q.fromMatrix(m);
    q.normalize();
    ROS_INFO("current : %3.3f,%3.3f,%3.3f,%3.3f",
             q.x(),q.y(),q.z(),q.w());
    ROS_INFO("north : %3.3f,%3.3f,%3.3f",
             north.x(),north.y(),north.z());
    //q = imu::Quaternion(q.x(),q.y(),q.z(),-q.w());
    //q = imu::Quaternion(-0.02,0.82,0.56,-0.005);
    last_millis = millis();
}


void  uimu_ahrs_set_offset(imu::Quaternion o)
{
    offset = o;
}

void uimu_ahrs_set_beta(float b)
{
    beta = b;
}

float millis(void) {
  timeval curTime;
  gettimeofday(&curTime, NULL);
  float mil=(float)curTime.tv_usec/1000.0;
  return mil;
}
    
void uimu_ahrs_iterate(imu::Vector<3> ang_vel, imu::Vector<3> acc, imu::Vector<3> mag, bool useIMU)
{
    float dt = millis() - last_millis;
    last_millis = millis();

    dt /= 1000.0;

    //if(dt == 0)
    //    return;
   // ROS_INFO("dt= %3.5f", dt);
    dt = 0.00500;

    ang_vel.toRadians();

    //ang_vel[0] = 1.0 / ang_vel[0]; ang_vel[1] = 1.0 / ang_vel[1]; ang_vel[2] = 1.0 / ang_vel[2];

    if(useIMU)
        MadgwickIMUupdate(ang_vel, acc);
    else
        MadgwickAHRSupdate(ang_vel, acc, mag, dt);

/*
	imu::Vector<3> correction;
	imu::Vector<3> down = acc;
    imu::Vector<3> east = down.cross(mag);
	imu::Vector<3> north = east.cross(down);

	down.normalize();
	east.normalize();
	north.normalize();

	imu::Matrix<3> rotationMatrix = q.toMatrix();
	correction = (
				north.cross(rotationMatrix.row_to_vector(0)) +
				east.cross(rotationMatrix.row_to_vector(1)) +
				down.cross(rotationMatrix.row_to_vector(2))
					) * beta;


	imu::Vector<3> w = ang_vel + correction;

	imu::Quaternion n(1, w.x()*dt/2.0, w.y()*dt/2.0, w.z()*dt/2.0);

    q = q*n;
*/
    q.normalize();

    body = q * offset.conjugate();
}


imu::Vector<3> uimu_ahrs_get_euler()
{
    imu::Vector<3> euler = body.toEuler();
    euler.toDegrees();
    return euler;
}

imu::Matrix<3> uimu_ahrs_get_matrix()
{
    return body.toMatrix();
}

imu::Quaternion uimu_ahrs_get_quaternion()
{
    return body;
}

imu::Quaternion uimu_ahrs_get_imu_quaternion()
{
	return q;
}

#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f		// half the sample period
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error

void MadgwickIMUupdate(imu::Vector<3> g, imu::Vector<3> a) {
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	// normalise the measurements
	norm = sqrt(a.x()*a.x() + a.y()*a.y() + a.z()*a.z());
	a.x() = a.x() / norm;
	a.y() = a.y() / norm;
	a.z() = a.z() / norm;

	// estimated direction of gravity
	vx = 2*(q.x()*q.z() - q.w()*q.y());
	vy = 2*(q.w()*q.x() + q.y()*q.z());
	vz = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();

	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (a.y()*vz - a.z()*vy);
	ey = (a.z()*vx - a.x()*vz);
	ez = (a.x()*vy - a.y()*vx);

	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	// adjusted g.y()roscope measurements
	g.x() = g.x() + Kp*ex + exInt;
	g.y() = g.y() + Kp*ey + eyInt;
	g.z() = g.z() + Kp*ez + ezInt;

	// integrate quaternion rate and normalise
	q.w() = q.w() + (-q.x()*g.x() - q.y()*g.y() - q.z()*g.z())*halfT;
	q.x() = q.x() + (q.w()*g.x() + q.y()*g.z() - q.z()*g.y())*halfT;
	q.y() = q.y() + (q.w()*g.y() - q.x()*g.z() + q.z()*g.x())*halfT;
	q.z() = q.z() + (q.w()*g.z() + q.x()*g.y() - q.y()*g.x())*halfT;

	// normalise quaternion
	norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
	q.w() = q.w() / norm;
	q.x() = q.x() / norm;
	q.y() = q.y() / norm;
	q.z() = q.z() / norm;
}


void MadgwickAHRSupdate(imu::Vector<3> g, imu::Vector<3> a, imu::Vector<3> m, float dt)
{
    imu::Vector<4> s;
    imu::Vector<4> qDot;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    if(isnan(m.magnitude()) || isinf(m.magnitude())) 
        return;

    qDot[0] = 0.5f * (-q.y() * g.x() - q.y() * g.y() - q.z() * g.z());
    qDot[1] = 0.5f * (q.w() * g.x() + q.y() * g.z() - q.z() * g.y());
    qDot[2] = 0.5f * (q.w() * g.y() - q.x() * g.z() + q.z() * g.x());
    qDot[3] = 0.5f * (q.w() * g.z() + q.x() * g.y() - q.y() * g.x());

    float recipNorm;
    if(!(isnan(a.magnitude()))) 
	{
        a.normalize();
        m.normalize();

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q.w() * m.x();
        _2q0my = 2.0f * q.w() * m.y();
        _2q0mz = 2.0f * q.w() * m.z();
        _2q1mx = 2.0f * q.x() * m.x();
        _2q0 = 2.0f * q.w();
        _2q1 = 2.0f * q.x();
        _2q2 = 2.0f * q.y();
        _2q3 = 2.0f * q.z();
        _2q0q2 = 2.0f * q.w() * q.y();
        _2q2q3 = 2.0f * q.y() * q.z();
        q0q0 = q.w() * q.w();
        q0q1 = q.w() * q.x();
        q0q2 = q.w() * q.y();
        q0q3 = q.w() * q.z();
        q1q1 = q.x() * q.x();
        q1q2 = q.x() * q.y();
        q1q3 = q.x() * q.z();
        q2q2 = q.y() * q.y();
        q2q3 = q.y() * q.z();
        q3q3 = q.z() * q.z();

        // Reference direction of Earth's magnetic field
        hx = m.x() * q0q0 - _2q0my * q.z() + _2q0mz * q.y() + m.x() * q1q1 + _2q1 * m.y() * q.y() + _2q1 * m.z() * q.z() - m.x() * q2q2 - m.x() * q3q3;
        hy = _2q0mx * q.z() + m.y() * q0q0 - _2q0mz * q.x() + _2q1mx * q.y() - m.y() * q1q1 + m.y() * q2q2 + _2q2 * m.z() * q.z() - m.y() * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q.y() + _2q0my * q.x() + m.z() * q0q0 + _2q1mx * q.z() - m.z() * q1q1 + _2q2 * m.y() * q.z() - m.z() * q2q2 + m.z() * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s[0] = -_2q2 * (2.0f * q1q3 - _2q0q2 - a.x()) + _2q1 * (2.0f * q0q1 + _2q2q3 - a.y()) - _2bz * q.y() * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x()) + (-_2bx * q.z() + _2bz * q.x()) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y()) + _2bx * q.y() * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z());

        s[1] = _2q3 * (2.0f * q1q3 - _2q0q2 - a.x()) + _2q0 * (2.0f * q0q1 + _2q2q3 - a.y()) - 4.0f * q.x() * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z()) + _2bz * q.z() * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x()) + (_2bx * q.y() + _2bz * q.w()) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y()) + (_2bx * q.z() - _4bz * q.x()) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z());

        s[2] = -_2q0 * (2.0f * q1q3 - _2q0q2 - a.x()) + _2q3 * (2.0f * q0q1 + _2q2q3 - a.y()) - 4.0f * q.y() * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z()) + (-_4bx * q.y() - _2bz * q.w()) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x()) + (_2bx * q.x() + _2bz * q.z()) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y()) + (_2bx * q.w() - _4bz * q.y()) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z());

        s[3] = _2q1 * (2.0f * q1q3 - _2q0q2 - a.x()) + _2q2 * (2.0f * q0q1 + _2q2q3 - a.y()) + (-_4bx * q.z() + _2bz * q.x()) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x()) + (-_2bx * q.w() + _2bz * q.y()) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y()) + _2bx * q.x() * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z());

        s.normalize();
        // Apply feedback step
        qDot[0] -= beta * s[0];
        qDot[1] -= beta * s[1];
        qDot[2] -= beta * s[2];
        qDot[3] -= beta * s[3];
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.w() += qDot[0] * dt;
    q.x() += qDot[1] * dt;
    q.y() += qDot[2] * dt;
    q.z() += qDot[3] * dt;

}

int instability_fix = 1;

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    if (instability_fix == 0)
    {
        /* original code */
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
    else if (instability_fix == 1)
    {
        /* close-to-optimal method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
        unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
        float tmp = *(float*)&i;
        return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
    }
    else
    {
        /* optimal but expensive method: */
        return 1.0f / sqrtf(x);
    }
}

