#include <ros/ros.h>

//#include <mavros/Sonar.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#define MAX_RANGE 40
#define MIN_RANGE 0.5

using namespace std;

//---Get parameters
void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}
//---

class RangefusionClass
{

public:
	RangefusionClass(ros::NodeHandle& node)
	{

		n_ = node;

		//subscribers
		subDistanceMiddle = n_.subscribe("/distance_middle", 10, &RangefusionClass::readDistanceMiddle,this);
		subDistanceForward = n_.subscribe("/distance_forward", 10, &RangefusionClass::readDistanceForward,this);
		subDistanceBackward = n_.subscribe("/distance_backward", 10, &RangefusionClass::readDistanceBackward,this);
		subVelocity = n_.subscribe("/mavros/local_position/velocity", 10, &RangefusionClass::readVelocities,this);
		subAttitude = n_.subscribe("/mavros/local_position/pose", 10, &RangefusionClass::readPose,this);

		// publishers
		pubFusedDistance = n_.advertise<sensor_msgs::Range>("/mavros/distance_sensor/laser_1_sub", 10);
		//pubFusedDistance = n_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

		// initialization
		rate = 20;
		_distanceMiddle = 0;
		_distanceForward = 0;
		_distanceBackward = 0;

		//fused_distance.pose.position.z = 0;		//EV
		fused_distance.range = 0;		//distance
		fused_distance.min_range = 0.1;
		fused_distance.max_range = 40.0;

		load_param( _correct_vel, true, "correct_vel" );
		load_param( _sens_orientation, 1.05, "sens_orient" );
		load_param( _k, 0.8, "gain_vel" );
	}

	void readDistanceMiddle(const sensor_msgs::Range::ConstPtr& msg)
	{
		_distanceMiddle = msg->range;
	}

	void readDistanceForward(const sensor_msgs::Range::ConstPtr& msg)
	{
		_distanceForward = msg->range;
	}

	void readDistanceBackward(const sensor_msgs::Range::ConstPtr& msg)
	{
		_distanceBackward = msg->range;
	}

	void readVelocities(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		_vx = msg->twist.linear.x;
	}

	void readPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		//TODO
		// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);

		// the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(_roll, _pitch, _yaw);
		
		//Pitch positive forward
		//ROS_INFO("Pitch: %.3f",_pitch);
	}

	void main()
	{
		float d = 0;		//minimum distance from center to plane (line) estimated with two range sensor
		if (_distanceForward >= MAX_RANGE && _distanceBackward >= MAX_RANGE && _distanceMiddle >= MAX_RANGE){
			//All measurements are out of range
			d = -1;
			ROS_WARN("No good distances: %.3f, %.3f, %.3f",_distanceBackward,_distanceMiddle,_distanceForward);
			return;
		} else {
			ROS_INFO("Distances: %.3f, %.3f, %.3f",_distanceBackward,_distanceMiddle,_distanceForward);
			bool forward_only_good, center_only_good, backward_only_good;
			forward_only_good = (_distanceForward < MAX_RANGE && _distanceBackward >= MAX_RANGE && _distanceMiddle >= MAX_RANGE);
			center_only_good = (_distanceForward >= MAX_RANGE && _distanceBackward >= MAX_RANGE && _distanceMiddle < MAX_RANGE);
			backward_only_good = (_distanceForward >= MAX_RANGE && _distanceBackward < MAX_RANGE && _distanceMiddle >= MAX_RANGE);
			if (forward_only_good || center_only_good || backward_only_good){
				if (forward_only_good){
					d = _distanceForward;
				} else if (center_only_good){
					d = _distanceMiddle;
				} else {
					d = _distanceBackward;
				}
				fused_distance.header.stamp = ros::Time::now();
				fused_distance.range = d;					//distance for distance_sensor message		//distance
				ROS_WARN("Only one good: D: %.3f",d);			//distance
				pubFusedDistance.publish(fused_distance);
			} else {
				//At least two measurements are good
				double min = std::min(_distanceForward, _distanceBackward);
				double m,b;	//Line coefficients
				if (min == _distanceForward){
					calculate_plane(_distanceMiddle, _distanceForward, true, &m, &b);
					//ROS_INFO("Forward. Distances: %.3f, %.3f.",_distanceMiddle,_distanceForward);
				} else {
					calculate_plane(_distanceMiddle, _distanceBackward, false, &m, &b);
					//ROS_INFO("Forward. Distances: %.3f, %.3f.",_distanceMiddle,_distanceBackward);
				}
				
				if (_correct_vel){
					//Correct using the forward velocity
					double theta = atan(m);
					double ix,iy;
					ix = cos(theta);
					iy = sin(theta);
					vector<double> p0(2);
					vector<double> p1(2);
					
					p0[0] = -(b*m)/(pow(m, 2.0)+1);
					p0[1] = b/(pow(m, 2.0)+1);
					
					p1[0] = p0[0] + _k*_vx*ix;		//p'. Point p moved along the plane/line proportional to the forward/backward velocity
					p1[1] = p0[1] + _k*_vx*iy;
					
					//ROS_INFO("[VEL] P0: %.2f - %.2f. P1: %.2f - %.2f",p0[0],p0[1],p1[0],p1[1]);
					double h_a,h_b,h_c;		//ax+by+c = 0; line of the horizon in Drone frame
					h_a = sin(-_pitch);
					h_b = cos(-_pitch);
					h_c = 0;
					
					//Calculate new distance as distance p' B' or distance p' projected into the horizon line h
					d = distance_point_line(h_a, h_b, h_c, p1);
					double temp_d = sqrt(pow(b, 2.0)/(pow(m, 2.0)+1.0));	//Distance to line
					ROS_INFO("[VEL] Plane m: %.3f b: %.3f. D: %.2f. D2: %.2f",m,b,d,temp_d);
				} else {
					d = sqrt(pow(b, 2.0)/(pow(m, 2.0)+1.0));	//Distance to line
					ROS_INFO("Plane m: %.3f b: %.3f. D: %.3f",m,b,d);			//distance
				}
				
				fused_distance.header.stamp = ros::Time::now();
				fused_distance.range = d;					//distance for distance_sensor message		//distance
				
				pubFusedDistance.publish(fused_distance);
			}
		}
		
	}

	void line_two_points(vector<double> p1, vector<double> p2, double *m, double *b){
		double m_temp, b_temp;
		//it should never happen that x1==x2
		m_temp = (p2[1]-p1[1])/(p2[0]-p1[0]);
		b_temp = p1[1]-m_temp*p1[0];
		*m = m_temp;
		*b = b_temp;
	}

	double distance_point_line(double a, double b, double c, vector<double> p){
		double d;
		//ROS_INFO("[DIST] A: %.4f B: %.4f C: %.4f. P: %.2f - %.2f",a,b,c,p[0],p[1]);
		d = std::abs(a*p[0]+b*p[1]+c)/(sqrt(pow(a, 2.0)+pow(b, 2.0)));
		return d;
	}

	void calculate_plane(double d1, double d2, bool forward, double *m, double *b){
		//forward true --> forward and middle; type false --> backward and middle
		//d1 always the middle measurement
		//d1 forward or backward depending on the closest
		double x1,x2,y1,y2;
		vector<double> p1(2);
		vector<double> p2(2);

		/*x1 = 0;
		y1 = -d1;
		if (forward){
			x2 = 0.2 + d2*sin((double)1.05);
			y2 = -d2*cos((double)1.05);
		} else {
			x2 = -0.2 - d2*sin((double)1.05);
			y2 = -d2*cos((double)1.05);
		}
		//it should never happen that x1==x2
		double m_temp, b_temp;
		m_temp = (y2-y1)/(x2-x1);
		b_temp = y1-m_temp*x1;
		*m = m_temp;
		*b = b_temp;*/
		p1[0] = 0;
		p1[1] = -d1;
		if (forward){
			p2[0] = 0.2 + d2*sin(_sens_orientation);
			p2[1] = -d2*cos(_sens_orientation);
		} else {
			p2[0] = -0.2 - d2*sin(_sens_orientation);
			p2[1] = -d2*cos(_sens_orientation);
		}
		double m_temp, b_temp;
		line_two_points(p1, p2, &m_temp, &b_temp);
		*m = m_temp;
		*b = b_temp;
	}


	void run()
	{
		//ros::spin();
		ros::Rate loop_rate(rate);
		while (ros::ok())
		{
			ROS_INFO_ONCE("POS_MIXER: RUNNING");

			main();
			ros::spinOnce();

			loop_rate.sleep();
		}
	}

protected:
	/*state here*/
	ros::NodeHandle n_;

	// subscriber
	ros::Subscriber subDistanceMiddle;
	ros::Subscriber subDistanceForward;
	ros::Subscriber subDistanceBackward;
	ros::Subscriber subVelocity;
	ros::Subscriber subAttitude;

	// publisher
	ros::Publisher pubFusedDistance;

	double _distanceMiddle;
	double _distanceForward;
	double _distanceBackward;
	//geometry_msgs::PoseStamped fused_distance;	//EV
	sensor_msgs::Range fused_distance;			//distance
	double _vx;
	double _roll;
	double _pitch;
	double _yaw;
	double _k;

	bool _correct_vel;
	double _sens_orientation;

	int rate;
	bool use_global_altitude;
	const double alpha_lp = 0.2;	//LP filter
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DistanceFusion");
	ros::NodeHandle node;

	RangefusionClass RangefusionNode(node);

	RangefusionNode.run();
	return 0;
}
