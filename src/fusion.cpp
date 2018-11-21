#include <ros/ros.h>

//#include <mavros/Sonar.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>

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
		//TODO sub to speed to calculate the point prediction?

		// publishers
		//pubFusedDistance = n_.advertise<sensor_msgs::Range>("/mavros/distance_sensor/laser_1_sub", 10);
		pubFusedDistance = n_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

		// initialization
		rate = 20;
		_distanceMiddle = 0;
		_distanceForward = 0;
		_distanceBackward = 0;

		//fused_distance.radiation_type = sensor_msgs::Range::INFRARED;
		//fused_distance.min_range = 0.5;
		//fused_distance.max_range = 40.0;
		fused_distance.pose.position.z = 0;
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

	void main()
	{
		float min = std::min(_distanceForward, _distanceBackward);
		float m,b;
		if (min == _distanceForward){
			calcualte_plane(_distanceMiddle, _distanceForward, true, &m, &b);
			ROS_INFO("Forward. Distances: %.3f, %.3f. Plane m: %.3f b: %.3f. D: %.3f",_distanceMiddle,_distanceForward);
		} else {
			calcualte_plane(_distanceMiddle, _distanceBackward, false, &m, &b);
			ROS_INFO("Forward. Distances: %.3f, %.3f. Plane m: %.3f b: %.3f. D: %.3f",_distanceMiddle,_distanceBackward);
		}
		fused_distance.header.stamp = ros::Time::now();
		//fused_distance.range = _distanceMiddle;					//distance for distance_sensor message
		float d = sqrt(pow(b, 2.0)/(pow(m, 2.0)+1.0));		//minimum distance from center sensor to plane

		fused_distance.pose.position.z = d;			//pose for mocap message
		ROS_INFO("Plane m: %.3f b: %.3f. D: %.3f",m,b,d);
		pubFusedDistance.publish(fused_distance);
	}

	void calcualte_plane(float d1, float d2, bool forward, float *m, float *b){
		//forward true --> forward and middle; type false --> backward and middle
		//d1 always the middle measurement
		//d1 forward or backward depending on the closest
		float x1,x2,y1,y2;
		x1 = 0;
		y1 = -d1;
		if (forward){
			x2 = 0.2 + d2*sin((double)1.05);
			y2 = -d2*cos((double)1.05);
		} else {
			x2 = -0.2 - d2*sin((double)1.05);
			y2 = -d2*cos((double)1.05);
		}
		//it should never happen that x1==x2
		float m_temp, b_temp;
		m_temp = (y2-y1)/(x2-x1);
		b_temp = y1-m_temp*x1;
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

	// publisher
	ros::Publisher pubFusedDistance;

	float _distanceMiddle;
	float _distanceForward;
	float _distanceBackward;
	//sensor_msgs::Range	fused_distance;
	geometry_msgs::PoseStamped fused_distance;

	int rate;
	bool use_global_altitude;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DistanceFusion");
	ros::NodeHandle node;

	RangefusionClass RangefusionNode(node);

	RangefusionNode.run();
	return 0;
}
