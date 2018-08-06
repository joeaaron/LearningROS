#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>

using namespace std;

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

class RecognizePrecision
{
public:
    RecognizePrecision();
    ~RecognizePrecision();

private:
    ros::NodeHandle nh_;
    ros::Subscriber lighthouse_pose_sub_;

    int recoder_num_;

    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> CartPoints;
    CartPoints point_in_robot_coordinate_;

    void LighthousePoseCallback(const geometry_msgs::PoseStampedPtr& lighthouse_pose_stamped_in_agv_msg);
    void GetStandardDeviation();
};

RecognizePrecision::RecognizePrecision()
    :nh_(),
    lighthouse_pose_sub_(),
    recoder_num_(600),
    point_in_robot_coordinate_()
{
    lighthouse_pose_sub_ = nh_.subscribe("/lighthouse_pose", 1, &RecognizePrecision::LighthousePoseCallback, this );
    point_in_robot_coordinate_.resize(3, recoder_num_);
}

RecognizePrecision::~RecognizePrecision()
{
}

void RecognizePrecision::LighthousePoseCallback(const geometry_msgs::PoseStampedPtr& lighthouse_pose_stamped_in_agv_msg)
{
    static int count_lighthouse_pose = 0;

    if ( count_lighthouse_pose < recoder_num_ )
    {
        try
        {
            tf::assertQuaternionValid(lighthouse_pose_stamped_in_agv_msg->pose.orientation );
        }
        catch (tf2::InvalidArgumentException& e)
        {
            ROS_ERROR( "assertQuaternionValid: %s", e.what() );
            return;
        }

        point_in_robot_coordinate_(0,count_lighthouse_pose) = lighthouse_pose_stamped_in_agv_msg->pose.position.x;
        point_in_robot_coordinate_(1,count_lighthouse_pose) = lighthouse_pose_stamped_in_agv_msg->pose.position.y;
        point_in_robot_coordinate_(2,count_lighthouse_pose) = tf::getYaw( lighthouse_pose_stamped_in_agv_msg->pose.orientation ) ;

        ++count_lighthouse_pose;
        if ( count_lighthouse_pose % 10 == 0 )
        {
            cout << "recoder_times: " << count_lighthouse_pose << endl;
        }
        return;
    }
    else
    {
        GetStandardDeviation();
        return;
    }
}

void RecognizePrecision::GetStandardDeviation()
{
    geometry_msgs::Pose2D min_pose2d;
    geometry_msgs::Pose2D max_pose2d;

    //初始化
    min_pose2d.x = max_pose2d.x = point_in_robot_coordinate_(0,0);
    min_pose2d.y = max_pose2d.y = point_in_robot_coordinate_(1,0);
    min_pose2d.theta = max_pose2d.theta = point_in_robot_coordinate_(2,0);

    geometry_msgs::Pose2D sum_pose2d;
    int num = point_in_robot_coordinate_.cols();

    //求范围，求和
    for ( int i = 0; i < num; ++i )
    {
        double x_value = point_in_robot_coordinate_(0,i);
        double y_value = point_in_robot_coordinate_(1,i);
        double theta_value = point_in_robot_coordinate_(2,i);

        if (x_value < min_pose2d.x)
        {
            min_pose2d.x = x_value;
        }
        else if ( x_value > max_pose2d.x )
        {
            max_pose2d.x = x_value;
        }

        if (y_value < min_pose2d.y)
        {
            min_pose2d.y = y_value;
        }
        else if ( y_value > max_pose2d.y )
        {
            max_pose2d.y = y_value;
        }
        if (theta_value < min_pose2d.theta)
        {
            min_pose2d.theta = theta_value;
        }
        else if ( theta_value > max_pose2d.theta )
        {
            max_pose2d.theta = theta_value;
        }

        sum_pose2d.x += x_value;
        sum_pose2d.y += y_value;
        sum_pose2d.theta += theta_value;
    }

    cout << "range x: " << max_pose2d.x << " - " << min_pose2d.x << "  =  " << max_pose2d.x - min_pose2d.x << endl;
    cout << "range y: " << max_pose2d.y << " - " << min_pose2d.y << "  =  " << max_pose2d.y - min_pose2d.y << endl;
    cout << "range theta: " << RadianToAngle(max_pose2d.theta) << " - " << RadianToAngle(min_pose2d.theta) << "  =  " << RadianToAngle(max_pose2d.theta - min_pose2d.theta) << endl;

    //求平均值
    geometry_msgs::Pose2D average_pose2d;
    average_pose2d.x = sum_pose2d.x / num;
    average_pose2d.y = sum_pose2d.y / num;
    average_pose2d.theta = sum_pose2d.theta / num;

    //求方差
    geometry_msgs::Pose2D variance_pose2d;
    for ( int i = 0; i < num; ++i )
    {
        variance_pose2d.x += pow( point_in_robot_coordinate_(0,i) - average_pose2d.x, 2);
        variance_pose2d.y += pow( point_in_robot_coordinate_(1,i) - average_pose2d.y, 2);
        variance_pose2d.theta += pow( point_in_robot_coordinate_(2,i) - average_pose2d.theta, 2);
    }
    variance_pose2d.x = variance_pose2d.x / num;
    variance_pose2d.y = variance_pose2d.y / num;
    variance_pose2d.theta = variance_pose2d.theta / num;

    //求标准差
    geometry_msgs::Pose2D standard_deviation_pose2d;
    standard_deviation_pose2d.x = sqrt(variance_pose2d.x);
    standard_deviation_pose2d.y = sqrt(variance_pose2d.y);
    standard_deviation_pose2d.theta = sqrt(variance_pose2d.theta);

    cout << "standard_deviation_pose2d.x: " << standard_deviation_pose2d.x << endl;
    cout << "standard_deviation_pose2d.y: " << standard_deviation_pose2d.y << endl;
    cout << "standard_deviation_pose2d.theta: " << RadianToAngle(standard_deviation_pose2d.theta) << endl;

    exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recognize_precision");
  RecognizePrecision recognize_precision;
  ros::spin();
  return 0;
};

