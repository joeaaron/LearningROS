#include <magnetic_tf_pub.h>
#include <tf/transform_broadcaster.h>

using namespace std;
#define SIGN(n) (n <= 0?( n < 0 ?-1:0):1)

MagneticTfPub::MagneticTfPub()
    : nh_()
    , magnetic_head_sub_()
    , magnetic_tail_sub_()
    , magnetic_head_pose2d_()
    , magnetic_tail_pose2d_()
    , magnetic_head_is_forward_(true)
    , magnetic_tail_is_forward_(true)
    , get_new_head_magnetic_data_(false)
    , get_new_tail_magnetic_data_(false)
    , start_magnetic_navigation_(true)
    , start_magnetic_navigation_first_time_(true)
    , magnetic_head_on_track_(false)
    , distance_magnetic_goal_to_magnetic_center_()
{
    magnetic_head_sub_ = nh_.subscribe("/magnetic_head_data", 1, &MagneticTfPub::MagneticHeadCallback, this);
    magnetic_tail_sub_ = nh_.subscribe("/magnetic_tail_data", 1, &MagneticTfPub::MagneticTailCallback, this);

    ros::NodeHandle nh_p("~");

    nh_p.param("magnetic_head_is_forward", magnetic_head_is_forward_, true);
    nh_p.param("magnetic_tail_is_forward", magnetic_tail_is_forward_, true);
    nh_p.param("distance_magnetic_goal_to_magnetic_center", distance_magnetic_goal_to_magnetic_center_, 0.5);

    double distance_magnetic_head_to_robot_center;
    double distance_magnetic_tail_to_robot_center;
    nh_p.param("distance_magnetic_head_to_robot_center", distance_magnetic_head_to_robot_center, 0.27);
    nh_p.param("distance_magnetic_tail_to_robot_center", distance_magnetic_tail_to_robot_center, 0.27);

    magnetic_head_pose2d_.x = distance_magnetic_head_to_robot_center;
    magnetic_tail_pose2d_.x = -distance_magnetic_tail_to_robot_center;

    nh_p.param("fake_pose_y_out_track_cm", fake_pose_y_out_track_cm_, 5.0);
    cout << "fake_pose_y_out_track_cm_: " << fake_pose_y_out_track_cm_ << endl;
}

MagneticTfPub::~MagneticTfPub()
{}

void MagneticTfPub::MagneticHeadCallback(const std_msgs::Int32MultiArrayPtr magnetic_head_msg)
{
    magnetic_head_pose2d_.y = GetPoseYInRobotBase(magnetic_head_is_forward_, magnetic_head_msg);
    get_new_head_magnetic_data_ = true;
    // cout << "magnetic_head_pose2d_.y: " << magnetic_head_pose2d_.y << endl;
}

void MagneticTfPub::MagneticTailCallback(const std_msgs::Int32MultiArrayPtr magnetic_tail_msg)
{
    magnetic_tail_pose2d_.y  = GetPoseYInRobotBase(magnetic_tail_is_forward_, magnetic_tail_msg);
    get_new_tail_magnetic_data_ = true;
}

double MagneticTfPub::GetPoseYInRobotBase(bool magnetic_is_forward, const std_msgs::Int32MultiArrayPtr magnetic_msg)
{
    int num_detected_magnetic = 0;
    double pose_y_sum = 0.0;

    int signal = 0;
    if ( magnetic_is_forward )
    {
        signal = -1;
    }
    else
    {
        signal = 1;
    }

    int magnetic_msg_size = magnetic_msg->data.size();
    float magnetic_data_middle_index = (magnetic_msg_size - 1) / 2.0;

    for ( int i = 0; i < magnetic_msg_size; ++i )
    {
        if (magnetic_msg->data[i] != 0)
        {
            ++num_detected_magnetic;
            pose_y_sum += signal * (magnetic_data_middle_index - i);
        }
    }

    double pose_y_cm = 0.0;
    static int get_signal = 0;

    if ( num_detected_magnetic == 0 )
    {
        magnetic_head_on_track_ = false;
        pose_y_cm = get_signal * fake_pose_y_out_track_cm_;
    }
    else
    {
        magnetic_head_on_track_ = true;
        pose_y_cm = (pose_y_sum / num_detected_magnetic);

        get_signal = SIGN(pose_y_cm);
        start_magnetic_navigation_first_time_ = false;
    }

    return pose_y_cm;
}

double MagneticTfPub::FakeYForControl(double magnetic_y_cm)
{
    double fake_y_for_control_m = magnetic_y_cm * fabs(magnetic_y_cm) * 0.01;// * 2;
    return fake_y_for_control_m;
}

void MagneticTfPub::Run()
{
    ros::Rate r(20);
    while ( ros::ok() )
    {
        //1.是否开始磁条导航
        if ( !start_magnetic_navigation_ )
        {
            r.sleep();
            continue;
        }

        ros::spinOnce();

        //2.磁条传感器数据是否有更新
        if ( !get_new_head_magnetic_data_ && !get_new_tail_magnetic_data_ )
        {
            ROS_WARN("both head and tail magnetic data no updata");
            r.sleep();
            continue;
        }
        get_new_head_magnetic_data_ = false;
        get_new_tail_magnetic_data_ = false;

        double magnetic_tf_theat = 0.0;
        double magnetic_tf_x = 0.0;
        double magnetic_tf_Y = 0.0;

        //3.开始磁条导航时，是否检测到磁条
        if ( start_magnetic_navigation_first_time_ && !magnetic_head_on_track_ )
        {
            //目前不发布tf，后期可以根据需要发布tf让agv左右旋转寻找磁条
            ROS_ERROR("start magnetic navigation while out track");
            r.sleep();
            continue;
        }
        else
        {
            double head_fake_dy = FakeYForControl(magnetic_head_pose2d_.y);
            double tail_fake_dy = FakeYForControl(magnetic_tail_pose2d_.y);

            //计算磁条与robot的yaw
            double dx = magnetic_head_pose2d_.x - magnetic_tail_pose2d_.x;
            double dy = head_fake_dy - tail_fake_dy;
            cout << "dx: " << dx << "   dy: " << dy << endl;
            magnetic_tf_theat = atan2(dy, dx);

            //计算base_footprint与磁条的垂足
            //由两点求直线方程的一般式AX+BY+C=0, A=Y2-Y1, B=X1-X2, C=X2*Y1-X1*Y2;
            //求已知点到已知直线的垂足( (B*B*X-A*B*Y-A*C)/(A*A+B*B), (A*A*Y-A*B*X-B*C)/(A*A+B*B) )
            double A = dy;
            double B = -dx;
            double C = magnetic_head_pose2d_.x * tail_fake_dy - magnetic_tail_pose2d_.x * head_fake_dy;
            double Apow2_add_Bpow2 = A*A + B*B;

            magnetic_tf_x = -A*C/Apow2_add_Bpow2;
            magnetic_tf_Y = -B*C/Apow2_add_Bpow2;
        }

//        cout << "magnetic_tf_x: " << magnetic_tf_x << endl;
//        cout << "magnetic_tf_Y: " << magnetic_tf_Y << endl;
//        cout << "magnetic_tf_theat: " << magnetic_tf_theat << endl;

        //发布magnetic_center的tf
        tf::Quaternion q;
        q.setRPY(0, 0, magnetic_tf_theat);

        static tf::TransformBroadcaster br;

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(magnetic_tf_x, magnetic_tf_Y, 0.0) );
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "magnetic_center"));

        if ( !magnetic_head_on_track_ )
        {
            ROS_ERROR("out stop!!!");
            q.setRPY(0, 0, 0);

            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "magnetic_goal"));

            r.sleep();
            continue;
        }

        //发布magnetic_goal的tf(磁条magnetic_center前方distance_magnetic_goal_to_magnetic_center_米处)
        q.setRPY(0, 0, 0);

        transform.setOrigin( tf::Vector3(distance_magnetic_goal_to_magnetic_center_, 0.0, 0.0) );
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "magnetic_center", "magnetic_goal"));

        r.sleep();
    }
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "magnetic_tf_pub");
    MagneticTfPub magnetic_tf_pub;
    magnetic_tf_pub.Run();
    return 0;
};
