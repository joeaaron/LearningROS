#ifndef REFLECTOR_RECOGNIZE
#define REFLECTOR_RECOGNIZE

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/LaserScan.h>

#include <dynamic_reconfigure/server.h>
#include <lighthouse_navigation/LighthouseRecognizeConfig.h>

class ReflectorRecognize
{
public:
    ReflectorRecognize();
    ~ReflectorRecognize();

    void Run();

private:
    ros::NodeHandle nh_;

    ros::Subscriber laser_scan_sub_;
    ros::Subscriber reflector_pose_in_map_sub_;
    ros::Publisher belt_fit_display_pub_;
    ros::Publisher reflector_pose_pub_;

    std::string frame_id_map_;
    std::string frame_id_robot_;

    bool laser_stand_upright_;
    bool get_new_dynamic_param_;
    bool get_new_laser_scan_;
    bool get_new_reflector_pose_in_map_;
    bool need_trans_reflector_pose_from_map_to_laser_;
    bool new_dock_first_recognize_;
    bool status_is_run_;

    sensor_msgs::LaserScan laser_msg_;

    double belt_half_length_;
    int min_point_num_consider_belt_;
    int min_intensite_consider_reflector_point_;
    double max_detecte_distance_belt_to_laser_;
    double max_detecte_angle_belt_to_laser_;
    int    belt_scan_index_extend_angle_;
    double max_tolerate_diff_distance_belt_to_laser_;
    double max_tolerate_diff_belt_length_detected_;

    double laser_roll_;
    double laser_in_agv_x_;

    int reflector_start_extend_scan_index_;
    int reflector_end_extend_scan_index_;

    geometry_msgs::PoseStamped pose_quaternion_invalid_;
    geometry_msgs::PoseStamped reflector_pose_stamped_in_map_;
    geometry_msgs::Pose2D reflector_pose2d_in_laser_upright_;

    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> CartPoints;
    CartPoints laser_scan_point_to_coordinate_;

    struct LaserScanMsg
    {
        int total_point;
        double angle_resolution;
    };
    LaserScanMsg Slaser_scan_msg_;

    dynamic_reconfigure::Server<lighthouse_navigation::LighthouseRecognizeConfig>* dsrv_;
    void reconfigureCB(lighthouse_navigation::LighthouseRecognizeConfig config, uint32_t level);

    void GetParam();
    void TurnToIdle();
    void InitPoseQuaternionInvalid();
    void PubReflectorPoseInAgv();
    void LaserScanCallback(const sensor_msgs::LaserScanPtr& laser_scan_msg);
    void ReflectorPoseInMapCallback(const geometry_msgs::PosePtr &reflector_pose_in_map_msg);
    bool TransReflectorPoseFromMapToLaserUpright();
    void BeltFitDisplayPub(double belt_fit_line_K, double belt_fit_line_B );
    void PubPoseQuaternionInvalid();

    double CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    double CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    void TransReflectorScanPointFromPolarToCart(double distance_reflector_to_laser);
    bool LineFitLeastSquaresABC(const int belt_index_start, const int belt_index_end, double &fit_line_k, double &fit_line_b);
    bool ReflectorLineFitAndCalculatePose2D(double distance_reflector_to_laser);

    bool CalculateReflectorExtendScanIndexFromLastPose(double distance_reflector_to_laser);
    double GetBeltXOrYInLaserCoordinate(int X_or_Y, int belt_index_start, int belt_index_end);
    double GetBeltVerticalLineAngleWithLaserAxisX( double belt_fit_line_k, double belt_fit_line_b );
    void CheckReflectorDetectCondition(double distance_triangle_to_laser);
    bool CheckDetectedBeltLength( double index_start, double index_end );
    bool CheckDetectedLighthousePoseDiff(double triangle_vertex_current_x, double triangle_vertex_current_y);
    double DistanceBetweenTwoPoint(double x1, double y1, double x2, double y2);
    double GetValueXByEquation( double belt_line_k, double belt_line_b );
};

#endif
