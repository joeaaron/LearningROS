#ifndef LIGHTHOUSE_RECOGNIZE
#define LIGHTHOUSE_RECOGNIZE

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/LaserScan.h>

#include <dynamic_reconfigure/server.h>
#include <lighthouse_navigation/LighthouseRecognizeConfig.h>

class LighthouseRecognize
{
public:
    LighthouseRecognize();
    ~LighthouseRecognize();

    void Run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber lighthouse_pose_in_map_sub_;
    ros::Publisher lighthouse_fit_display_pub_;
    ros::Publisher lighthouse_pose_pub_;

    //共同参数
    int    lighthouse_extend_angle_;

    //三角板参数
    double triangle_angle_included_;
    double max_detecte_distance_triangle_to_laser_;
    double max_detecte_angle_triangle_to_laser_;
    int    num_of_point_first_fit_used_;
    double max_distance_of_point_to_line_;
    int    point_num_of_next_several_point_;
    int    min_continuity_point_num_consider_line_;
    int    min_continuity_point_num_consider_line_far_;
    int    min_continuity_point_num_consider_line_near_;
    int    line_cross_tolerate_discontinuity_point_;
    double line_cross_tolerate_offset_angle_;

    //反光胶贴参数
    int min_point_num_consider_belt_;
    int min_intensite_consider_reflector_point_;
    double max_detecte_distance_belt_to_laser_;
    double max_detecte_angle_belt_to_laser_;
    double max_tolerate_diff_belt_length_detected_;

    double laser_roll_;
    double laser_in_agv_x_;

    int lighthouse_start_extend_scan_index_;
    int lighthouse_end_extend_scan_index_;

    sensor_msgs::LaserScan laser_msg_;
    geometry_msgs::PoseStamped pose_quaternion_invalid_;
    geometry_msgs::Pose2D lighthouse_pose2d_in_laser_upright_;

    std::string frame_id_map_;
    std::string frame_id_agv_;

    struct TriangleFit
    {
        int index_first_start;
        int index_first_end;
        int index_seconde_start;
        int index_seconde_end;

        double line_first_k;
        double line_first_b;
        double line_seconde_k;
        double line_seconde_b;
    };

    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> CartPoints;
    CartPoints lighthouse_in_laser_upright_coordinate_;

    double lighthouse_length_;
    std::string lighthouse_type_;

    bool laser_stand_upright_;
    bool get_new_lighthouse_param_;
    bool status_is_run_;
    bool get_new_laser_scan_;
    bool get_new_lighthouse_pose_in_map_;

    bool need_trans_lighthouse_pose_from_map_to_laser_;
    bool new_dock_first_recognize_;

    geometry_msgs::PoseStamped lighthouse_pose_stamped_in_map_;

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
    void LaserScanCallback(const sensor_msgs::LaserScanPtr &laser_scan_msg);
    void LighthousePoseInMapCallback(const geometry_msgs::PosePtr &lighthouse_pose_in_map_msg);
    bool TransLighthousePoseFromMapToLaserUpright();
    void LighthouseScanPointDisplayPub();
    void LighthouseFitDisplayPub( TriangleFit Striangle_fit );
    double CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    double CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    bool CalculateLighthouseExtendScanIndexFromLastPose(double distance_lighthouse_to_laser, double angle_lighthouse_with_laser_axisX);
    void PubPoseQuaternionInvalid();
    inline void TransLighthouseScanPointFromPolarToCart(double distance_lighthouse_to_laser);
    void PubLighthousePoseInAgv();
    bool LineFitLeastSquaresABC(int index_start, int index_end, double &fit_line_k, double &fit_line_b);
    double DistancePointToLine( double line_k, double line_b, double point_x, double point_y );
    double DistanceBetweenTwoPoint(double x1, double y1, double x2, double y2);
    void CheckLighthouseDetectCondition(double distance_triangle_to_laser, double max_distance, double max_angle);
    bool CheckDetectedLighthousePoseDiff(double current_x, double current_y);

    bool TriangleLineFitAndCalculatePose2D(double distance_triangle_to_laser);
    double TriangleGetNextSeveralPointDistanceToLine( int start, int total_point_num, double line_k, double line_b );
    bool TriangleCalculatePose2D(TriangleFit Striangle_fit);

    double ReflectorGetBeltVerticalLineAngleWithLaserAxisX(double belt_fit_line_k, double belt_fit_line_b);
    double ReflectorGetBeltXOrYInLaserCoordinate(int X_or_Y, int belt_index_start, int belt_index_end);
    double ReflectorGetValueXByEquation( double belt_line_k, double belt_line_b );
    bool ReflectorCheckDetectedBeltLength(double index_start, double index_end);
    bool ReflectorLineFitAndCalculatePose2D(double distance_reflector_to_laser);
};

#endif
