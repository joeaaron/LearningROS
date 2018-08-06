#ifndef TRIANGLE_RECOGNIZE
#define TRIANGLE_RECOGNIZE

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/LaserScan.h>

#include <dynamic_reconfigure/server.h>
#include <lighthouse_navigation/LighthouseRecognizeConfig.h>

class TriangleRecognize
{
public:
    TriangleRecognize();
    ~TriangleRecognize();

    void Run();

private:
    ros::NodeHandle nh_;

    ros::Subscriber laser_scan_sub_;
    ros::Subscriber triangle_pose_in_map_sub_;

    ros::Publisher triangle_fit_display_pub_;
    ros::Publisher triangle_pose_pub_;

    double triangle_angle_included_;
    double triangle_length_side_;

    int triangle_start_extend_scan_index_;
    int triangle_end_extend_scan_index_;

    double laser_roll_;
    double laser_in_agv_x_;

    double max_detecte_distance_triangle_to_laser_;
    double max_detecte_angle_triangle_to_laser_;
    int    num_of_point_first_fit_used_;
    double max_distance_of_point_to_line_;
    int    point_num_of_next_several_point_;
    int    min_continuity_point_num_consider_line_;
    int    min_continuity_point_num_consider_line_far_;
    int    min_continuity_point_num_consider_line_near_;
    int    triangle_angle_extend_angle_;
    int    line_cross_tolerate_discontinuity_point_;
    double line_cross_tolerate_offset_angle_;

    bool laser_stand_upright_;
    bool get_new_dynamic_param_;
    bool get_new_triangle_pose_in_map_;
    bool status_is_run_;

    bool get_new_laser_scan_;
    bool need_trans_triangle_pose_from_map_to_laser_;
    bool new_dock_first_recognize_;

    sensor_msgs::LaserScan laser_msg_;
    geometry_msgs::PoseStamped triangle_pose_stamped_in_map_;
    geometry_msgs::PoseStamped pose_quaternion_invalid_;
    geometry_msgs::Pose2D triangle_pose2d_in_laser_upright_;

    std::string frame_id_map_;
    std::string frame_id_agv_;

    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> CartPoints;
    CartPoints triangle_scan_point_in_laser_coordinate_;

    struct LaserScanMsg
    {
        int total_point;
        double angle_resolution;
    };
    LaserScanMsg Slaser_scan_msg_;

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

    dynamic_reconfigure::Server<lighthouse_navigation::LighthouseRecognizeConfig>* dsrv_;
    void reconfigureCB(lighthouse_navigation::LighthouseRecognizeConfig config, uint32_t level);

    void GetParam();
    void TurnToIdle();
    void LaserScanCallback(const sensor_msgs::LaserScanPtr &laser_scan_msg);
    void TrianglePoseInMapCallback(const geometry_msgs::PosePtr &triangle_pose_in_map_msg);

    bool TransTrianglePoseFromMapToLaserUpright();
    double CalculateSidecBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    double CalculateAngleaBySideaSidebAnglec(double side_a, double side_b, double angle_c);
    inline void TransTriangleScanPointFromPolarToCart(double distance_triangle_to_laser);
    bool CalculateTriangleExtendScanIndexFromLastPose(double distance_triangle_to_laser);
    bool TriangleLineFitAndCalculatePose2D(double distance_triangle_to_laser);
    double DistanceBetweenTwoPoint(double x1, double y1, double x2, double y2);
    inline void LineFitLeastSquaresABC(int start, int end, double &fit_line_k, double &fit_line_b);
    bool CalculateTrianglePose2D(TriangleFit Striangle_fit);
    void PubTrianglePoseInAgv();

    void TriangleScanPointDisplayPub();
    void TriangleFitDisplayPub( TriangleFit Striangle_fit );
    double DistancePointToLine( double line_a, double line_b, double point_x, double point_y );
    double GetNextSeveralPointDistanceToLine( int start, int total_point_num, double line_k, double line_b );
    bool CheckTrianglePoseDiff(double triangle_vertex_current_x, double triangle_vertex_current_y);
    bool CheckDetectCondition(double distance_triangle_to_laser);
    void InitPoseQuaternionInvalid();
    void PubPoseQuaternionInvalid();
};

#endif
