#ifndef PUBLIC_TYPE_H
#define PUBLIC_TYPE_H
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace costmap{
// Header
struct StdStamp {
    int secs;
    int nsecs;
};

struct StdHeader {
    unsigned int    seq;
    StdStamp        stamp;
    std::string     frame_id;
};

/**
 * Pose struct with 2D axis x and y, and th
**/
struct StdPose2D {
    float x;
    float y;
    float th;
};

struct StdPose2DWithHeader {
    StdHeader header;
    StdPose2D pose;
};

enum class LocateMode {
    Normal       = 1,
    Drift        = 2,
    Sorround     = 3,
    Odom         = 4,
    Rest         = 5,
    ParamError   = 6,
    NoInit       = 7
};

struct LocateState {
    LocateMode          mode;                   // mode of localization
    float               confidence;             // confidence of localization
    StdPose2DWithHeader pose_timestamped;       // pose to output
};

struct NumWithAngular {
    NumWithAngular(): num(0.0f), angular(0.0f)
    {}
    NumWithAngular& operator = (const NumWithAngular& num_angle)
    {
        if (this == &num_angle)
            return *this;

        this->num = num_angle.num;
        this->angular = num_angle.angular;

        return *this;
    }
    float num;
    float angular;
};

struct LaserScan {
    LaserScan() {}
    LaserScan& operator = (const LaserScan& laser)
    {
        if (this == &laser)
            return *this;

        this->header = laser.header;
        this->angle_min = laser.angle_min;
        this->angle_max = laser.angle_max;
        this->range_min = laser.range_min;
        this->range_max = laser.range_max;
        this->ranges.assign(laser.ranges.begin(), laser.ranges.end());
        this->intensities.assign(laser.intensities.begin(), laser.intensities.end());

        return *this;
    }
    StdHeader header;
    float angle_min;
    float angle_max;
    float range_min;
    float range_max;
    std::vector<NumWithAngular> ranges;
    std::vector<NumWithAngular> intensities;
};


struct RGBD_INFO {
public:
    struct Intrinsic {
    public:
        float scale;
        float fx;
        float fy;
        float cx;
        float cy;

        void clear()
        {
            scale = 0.0f;
            fx = 0.0f;
            fy = 0.0f;
            cx = 0.0f;
            cy = 0.0f;
        }
    };

    struct RobotLocateStateInW {
    public:
        float pose_x;
        float pose_y;
        float angle; //机器人的旋转角度，正方向为零位角，逆时针为正，单位：弧度

        void clear()
        {
            pose_x = 0.0f;
            pose_y = 0.0f;
            angle = 0.0f;
        }
    };

    void operator=(RGBD_INFO&& rgbd_INFO) noexcept
    {
        if(this != &rgbd_INFO) {
            this->image_size = rgbd_INFO.image_size;
            this->intrinsic = rgbd_INFO.intrinsic;
            this->robot_locate_state_in_W = rgbd_INFO.robot_locate_state_in_W;
            this->depth_image = rgbd_INFO.depth_image;
            rgbd_INFO.depth_image.release();
        }
    }

    void clear()
    {
        intrinsic.clear();
        robot_locate_state_in_W.clear();
        image_size = cv::Size(0, 0);
        depth_image.release();
    }

    Intrinsic intrinsic;
    RobotLocateStateInW robot_locate_state_in_W;
    cv::Size image_size;
    cv::Mat depth_image;
};
}
bool loadLidarData(const std::string& filename, costmap::LaserScan& ldata, costmap::LocateState& vpose);

#endif