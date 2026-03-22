#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <cmath>
#include <vector>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <map>
constexpr float PI_F = static_cast<float>(M_PI);

class ScanMerger : public rclcpp::Node
{
public:
    ScanMerger() : Node("scan_merger")
    {
        this->declare_parameter("front_topic", "/scan_front");
        this->declare_parameter("rear_topic", "/scan_rear");
        this->declare_parameter("output_topic", "/scan");
        this->declare_parameter("output_frame", "laser_frame");
        this->declare_parameter("corridor_width", 0.8);
        this->get_parameter("front_topic", front_topic_);
        this->get_parameter("rear_topic", rear_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("output_frame", output_frame_);
        this->get_parameter("corridor_width", corridor_width_);
        // ==========================
        // YAML読み込み
        // ==========================
        std::string config_path;
        this->declare_parameter("config_file", "");
        this->get_parameter("config_file", config_path);

        if (!config_path.empty()) {
            try {
                YAML::Node config = YAML::LoadFile(config_path);

                for (auto area : config["areas"]) {
                    int id = area.first.as<int>();
                    float width = area.second["corridor"]["width"].as<float>();
                    corridor_width_map_[id] = width;
                }

                RCLCPP_INFO(this->get_logger(), "YAML loaded successfully");
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "YAML load failed: %s", e.what());
            }
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ScanMerger::front_callback, this, std::placeholders::_1));

        rear_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            rear_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ScanMerger::rear_callback, this, std::placeholders::_1));

        merged_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            output_topic_, rclcpp::SensorDataQoS());

        self_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            output_topic_,                      // ← 自分の出す /scan
            rclcpp::SensorDataQoS(),
            std::bind(&ScanMerger::self_scan_callback, this, std::placeholders::_1)
        );

        wall_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wall_distance", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                wall_distance_ = msg->data;
            });

        wall_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wall_yaw_deg", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                wall_yaw_deg_ = msg->data;
            });

        corridor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/corridor_status", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {

                corridor_status_ = msg->data;

                if (corridor_width_map_.count(corridor_status_)) {
                    corridor_width_ = corridor_width_map_[corridor_status_];

                    RCLCPP_INFO(this->get_logger(),
                        "Area %d -> corridor_width = %.2f",
                        corridor_status_, corridor_width_);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Area %d not found in YAML", corridor_status_);
                }
            });

        RCLCPP_INFO(get_logger(), "scan_merger started");
    }

private:

    // --- corridor / wall info ---
    int   corridor_status_ = -1;  // -1: 遷移中, 0以上: 通路内区画ID
    float wall_distance_   = std::numeric_limits<float>::quiet_NaN(); // 左壁距離 [m]
    float wall_yaw_deg_    = std::numeric_limits<float>::quiet_NaN(); // 壁基準のAMR姿勢 [deg]
    float corridor_width_ = 0.8f; // 通路幅 [m]
    float side_distance_ = std::numeric_limits<float>::quiet_NaN();
    float diag_distance_ = std::numeric_limits<float>::quiet_NaN();
    float side_filt_ = std::numeric_limits<float>::quiet_NaN();
    float diag_filt_ = std::numeric_limits<float>::quiet_NaN();

    // --- subscribers ---
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr self_scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wall_dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wall_yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr corridor_sub_;

    std::vector<float> cell_min_;
    std::map<int, float> corridor_width_map_;
    sensor_msgs::msg::LaserScan::SharedPtr front_msg_, rear_msg_;

    inline bool isRightOccluded(float th_rad)
    {
        // 右側角度帯（例）：-150deg ～ -30deg
        return (th_rad < -70.0f * M_PI / 180.0f &&
                th_rad > -120.0f * M_PI / 180.0f);
    }


    void front_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        front_msg_ = msg;
        merge_if_ready();
    }

    void rear_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        rear_msg_ = msg;
        merge_if_ready();
    }

    void self_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!std::isfinite(wall_yaw_deg_)) return;

        // 壁に対する横方向・斜め方向（deg → rad）
        float base = -wall_yaw_deg_ * PI_F / 180.0f;

        const float target_angles[] = {
            base + PI_F / 2.0f,
            base + 3.0f * PI_F / 4.0f
        };


        float side_max = -std::numeric_limits<float>::infinity();
        float diag_max = -std::numeric_limits<float>::infinity();

        for (float target : target_angles) {

            int idx = static_cast<int>(
                (target - msg->angle_min) / msg->angle_increment
            );

            if (idx < 0 || idx >= (int)msg->ranges.size()) continue;

            float r = msg->ranges[idx];
            if (!std::isfinite(r)) continue;

            if (std::abs(target - (base + PI_F/2.0f)) < 1e-3f) {
                side_max = std::max(side_max, r);
            } else {
                diag_max = std::max(diag_max, r);
            }
        }

        side_distance_ = std::isfinite(side_max)
            ? side_max
            : std::numeric_limits<float>::quiet_NaN();

        diag_distance_ = std::isfinite(diag_max)
            ? diag_max
            : std::numeric_limits<float>::quiet_NaN();
    }


    // ----------------------------
    // LASER フィルタ（妥当距離のみ採用）
    // ----------------------------
    inline bool validRange(float r, float rmin, float rmax)
    {
        if (!std::isfinite(r)) return false;
        if (r < rmin || r > rmax) return false;
        if (r <= 0.05f) return false;
        if (r >= 29.5f) return false;
        return true;
    }

    // ----------------------------
    // FRONT + REAR を統合
    // ----------------------------
    void merge_if_ready()
    {
        if (!front_msg_ || !rear_msg_) return;

        static int cnt = 0;
        if (++cnt % 2 != 0) return;

        const int N = 2880;
        const float angle_min = -M_PI;
        const float angle_max  =  M_PI;
        const float angle_inc  = (angle_max - angle_min) / N;

        
        // ★1回作ったら再利用できるようにする
        if (cell_min_.empty()) {
            cell_min_.resize(N, std::numeric_limits<float>::infinity());
        }

        // ★毎フレーム値だけリセット（これが重要）
        std::fill(cell_min_.begin(), cell_min_.end(), std::numeric_limits<float>::infinity());

        
        rclcpp::Time merged_stamp = this->now();

        merge_scan(front_msg_, cell_min_, angle_min, angle_inc, merged_stamp);
        merge_scan(rear_msg_,  cell_min_, angle_min, angle_inc, merged_stamp);


        auto merged = std::make_shared<sensor_msgs::msg::LaserScan>();
        merged->header.stamp    = this->now();
        merged->header.frame_id = output_frame_;
        merged->angle_min       = angle_min;
        merged->angle_max       = angle_max;
        merged->angle_increment = angle_inc;
        merged->range_min       = std::min(front_msg_->range_min, rear_msg_->range_min);
        merged->range_max       = std::max(front_msg_->range_max, rear_msg_->range_max);

        merged->ranges.resize(N);

        const float W = corridor_width_; // 通路幅 [m]
        const float HALF_MARGIN = 0.23f;

        for (int i = 0; i < N; i++) {

            float th = angle_min + i * angle_inc; // LaserScan角度（ロボット前方=0）

            // 1) 実測があるなら最優先
            if (std::isfinite(cell_min_[i])) {
                merged->ranges[i] = cell_min_[i];
                continue;
            }

            // 2) 通路内（0以上）だけ補完を試みる
            if (corridor_status_ >= 0
                && std::isfinite(wall_distance_)
                && std::isfinite(wall_yaw_deg_)
                && isRightOccluded(th)
                && std::abs(wall_yaw_deg_) <= 15.0f 
                && std::abs(wall_distance_ - W * 0.5f) <= HALF_MARGIN
                && std::isfinite(side_distance_)   
                && side_distance_ <= 0.6f        
                && std::isfinite(diag_distance_)
                && diag_distance_ < (0.6f * std::sqrt(2.0f)) )
            {
                // 右壁までの「法線距離」（通路幅 - 左壁距離）
                float d_Rn = W - wall_distance_;

                // ありえない値は補完しない（安全側）
                if (d_Rn > 0.05f && d_Rn < W - 0.05f) {

                    // wall_yaw_deg は「壁基準のAMR姿勢」なので base基準の角度としてそのまま使える
                    float wall_yaw = (-1)*wall_yaw_deg_ * M_PI / 180.0f;

                    // 右壁の法線方向（壁方向から - 90deg）
                    float right_wall_normal = wall_yaw - M_PI / 2.0f;

                    // レーザ方向との差
                    float dtheta = th - right_wall_normal;
                    float cosv = std::cos(dtheta);

                    // cosが小さいと発散するので除外
                    if (std::abs(cosv) > 0.1f) {
                        float range = d_Rn / cosv;

                        // rangeの安全クリップ
                        range = std::clamp(
                            range,
                            merged->range_min + 0.05f,
                            merged->range_max * 0.9f
                        );

                        merged->ranges[i] = range;
                        continue;
                    }
                }
            }

            // 3) 補完しない場合は最大（＝unknown扱いに近い）
            merged->ranges[i] = merged->range_max;
        }


        // --- 孤立した 30m ノイズを補間（前後の平均） ---
        for (int i = 1; i < N - 1; i++) {
            float prev = merged->ranges[i - 1];
            float curr = merged->ranges[i];
            float next = merged->ranges[i + 1];

            bool prev_ok = std::isfinite(prev) && prev < merged->range_max * 0.99f;
            bool next_ok = std::isfinite(next) && next < merged->range_max * 0.99f;

            if (curr >= merged->range_max * 0.99f && prev_ok && next_ok) {
                merged->ranges[i] = 0.5f * (prev + next);
            }
        }

        merged_pub_->publish(*merged);
    }


    // ----------------------------
    // scan → 360セルへ投入
    // ----------------------------
    void merge_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        std::vector<float> &cell_min,
        float angle_min,
        float angle_inc,
        const rclcpp::Time &merged_stamp)
    {
        geometry_msgs::msg::TransformStamped tf;

        try {
            tf = tf_buffer_->lookupTransform(
                output_frame_,
                scan->header.frame_id,
                rclcpp::Time(merged_stamp)     // ← merged->header.stamp と同じ時刻
            );

        }
        catch (...) {
            RCLCPP_WARN(get_logger(), "TF failed");
            return;
        }

        float angle = scan->angle_min;

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float r = scan->ranges[i];
            if (!validRange(r, scan->range_min, scan->range_max)) {
                angle += scan->angle_increment;
                continue;
            }

            geometry_msgs::msg::PointStamped p_in, p_out;
            p_in.header.frame_id = scan->header.frame_id;
            p_in.point.x = r * cos(angle);
            p_in.point.y = r * sin(angle);
            p_in.point.z = 0.1;

            tf2::doTransform(p_in, p_out, tf);

            float R  = hypot(p_out.point.x, p_out.point.y);
            float TH = atan2(p_out.point.y, p_out.point.x);

            int idx = floor((TH - angle_min) / angle_inc);
            if (idx < 0 || idx >= (int)cell_min.size()) {
                angle += scan->angle_increment;
                continue;
            }

            // ★ 飛び値除去（既存の2倍以上は無視）
            float prev = cell_min[idx];
            if (std::isfinite(prev) && R > prev * 2.0f) {
                angle += scan->angle_increment;
                continue;
            }

            // ★ セルの代表値を min に更新
            cell_min[idx] = std::min(cell_min[idx], R);

            angle += scan->angle_increment;
        }
    }

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS IO
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_pub_;

    // params
    std::string front_topic_, rear_topic_, output_topic_, output_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMerger>());
    rclcpp::shutdown();
    return 0;
}
