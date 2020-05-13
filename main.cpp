#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <thread>
//#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/transform/transform.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/histogram.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer/common/time.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer/transform/transform.h"
#include <iostream>
#include <fstream>
#include <sstream>
//获得全局图
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/time_conversion.h"
#include "ros/time.h"
#include "absl/strings/internal/str_format/arg.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"

namespace mycartographer {

    namespace src {

        struct TrajectoryState {
            // Contains the trajectory state data received from local SLAM, after
            // it had processed accumulated 'range_data_in_local' and estimated
            // current 'local_pose' at 'time'.
            struct LocalSlamData {
                ::cartographer::common::Time time;
                ::cartographer::transform::Rigid3d local_pose;
                ::cartographer::sensor::RangeData range_data_in_local;
            };
            ::std::shared_ptr<const LocalSlamData> local_slam_data;
            ::cartographer::transform::Rigid3d local_to_map;
            ::std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
            ::cartographer_ros::TrajectoryOptions trajectory_options;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void SerializeState(const std::string& filename,cartographer::mapping::MapBuilder*map_builder_,const bool include_unfinished_submaps) {
            map_builder_->SerializeStateToFile(include_unfinished_submaps,filename);
        }

        static void OnLocalSlamResult(
                const int trajectory_id, const ::cartographer::common::Time time,
                const ::cartographer::transform::Rigid3d local_pose,
                ::cartographer::sensor::RangeData range_data_in_local,
                const ::std::unique_ptr<const ::cartographer::mapping::
                TrajectoryBuilderInterface::InsertionResult>) {
            std::cout << "OnLocalSlamResult" << std::endl;
            ::std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data =
                    ::std::make_shared<TrajectoryState::LocalSlamData>(
                            TrajectoryState::LocalSlamData{time, local_pose,
                                                           ::std::move(range_data_in_local)});
            ::absl::Mutex mutex_;
            ::absl::MutexLock lock(&mutex_);
            ::std::unordered_map<int, ::std::shared_ptr<const TrajectoryState::LocalSlamData>>
                    trajectory_state_data_ GUARDED_BY(mutex_);
            trajectory_state_data_[trajectory_id] = ::std::move(local_slam_data);
        }

        // For sensor_msgs::LaserScan.
        bool HasEcho(float) { return true; }
        float GetFirstEcho(float range) { return range; }
        template<typename LaserMessageType>
        std::tuple<::cartographer::sensor::PointCloudWithIntensities, ::cartographer::common::Time>
        LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
            CHECK_GE(msg.range_min, 0.f);
            CHECK_GE(msg.range_max, msg.range_min);
            if (msg.angle_increment > 0.f) {
                CHECK_GT(msg.angle_max, msg.angle_min);
            } else {
                CHECK_GT(msg.angle_min, msg.angle_max);
            }
            ::cartographer::sensor::PointCloudWithIntensities point_cloud;
            float angle = msg.angle_min;
            for (size_t i = 0; i < msg.ranges.size(); ++i) {
                const auto& echoes = msg.ranges[i];
                if (HasEcho(echoes)) {
                    const float first_echo = GetFirstEcho(echoes);
                    if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
                        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                        const cartographer::sensor::TimedRangefinderPoint point{
                                rotation * (first_echo * Eigen::Vector3f::UnitX()),
                                i * msg.time_increment};
                        point_cloud.points.push_back(point);
                        if (msg.intensities.size() > 0) {
                            CHECK_EQ(msg.intensities.size(), msg.ranges.size());
                            const auto& echo_intensities = msg.intensities[i];
                            CHECK(HasEcho(echo_intensities));
                            point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
                        } else {
                            point_cloud.intensities.push_back(0.f);
                        }
                    }
                }
                angle += msg.angle_increment;
            }
            ::cartographer::common::Time timestamp = cartographer_ros::FromRos(msg.header.stamp);
            if (!point_cloud.points.empty()) {
                const double duration = point_cloud.points.back().time;
                timestamp += cartographer::common::FromSeconds(duration);
                for (auto& point : point_cloud.points) {
                    point.time -= duration;
                }
            }
            return std::make_tuple(point_cloud, timestamp);
        }

        ::std::vector<float> split(const ::std::string &s, char delim) {
            ::std::vector<float> elems;
            ::std::stringstream ss(s);
            ::std::string number;
            while (::std::getline(ss, number, delim)) {
                elems.push_back(atof(number.c_str()));
            }
            return elems;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void Run(const std::string& pbstream_filename, const std::string& map_filestem,
                 const double resolution) {
            ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
            ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
            LOG(INFO) << "Loading submap slices from serialized data.";
            std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
                    submap_slices;
            ::cartographer::mapping::ValueConversionTables conversion_tables;
            ::cartographer::io::DeserializeAndFillSubmapSlices(
                    &deserializer, &submap_slices, &conversion_tables);
            CHECK(reader.eof());
            LOG(INFO) << "Generating combined map image from submap slices.";
            auto result =
                    ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
            ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");
            // There is a problem here
            ::cartographer::io::Image image(::std::move(result.surface));
            ::cartographer_ros::WritePgm(image, resolution, &pgm_writer);
            const Eigen::Vector2d origin(
                    -result.origin.x() * resolution,
                    (result.origin.y() - image.height()) * resolution);
            ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
            ::cartographer_ros::WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
        }
        
    }
};

        int main(int argc, char *argv[]) {
            google::InitGoogleLogging(argv[0]);
            google::ParseCommandLineFlags(&argc, &argv, true);
            fflush(stdout);
            cartographer_ros::NodeOptions node_options;
            cartographer_ros::TrajectoryOptions trajectory_options;
            std::string FLAGS_configuration_directory = "/home/ethan/catkin_ws/src/turtlebot3/turtlebot3_slam/config";
            std::string FLAGS_configuration_basename = "turtlebot3_lds_2d_gazebo.lua";
            ::std::tie(node_options, trajectory_options) =
                    cartographer_ros::LoadOptions(FLAGS_configuration_directory,
                                                  FLAGS_configuration_basename);
            auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
                    node_options.map_builder_options);
            using SensorId = ::cartographer::mapping::TrajectoryBuilderInterface::SensorId;
            SensorId sensorid;
            sensorid.id = "range_sensor";
            sensorid.type = ::cartographer::mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE;
            ::std::set<SensorId> expected_sensor_ids;
            expected_sensor_ids.insert(sensorid);
            using LocalSlamResultCallback =
            ::cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback;
            const int trajectory_id = map_builder->AddTrajectoryBuilder(
                    expected_sensor_ids, trajectory_options.trajectory_builder_options,
                    ::std::bind(&::mycartographer::src::OnLocalSlamResult,
                                ::std::placeholders::_1, ::std::placeholders::_2,
                                ::std::placeholders::_3, ::std::placeholders::_4,
                                ::std::placeholders::_5));
            auto trajectory_builder_ = map_builder->GetTrajectoryBuilder(trajectory_id);
            std::string sensor_id = "range_sensor";

            /***************************************************************************/
            std::ifstream myfile("/home/ethan/MyBag/20200420/myScan.txt");
            std::ofstream outfile("/home/ethan/MyBag/20200420/fftestout2.txt", std::ios::app);
            std::string temp;
            if (!myfile.is_open()) {
                std::cout << "未成功打开txt文件" << std::endl;
            }
            std::vector<::sensor_msgs::LaserScan> laser_scans;
            int size = 0;
            ::sensor_msgs::LaserScan scan;
            while (getline(myfile, temp)) {
                unsigned long i = temp.find("\n");
                int temp_size = temp.size();
                unsigned long iend = temp.find("---");
                if (iend == 0) {
                    laser_scans.push_back(scan);
                }
                unsigned long iheader = temp.find("header: ");
                if (iheader == 0) {
                }
                unsigned long iseq = temp.find("  seq: ");
                if (iseq == 0) {
                    std::string now_string = "  seq: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    int c_int = atoi(c_string.c_str());
                    scan.header.seq = (uint32_t) c_int;
                }
                unsigned long istamp = temp.find("  stamp: ");
                if (istamp == 0) {
                }
                unsigned long isecs = temp.find("    secs: ");
                if (isecs == 0) {
                    std::string now_string = "    secs: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    int c_int = atoi(c_string.c_str());
                    scan.header.stamp.sec = (uint32_t) c_int;
                }
                //
                unsigned long insecs = temp.find("    nsecs: ");
                if (insecs == 0) {
                    std::string now_string = "    nsecs: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    int c_int = atoi(c_string.c_str());
                    scan.header.stamp.nsec = (uint32_t) c_int;
                }
                unsigned long iframe_id = temp.find("  frame_id: ");
                if (iframe_id == 0) {
                    std::string now_string = "  frame_id: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    scan.header.frame_id = c_string;
                }
                unsigned long iangle_min = temp.find("angle_min: ");
                if (iangle_min == 0) {
                    std::string now_string = "angle_min: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.angle_min = c_float;
                }
                unsigned long iangle_max = temp.find("angle_max: ");
                if (iangle_max == 0) {
                    std::string now_string = "angle_max: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.angle_max = c_float;
                }
                unsigned long iangle_increment = temp.find("angle_increment: ");
                if (iangle_increment == 0) {
                    std::string now_string = "angle_increment: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.angle_increment = c_float;
                }
                unsigned long itime_increment = temp.find("time_increment: ");
                if (itime_increment == 0) {
                    std::string now_string = "time_increment: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.time_increment = c_float;
                }
                unsigned long iscan_time = temp.find("scan_time: ");
                if (iscan_time == 0) {
                    std::string now_string = "scan_time: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.scan_time = c_float;
                }
                unsigned long irange_min = temp.find("range_min: ");
                if (irange_min == 0) {
                    std::string now_string = "range_min: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.range_min = c_float;
                }
                unsigned long irange_max = temp.find("range_max: ");
                if (irange_max == 0) {
                    std::string now_string = "range_max: ";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    float c_float = atof(c_string.c_str());
                    scan.range_max = c_float;
                }
                unsigned long iranges = temp.find("ranges: [");
                if (iranges == 0) {
                    std::string now_string = "ranges: [";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    const std::vector<float> &vector = mycartographer::src::split(c_string, ',');
                    scan.ranges = vector;
                }
                unsigned long iintensities = temp.find("intensities: [");
                if (iintensities == 0) {
                    std::string now_string = "intensities: [";
                    int now_int = now_string.size();
                    int now_size = temp.size();
                    std::string c_string = temp.substr(now_int, now_size);
                    const std::vector<float> &vector = mycartographer::src::split(c_string, ',');
                    scan.intensities = vector;
                }
            }
            //std::cout << laser_scans.size() << std::endl;
            for (int i = 0; i < laser_scans.size(); ++i) {
                ::sensor_msgs::LaserScan scan_t = laser_scans[i];
            }
            myfile.close();
            outfile.close();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            for (int j = 0; j < laser_scans.size(); ++j) {
                ::sensor_msgs::LaserScan scan_msg = laser_scans[j];
                const std::tuple<cartographer::sensor::PointCloudWithIntensities, cartographer::common::Time> &tuple = mycartographer::src::LaserScanToPointCloudWithIntensities(
                        scan_msg);
                cartographer::sensor::PointCloudWithIntensities points;
                cartographer::common::Time commontime;
                std::tie(points, commontime) = tuple;
                using rep = int64_t ;
                using period = std::ratio<1, 10000000>;
                const std::chrono::duration<rep, period> &durationtime = commontime.time_since_epoch();
                std::map<std::string, cartographer::common::Time> sensor_to_previous_subdivision_time_;
                int num_subdivisions_per_laser_scan_ = 1;
                CHECK_LE(points.points.back().time, 0);
                std::cout<<"points size"<<std::endl;
                std::cout<<points.points.size()<<std::endl;
                for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
                    const size_t start_index =
                            points.points.size() * i / num_subdivisions_per_laser_scan_;
                    const size_t end_index =
                            points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
                    cartographer::sensor::TimedPointCloud subdivision(
                            points.points.begin() + start_index, points.points.begin() + end_index);
                    if (start_index == end_index) {
                        continue;
                    }
                    const double time_to_subdivision_end = subdivision.back().time;
                    // `subdivision_time` is the end of the measurement so sensor::Collator will
                    // send all other sensor data first.
                    const cartographer::common::Time subdivision_time =
                            commontime + cartographer::common::FromSeconds(time_to_subdivision_end);
                    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
                    if (it != sensor_to_previous_subdivision_time_.end() &&
                        it->second >= subdivision_time) {
                        std::cout << "ignored message" << std::endl;
                        LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                                     << sensor_id << " because previous subdivision time "
                                     << it->second << " is not before current subdivision time "
                                     << subdivision_time;
                        continue;
                    }
                    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
                    for (auto& point : subdivision) {
                        point.time -= time_to_subdivision_end;
                    }
                    CHECK_EQ(subdivision.back().time, 0.f);
                    //
                    using Vector = Eigen::Matrix<double, 3, 1>;
                    double a = 0;
                    Vector Vtranslation(a, a, a);
                    Eigen::Quaterniond rotation(a, a, a, a);
                    ::std::unique_ptr<::cartographer::transform::Rigid3d> sensor_to_tracking(
                            new ::cartographer::transform::Rigid3d(Vtranslation, rotation));
                    if (sensor_to_tracking == nullptr) {
                        std::cout << "sensor_to_tracking == nullptr" << std::endl;
                    }
                    if (sensor_to_tracking != nullptr) {
                        std::cout<<j<<std::endl;
                        std::cout<<"position"<<std::endl;
                        std::cout<< subdivision.back().position <<std::endl;
                        std::cout<<"time"<<std::endl;
                        std::cout<<subdivision_time<<std::endl;
                        trajectory_builder_->AddSensorData(
                                sensor_id, cartographer::sensor::TimedPointCloudData{
                                        subdivision_time, sensor_to_tracking->translation().cast<float>(),
                                        cartographer::sensor::TransformTimedPointCloud(subdivision,
                                                                                       sensor_to_tracking->cast<float>())
                                });
                        std::cout<<"sensor data added"<<std::endl;
                    }
                }
            }

            mycartographer::src::SerializeState("/home/ethan/MyBag/MyPbstream/pbstream_test.pbstream",map_builder.get(),true);
            mycartographer::src::Run("/home/ethan/MyBag/MyPbstream/pbstream_test.pbstream",
                                   "/home/ethan/MyBag/MyPbstream/pbstream_test",
                                   0.1);

        }