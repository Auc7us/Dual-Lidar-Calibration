#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class DownsamplingNode : public rclcpp::Node
{
public:
  DownsamplingNode() : Node("downsampling_node")
  {
    subscription_l_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/x90_l/points", 10, std::bind(&DownsamplingNode::topic_callback_l, this, std::placeholders::_1));
    subscription_r_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/x90_r/points", 10, std::bind(&DownsamplingNode::topic_callback_r, this, std::placeholders::_1));

    publisher_l_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/x90_l/points_downsampled", 10);
    publisher_r_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/x90_r/points_downsampled", 10);
  }

private:
  void topic_callback_l(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    process_point_cloud(msg, publisher_l_);
  }

  void topic_callback_r(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    process_point_cloud(msg, publisher_r_);
  }

  void process_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
  {
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Convert ROS message to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform voxel grid downsampling
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.07f, 0.07f, 0.07f);  // Example voxel size
    sor.filter(*cloud_filtered);

    // Convert filtered PCL data back to ROS message
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);

    publisher->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_l_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_r_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_l_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_r_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DownsamplingNode>());
  rclcpp::shutdown();
  return 0;
}





















// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>

// class DownsamplingNode : public rclcpp::Node
// {
// public:
//   DownsamplingNode() : Node("downsampling_node")
//   {
//     subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sensing/x90_l/points", 10, std::bind(&DownsamplingNode::topic_callback, this, std::placeholders::_1));
//     publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/x90_l/points_downsampled", 10);
//   }

// private:
//   void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
//   {
//     pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
//     pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

//     // Convert ROS message to PCL data type
//     pcl_conversions::toPCL(*msg, *cloud);

//     // Perform voxel grid downsampling
//     pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//     sor.setInputCloud(cloud);
//     sor.setLeafSize(0.07f, 0.07f, 0.07);  // Example voxel size
//     sor.filter(*cloud_filtered);

//     // Convert filtered PCL data back to ROS message
//     sensor_msgs::msg::PointCloud2 output;
//     pcl_conversions::fromPCL(*cloud_filtered, output);

//     publisher_->publish(output);
//   }

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<DownsamplingNode>());
//   rclcpp::shutdown();
//   return 0;
// }
