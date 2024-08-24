#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
// #include <mlpack/methods/dbscan/dbscan.hpp>
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <math.h>
#include "geometry_msgs/msg/twist.hpp"
// #include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;
using namespace cv;

class OneLaneFollower : public rclcpp::Node
{
public:
    OneLaneFollower() : Node("lane_follower")
    {   
      
        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/dbImage", 10, std::bind(&OneLaneFollower::binary_thresholding, this, std::placeholders::_1));
        

  
        
    }

    void binary_thresholding(const sensor_msgs::msg::Image::SharedPtr msg) {
      // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get image dimensions
        int height = cv_ptr->image.rows/2;
        int width = cv_ptr->image.cols;
        int nLines = 10;
        // Divide the image into 10 equal horizontal parts
        int step = height / nLines;
        std::vector<int> y_coords(nLines);
        for (int i = 0; i < nLines; ++i)
        {
            y_coords[i] = i * step + step / 2 + height/2; // Middle of each horizontal section
        }

        // Vector to store leftmost and rightmost points for each line
        std::vector<std::pair<cv::Point, cv::Point>> lane_edges;

        // Iterate over each horizontal line
        for (int y : y_coords)
        {
            int leftmost_x = width;   // Start with max possible x
            int rightmost_x = 0;      // Start with min possible x
            bool found = false;

            for (int x = 0; x < width; ++x)
            {
                if (cv_ptr->image.at<uchar>(y, x) == 255) // Assuming white pixels represent the lane
                {
                    found = true;
                    if (x < leftmost_x)
                    {
                        leftmost_x = x;
                    }
                    if (x > rightmost_x)
                    {
                        rightmost_x = x;
                    }
                }
            }

            if (found)
            {
                lane_edges.emplace_back(cv::Point(leftmost_x, y), cv::Point(rightmost_x, y));
            }
        }

        // Optionally, you can visualize the results
        cv::Mat output_image;
        cv::cvtColor(cv_ptr->image, output_image, cv::COLOR_GRAY2BGR);
        for (const auto &edge : lane_edges)
        {
            cv::circle(output_image, edge.first, 3, cv::Scalar(0, 255, 0), -1);  // Green circle for leftmost point
            cv::circle(output_image, edge.second, 3, cv::Scalar(0, 0, 255), -1); // Red circle for rightmost point
        }

        vector <int> labels(lane_edges.size());
        int count_0 = 0, count_1 = 1, count_2 = 0;
        for (int i = 0 ; i<lane_edges.size() ; i++) {
            if ((lane_edges[i].second.x - lane_edges[i].first.x) < 30) {
                if (lane_edges[i].first.x<width/2) {
                    labels[i] = 0;
                    count_0++;
                }
                else {
                    labels[i] = 1;
                    count_1++;
                }
            }
            else {
                labels[i] = 2;
                count_2++;
            }

        }

        int lane_status ; // 0 for single lane(left), 1 for single lane(right), 2 for double lane 
        if (count_0 > count_2 or count_1 > count_2) {
            if (count_0>count_1) lane_status = 0;
            else lane_status = 1;
        }
        else {
            lane_status = 2;
        }

        if (lane_status ==0 ) {
            cout<<"left lane"<<endl;
        }
        else if (lane_status == 1) {
            cout<<"right lane"<<endl;
        }
        else if (lane_status == 2){
            cout<<"double lane"<<endl;
        }
        else {
            cout<<"THere is some error"<<endl;
        }
        // Show the output image
        cv::imshow("Lane Detection", output_image);
        cv::waitKey(1);
    }

private:

    

    // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;


   

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto lane_follower = std::make_shared<OneLaneFollower>();
    
    rclcpp::spin(lane_follower);
    rclcpp::shutdown();
    return 0;
}

