#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
// #include <mlpack/methods/dbscan/dbscan.hpp>

using namespace std;
using namespace cv;
class LaneFollower : public rclcpp::Node
{
public:
    LaneFollower() : Node("lane_follower")
    {
        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_forward/image_raw", 10, std::bind(&LaneFollower::binary_thresholding, this, std::placeholders::_1));
        

    }

private:

    void binary_thresholding(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
        //cout<<" RECEIVED IMAGE"<<endl; 
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cv_image = cv_ptr->image, gray_image, thresholded_image, edges, image;
        int rows = cv_image.rows;
    
        image = cv_image(cv::Range(rows/2,rows),cv::Range::all());
        // cv::medianBlur(cv_image,cv_image,5);
        cv::cvtColor(image, gray_image,CV_BGR2GRAY); 

        cv::medianBlur(gray_image,gray_image,5);
 
        cv::inRange(gray_image, 120, 140, thresholded_image);
        
        cv::Canny(thresholded_image,edges,50,75, 5,true);
        
        Mat dilated;
        int dilation_size = 1;  // Adjust the dilation size as needed
        Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
        dilate(edges, edges, element);

        vector<cv::Vec4i> lines;
        HoughLinesP(edges, lines, 5, CV_PI / 180, 160, 100, 10);

        for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        double angle = atan2(l[3] - l[1], l[2] - l[0]); // Calculate angle in radians

        // Filter lines with angle greater than 0.5 radians
        
        line(image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
        

        imshow("window",edges);
        cv::waitKey(10);
    }
        // cout<<endl;


    

    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;

};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto lane_follower = std::make_shared<LaneFollower>();
    
    // Example: process a single point (x, y)
    // ipm_node->process_point(100, 150); // Replace (100, 150) with desired point coordinates
    
    rclcpp::spin(lane_follower);
    rclcpp::shutdown();
    return 0;
}
    