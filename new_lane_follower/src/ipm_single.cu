#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "cuda_runtime.h"
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <math.h>

#include "cv_bridge/cv_bridge.h"
#define BLOCKS 64
#define imin(a,b) (a<b?a:b)

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;
using std::placeholders::_1;

template<typename T>
__global__ void dev_matmul(const T *a, const T *b, T *output, int rows) {
    // a is 3x3 matrix
    // b is 3x1 set of matrices
    // output is 3x1 set of matrices
    int thread_id = threadIdx.x;
    int block_id = blockIdx.x;
    int offset = block_id * (rows + BLOCKS - 1) / BLOCKS + thread_id;

    if (offset < rows) {
        for (int i = 0; i < 3; ++i) {
            double temp = 0;
            for (int k = 0; k < 3; ++k) {
                temp += a[i * 3 + k] * b[offset * 3 + k];
            }
            output[offset * 3 + i] = temp;
        }
    }
}

void matmul(double *a, double *b, double *c){
	//a is 3x3
	//b is 3x1
	for(int i=0; i < 3; ++i){
		double temp=0;
		for(int k=0; k < 3; ++k){
			temp += a[i*3 + k]*b[k]; 
		}
		c[i] = temp;
	}
}

__global__ void dot(double* a, double* b, double* c, int rows) {
	int thread_id = threadIdx.x;
	int block_id = blockIdx.x;

	int offset = block_id*(rows+BLOCKS-1)/BLOCKS + thread_id;
	if(offset < rows){
		double temp=0;
		for(int i=0; i < 3; ++i){
		    temp += a[i]*b[offset*3+i];    
		}
		c[offset] = temp;
	}
	
	//debug
	//if(offset == 0){
		//for(int i=0;i < 3; ++i){
			//printf("gpu : %f ", c[0]);
		//}
		//printf("\n");
	//}

}

class IPM : public rclcpp::Node {
public:
    IPM() : Node("ipm") {
        subscription_caminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_forward/camera_info", 10,
            std::bind(&IPM::call, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/igvc/ipm", 10);
    }


    void call(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->camera_info = *msg;
        process_point(100,150);
    }

    void process_point(int x, int y) {
        sensor_msgs::msg::PointCloud2 pub_pointcloud;
        auto cloud_msg = std::make_unique<PointCloud>();

        // Process the single point
        float roll = 0;
        float pitch = -24 * M_PI / 180;
        float yaw = 0;
        float h = 1.41;

        vector<double> k(9), nor(3), uv(3);

        double cy, cr, sy, sr, sp, cp;
        cy = cos(yaw);
        sy = sin(yaw);
        cp = cos(pitch);
        sp = sin(pitch);
        cr = cos(roll);
        sr = sin(roll);
        k[0] = cr * cy + sp * sr + sy;
        k[1] = cr * sp * sy - cy * sr;
        k[2] = -cp * sy;
        k[3] = cp * sr;
        k[4] = cp * cr;
        k[5] = sp;
        k[6] = cr * sy - cy * sp * sr;
        k[7] = -cr * cy * sp - sr * sy;
        k[8] = cp * cy;

        nor[0] = 0;
        nor[1] = 1.0;
        nor[2] = 0;

        // Calculate UV
        matmul(k.data(), nor.data(), uv.data());

        // Inverse camera matrix
        auto caminfo = this->camera_info.k;
        Eigen::Map<Matrix<double, 3, 3, RowMajor>> mat(caminfo.data());
        mat = mat.inverse();
        double *inv_caminfo = mat.data();

        vector<double> kin_uv(3), denom(1);
        double *d_uv, *d_caminfo, *d_kin_uv, *d_denom;

        // Device allocations
        cudaMalloc((void**)&d_kin_uv, sizeof(double) * 3);
        cudaMalloc((void**)&d_caminfo, sizeof(double) * 9);
        cudaMalloc((void**)&d_denom, sizeof(double));

        // Prepare UV homogeneous coordinates
        double uv_hom[3] = { static_cast<double>(x), static_cast<double>(y), 1 };

        // Copy to device
        cudaMemcpy(d_caminfo, inv_caminfo, sizeof(double) * 9, cudaMemcpyHostToDevice);
        cudaMemcpy(d_uv, uv_hom, sizeof(double) * 3, cudaMemcpyHostToDevice);

        // Launch matrix multiplication
        dev_matmul<<<1, 1>>>(d_caminfo, d_uv, d_kin_uv, 1);
        cudaMemcpy(kin_uv.data(), d_kin_uv, sizeof(double) * 3, cudaMemcpyDeviceToHost);

        // Calculate the denominator for the mapping
        denom[0] = kin_uv[2];

        pcl::PointXYZ vec;
        vec.x = h * kin_uv[2] / denom[0];
        vec.y = -h * kin_uv[0] / denom[0];
        vec.z = -h * kin_uv[1] / denom[0];
        cloud_msg->points.push_back(vec);

        // Clean up
        cudaFree(d_kin_uv);
        cudaFree(d_caminfo);
        cudaFree(d_denom);

        // Prepare the PointCloud message for publishing
        cloud_msg->height = 1;
        cloud_msg->width = cloud_msg->points.size();
        cloud_msg->is_dense = false;
        pcl::toROSMsg(*cloud_msg, pub_pointcloud);
        pub_pointcloud.header.frame_id = "base_link";
        pub_pointcloud.header.stamp = rclcpp::Clock().now();

        // Publish the cloud
        publisher_->publish(pub_pointcloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_caminfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    sensor_msgs::msg::CameraInfo camera_info;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto ipm_node = std::make_shared<IPM>();
    
    // Example: process a single point (x, y)
    // ipm_node->process_point(100, 150); // Replace (100, 150) with desired point coordinates
    
    rclcpp::spin(ipm_node);
    rclcpp::shutdown();
    return 0;
}
