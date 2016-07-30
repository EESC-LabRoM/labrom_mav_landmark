#ifndef GENERIC_H
#define GENERIC_H

//ROS Libraries
#include <geometry_msgs/PointStamped.h>

// VisoOdom libraries
#include <uspart_viso_odom/viso_odom.h> 

// OpenC libraries
#include "opencv2/highgui/highgui.hpp"

// Landmark libraries
#include <uspart_landing/landmark.h>
#include <uspart_landing/descriptor.h>

// CPP libraries
#include <vector>
#include <string>

namespace landmark{
    class Generic: public monocularOdom::MonocularOdom{
        private:
            int threshold_;                //!< Black and white threshold values
            Landmark blob_;                //!< Desired landmark to be learned
            Descriptor descriptor;         //!< Descriptor of the detected landmark
            
            double landmark_diag_;         //!< Landmark diagonal size (in meters) 
        
            Eigen::Vector3d xc_c_;              //!< Vector from optical center to center of ellipse
        public:
            //! Constructor.
            Generic();
            //! Destructor.
            ~Generic();
        
        	//! Inheritable virtual function (see MonocularOdom::initializeParameters)
	        void initializeParameters(ros::NodeHandle &node);
            
            //! Inheritable virtual function (see MonocularOdom::resetParameters)
            void resetParameters(void);
        
            //! Inheritable pure virtual fnction. Algorithm routine 
            int runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &imageEncoding);
                    
            // Pre-processing image;
            void preProcessing(const cv::Mat &input_image, cv::Mat &output_image);
            
            // Segmentation image;
            bool segmentation(const cv::Mat &input_image);
        
            // Pose estimation;
            bool poseEstimation(const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated);

            // Pose estimation DEPRECATED;
            bool poseEstimationOld(nav_msgs::Odometry &odom_estimated);
            
            // Atualizate ROI
            void assembleROI(const int size);
        
            // Low pass filter
            bool lowPassFilter(nav_msgs::Odometry &odom_estimated, bool initialize=false);
    };
} // landmark namespace
#endif