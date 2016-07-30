#ifndef CIRCULAR_H
#define CIRCULAR_H

//ROS Libraries
#include <geometry_msgs/PointStamped.h>

// VisoOdom libraries
#include <uspart_viso_odom/viso_odom.h> 

// OpenC libraries
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// Landmark libraries
#include <uspart_landing/landmark.h>
#include <uspart_landing/descriptor.h>

// CPP libraries
#include <vector>
#include <string>

namespace landmark{
    class Circular: public monocularOdom::MonocularOdom{
        private:
            int threshold_;                     //!< Black and white threshold values
            double landmark_diag_;              //!< Landmark diagonal size (in meters) 
            Landmark blob_;                     //!< Desired landmark to be learned
            Descriptor descriptor_;             //!< Descriptor of the detected landmark
            std::vector<cv::Point> contours_;   //!< Points that describe the contour of an ellipse
            cv::RotatedRect ellipse_;           //!< Ellipse parameters
            Eigen::Vector3d xc_c_;              //!< Vector from optical center to center of ellipse

        public:
            //! Constructor.
            Circular();
            //! Destructor.
            ~Circular();
        
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
        
            // Compute ellipse parameters
            bool getEllipseParameters(void);
        
            // Estimate pose
            bool estimatePose(const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated);
        
            // EStimate Pose using Homography
            bool estimatePoseHom(nav_msgs::Odometry &odom_estimated);
                
            // Atualizate ROI
            void assembleROI(const int size);
    
            // Low pass filter
            bool lowPassFilter(nav_msgs::Odometry &odom_estimated, bool initialize=false);
    };
} // landmark namespace
#endif