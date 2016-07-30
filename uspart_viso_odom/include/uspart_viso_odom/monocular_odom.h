#ifndef MONOCULAR_ODOM_H
#define MONOCULAR_ODOM_H

// ROS Libraries
#include "ros/ros.h"

// OpenCV
#include <opencv2/core/core.hpp>

// ROS message libraries 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

/**\class MonoOdom
 * \brief Abstract class that interfaces monocular odometry  algorithm.
 * 
 * MonoOdom is a pure abstract class that provides an interface to implement monocular vision odometry algorithms. 
 */
namespace monocularOdom{
    class MonocularOdom{
        protected:  
           double fx_, fy_, cx_, cy_;                   //!< Camera intrisic parameters    
	       uint32_t width_, height_;			        //!< Image size
	       double k1_, k2_, k3_, p1_, p2_;     			//!< Radial abd tangential distortion parameters
    
           sensor_msgs::RegionOfInterest roi_;          //!< Region of interest
           bool camera_calibrated_;                     //!< Camera calibration flag
           tf::Matrix3x3 imuToCamRot_;                  //!< Rotation of camera in respect to imu

        public:
	       //! Empty Constructor.
	       MonocularOdom();
		
	       //! Destructor.
	       ~MonocularOdom();
	
	       //! [PURE VIRTUAL] Initialize specific paremeters
	       virtual void initializeParameters(ros::NodeHandle &node) = 0;
		
	       //! [Pure VIRTUAL] Reinitialize parameters.
	       virtual void resetParameters(void) = 0;
    
            //! [VIRTUAL] Sets camera calibration parameters.
	       virtual void cameraCalibration(const sensor_msgs::CameraInfoConstPtr &camera_info);
		
	       //! [PURE VIRTUAL] User defined VO algorithm. For image encoding see cv_bridge::cvImage class.
	       virtual  int runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &imageEncoding) = 0;
    };    
}


#endif