#ifndef LEARNING_H
#define LEARNING_H

// VisoOdom libraries
#include <uspart_viso_odom/viso_odom.h> 

// OpenC libraries
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
// HLanding libraries
#include <uspart_landing/descriptor.h>
#include <uspart_landing/landmark.h>

// CPP libraries
#include <vector>
#include <string>

namespace landmark{
    class Learning: public monocularOdom::MonocularOdom{
        private:
            int threshold_;                 //!< Black and white threshold values
            Landmark blob_;       //!< Desired landmark to be learned
        
            ros::Time begin_;               //!< Time the node starts learning
            int duration_;                  //!< time in secs that the node shall learn
        
        
        public:
            //! Constructor.
            Learning();
            //! Destructor.
            ~Learning();
        
        	//! Inheritable virtual function (see MonocularOdom::initializeParameters)
	        void initializeParameters(ros::NodeHandle &node);
            
            //! Inheritable virtual function (see MonocularOdom::resetParameters)
            void resetParameters(void);
        
            //! Inheritable pure virtual fnction. Algorithm routine 
            int runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &imageEncoding);
                    
            // Crops image;
            void cropImage(const cv::Mat &input_image, cv::Mat &output_image);
        
            //! RGB-> Gray
            void rgbToGray(const cv::Mat &input_image, cv::Mat &output_image);
        
            //! Gray -> Black and white
            void grayToBW(const cv::Mat &input_image, cv::Mat &output_image);
        

    };
} // landmark namespace
#endif