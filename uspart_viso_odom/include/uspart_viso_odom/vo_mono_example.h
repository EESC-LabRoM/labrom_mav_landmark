#ifndef VO_MONO_EXAMPLE_H
#define VO_MONO_EXAMPLE_H

// VisoOdom libraries
#include <uspart_viso_odom/viso_odom.h> 

namespace monocularOdom{
namespace voMonoExample{
    class VOMonoExample: public MonocularOdom{
        private:
            int threshold_;                 //!< Black and white threshold value
        public:
            //! Constructor.
            VOMonoExample();
            //! Destructor.
            ~VOMonoExample();
        
        	//! Inheritable virtual function (see MonocularOdom::initializeParameters)
	        void initializeParameters(ros::NodeHandle &node);
            
            //! Inheritable virtual function (see MonocularOdom:: resetParameters)
            void resetParameters(void);
        
            //! Inheritable pure virtual function Algorithm routine
            int runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &imageEncoding);
        
    };
} // voMonoExample namespace
} // monocularOdom namespace

#endif