#ifndef MONOCULAR_NODE_H
#define MONOCULAR_NODE_H

// ROS Libraries
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
// OpenCV
#include <opencv2/core/core.hpp>

// VisoOdom libraries
#include <uspart_viso_odom/monocular_odom.h> 

// ROS message libraries 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"



namespace visoOdom{
    /**\class MonocularNode
    * \brief ROS Node for monocular visual odometry 
    * 
    * This class interfaces communication between ROS and monocular visual odometry algorithms.
    */
    class MonocularNode{
        public:
            //! Constructor
            MonocularNode(monocularOdom::MonocularOdom &monoOdomMethod);
        
            //! Destructor (empty)
            ~MonocularNode();
            
            //! ROS loop 
            void spin(void);
        
            //! IMU callback (ROS subscriber)
            void imuCallback(const sensor_msgs::ImuConstPtr &msg);
        
            //! Camera info callback (ROS subscriber)
            void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
            //! Image callback (ROS subscriber)
            void imageCallback(const sensor_msgs::ImageConstPtr &msg);
        
            //! Publish output image (ROS publisher)
            void publishImage(const cv::Mat& image, const std::string &imageEncoding);
            //! Publish estimated odometry (ROS Publisher)
            void publishOdometry(const nav_msgs::Odometry &odom_estimated);
        
        private:
            ros::NodeHandle node_;                          //!< ROS node handle
            
            // Subscribers
            ros::Subscriber imu_sub_;                       //!< IMU subscriber
            ros::Subscriber camera_info_sub_;               //!< Camera info Subscriber (calibration)
            image_transport::Subscriber image_sub_;         //!< Input image subscriber
        
            // Publishers
            ros::Publisher odom_pub_;                       //!< Estimated odometry publisher
            image_transport::Publisher image_pub_;          //!< Processed image publisher
            // ROS messages
            sensor_msgs::Imu imu_msg_;                      //!< Received IMU message 
            
            // Monocular odometry node
            monocularOdom::MonocularOdom* monocularOdom_;    
            tf::TransformBroadcaster br_;                   //!< Object that sends tf      
        
            // General purpose variables
            std::string name_identifier_;                   //!< Name that identifies the node
            std::string inertial_frame_id_;                 //!< Name of the frame that odometer is related to               
            std::string base_link_frame_id_;                //!< Name of the moving frame that the odometer reports to  
            std::string camera_frame_id_;                   //!< Name of the camera frame id                  

            bool publish_tf_;                               //!< If set to true, tf messages are published
    };
}

#endif