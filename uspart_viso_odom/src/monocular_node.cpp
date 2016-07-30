#include <uspart_viso_odom/monocular_node.h> 

namespace visoOdom{
    /**
    * Constructor. 
    * Input: 
    * MonocularOdom: Derived visual odometry class (algorith implementation)
    */
    MonocularNode::MonocularNode(monocularOdom::MonocularOdom &monoOdomMethod): node_("~"){
        // Upload parameters from launch or .yawl file
       node_.param<std::string>("name", name_identifier_, "mono_odom");
       node_.param<std::string>("inertial_frame_id", inertial_frame_id_, "world");
       node_.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
       node_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_link");    
       node_.param("publish_tf", publish_tf_,true);
   
       // Copy monocular odometry method class
	   monocularOdom_ = &monoOdomMethod;
       // Set parameters (specific to VO algorithm) 
	   monocularOdom_->initializeParameters(node_);
       // image transport for subscribing and publishing images through ROS
	   image_transport::ImageTransport it(node_);
        
       // Subscribers
	   //image_transport::TransportHints hints("compressed", ros::TransportHints());
       //image_sub_ = it.subscribe("camera/image_raw",1, &MonocularNode::imageCallback, this, hints );
	   image_sub_ = it.subscribe("/camera/image_raw",1, &MonocularNode::imageCallback, this );
	   camera_info_sub_ = node_.subscribe("/camera/camera_info",1,&MonocularNode::cameraInfoCallback, this);
	   imu_sub_ = node_.subscribe("/imu",1,&MonocularNode::imuCallback, this);

        // Publishers
	   odom_pub_ = node_.advertise<nav_msgs::Odometry>("/"+ name_identifier_+"/odometry",1);
	   image_pub_ = it.advertise("/"+name_identifier_+"/image", 1);
        
        
    }
    
    /**
    * Destructor.
    */
    MonocularNode::~MonocularNode(){;}
    
    /**
    * Spin: ROS loop function. 
    */
    void MonocularNode::spin(void){
        ros::spin();
    }
    
    /**
    * IMU callback. It copies the received message from IMU to a local class variable.
    */
    void MonocularNode::imuCallback(const sensor_msgs::ImuConstPtr &msg){
	   imu_msg_ = *msg;
    }
    
    /**
    * Camera info function handle. Call the MonoOdom camera calibration function handle
    */
    void MonocularNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg){
	
	   // Setting calibration parameters in the VO algorithm
       monocularOdom_->cameraCalibration(msg);
	  
       // Shutdown the subscriber after calibration
       camera_info_sub_.shutdown();
    
       ROS_INFO("Camera successfully calibrated!");
    }
    
    /**
    * Image function handle. It receives a image from the camera node, compiles sensors information , calls the vision algorithm and publishes the VO results in ROS environment.
    */
    void MonocularNode::imageCallback(const sensor_msgs::ImageConstPtr &msg){
	   cv_bridge::CvImagePtr cvImagePtr;
	
		try{
            cv::Mat original_image, output_image;
            nav_msgs::Odometry odom_estimated; 
            std::string encoding;
            
            // Converting image from ROS to OpenCV format
            cvImagePtr     = cv_bridge::toCvCopy(msg);
            original_image = cvImagePtr->image;
            encoding      = msg->encoding;
            odom_estimated.header = cvImagePtr ->header;
            // Calling monocular odometry routine
            int result = monocularOdom_->runAlgorithm(original_image, imu_msg_, odom_estimated, output_image, encoding); 
            // Publish image result only if image encoding is provided by VO handle
            this->publishImage(output_image, encoding);
            // Publish odometry only if it was successfully estimated
            if (result > 0)
                this->publishOdometry(odom_estimated);
            
        } catch (cv_bridge::Exception &e){
            ROS_ERROR("CV_BRIDGE Exception: %s", e.what());
        }
	
    }
    
    /** @todo Allow publishing different image format: mono8, rgb8, etc. */
    /**
    * Image publish: This function publishes a cv::Mat image in the ROS environment.
    */
    void MonocularNode::publishImage(const cv::Mat& image, const std::string &imageEncoding){
	   // Build message
	   sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), imageEncoding, image).toImageMsg();
	  ;
	   // Publish image
	   image_pub_.publish(image_msg);
    }
    
    /**
    * Odometry publish: This function publishes a state vector as an odometry message in the ROS environment.
    * The Odometry message timestamp is the same as the received message. See imageCallback() function. 
    */
    void MonocularNode::publishOdometry(const nav_msgs::Odometry &odom_estimated){
        // Publish odometry message
	    odom_pub_.publish(odom_estimated);
        
        if (this->publish_tf_){
            // TF message
           // 
           //tf::Quaternion q;

            // Obtaining the translation and rotation
            //q = tf::createQuaternionFromRPY(M_PI,0, tf::getYaw(odom_estimated.pose.pose.orientation)  );
            //transform.setRotation(q);
            //transform.setOrigin(-1 * tf::quatRotate(q,tf::Vector3(odom_estimated.pose.pose.position.x, odom_estimated.pose.pose.position.y, odom_estimated.pose.pose.position.z)));

            // Publish TF message
            //this->br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->inertial_frame_id_, this->camera_frame_id_) );
                          
            tf::Stamped<tf::Transform> transform;
            geometry_msgs::PoseStamped pose;
            pose.pose = odom_estimated.pose.pose;
            pose.header = odom_estimated.header;
            tf::poseStampedMsgToTF(pose, transform);
            
            this->br_.sendTransform(tf::StampedTransform(transform, transform.stamp_, this->inertial_frame_id_, transform.frame_id_ ) );
        }
    }
    
}