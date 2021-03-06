#include <uspart_landing/generic.h> 

//namespace monocularOdom{
namespace landmark{
    /**
    * RunAlgorithm. See Mono_odom::runAlgorithm()
    */
    int Generic::runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &image_encoding){   
        double start_time = ros::Time::now().toSec();
        static int counter=0, state = 0;   
        cv::Mat mat;
        bool result = false;
        
        switch (state){
            // Remain in this state until camera calibration is uploaded
            case 0: if(this->camera_calibrated_ == true){
                        this->resetParameters();
                        this->blob_.lock = false;
                        counter = 0;
                        state = 1;
                    }else
                        return -1;
                   // No break here intentionally..
                
            // Searching target
            case 1: // Image pre-processing (crop and thresh)
                    this->preProcessing(input_image, mat);
                    // Landmark detection
                    result = this->segmentation(mat);
                    // Reamin in this state until a landmark is found
                    if (result == true){ 
                        this->blob_.lock = true;
                        state = 2;
                    }
                    break;
                
            // A landmark candidate has been detected
            case 2: // ROI definition
                    this->assembleROI(100);
                    // Image pre-processing (crop and thresh)
                    this->preProcessing(input_image, mat);
                    // Landmark detection
                    result = this->segmentation(mat);   
                    // Incremet counter each time a target has been correctly detected
                    if (result == true){
                        // Remain in this state until counter greater than a threshold
                        ++counter;
                        if (counter > 5){
                            // Estimate pose
                            //this->poseEstimation(imu, odom_estimated);
                            this->poseEstimationOld(odom_estimated);
                            // Low pass filter (validates estimation)
                            this->lowPassFilter(odom_estimated, true);
                            state = 3;
                        }
                    } else
                        --counter;         
                    break;
                
            // Locked stated. Run pose estimation
            case 3: // ROI definition
                    this->assembleROI(30);
                    // Image pre-processing (crop and thresh)
                    this->preProcessing(input_image, mat);
                    // Landmark detection
                    result = this->segmentation(mat); 
                    // Check if landmark detected
                    if (result == true){
                        // Estimate pose
                        //this->poseEstimation(imu, odom_estimated);
                        this->poseEstimationOld( odom_estimated);
                        // Low pass filter (validates estimation)
                        result = this->lowPassFilter(odom_estimated);
                        if (result == true)
                            counter = std::min(5, counter+1);
                        else
                            --counter; 
                    } else 
                        --counter; 
                    
                
                    break;
                        
                    
        }
        
        // Check if counter reached minimum value
        if (counter < 0)
            state = 0;
                  
        // Draw ROI rectangle on output image
        output_image = input_image.clone();
        
        cv::rectangle(output_image,
                    cv::Point(this->roi_.x_offset, this->roi_.y_offset), cv::Point(this->roi_.x_offset+this->roi_.width , this->roi_.y_offset+this->roi_.height ), 
                    cv::Scalar( 0,255,0), 2 );
        image_encoding = "mono8";
        
        if (state==3 && result == true){
            double time = ros::Time::now().toSec() - start_time;
            std::cout << time << std::endl;
            return 1;       
        }else
            return 0;
        
        
    }

        
    /**
    * Constructor (empty).
    */
    Generic::Generic() : MonocularOdom() {}

    /**
    * Destructor (empty).
    */
    Generic::~Generic(){};
    
    /**
    * Initialize parameters from launch file
    * @param node a ROS node handle
    */
    void Generic::initializeParameters(ros::NodeHandle &node){   
       Descriptor base, range;
       // Loading parameters from a launch file
	   node.param("threshold", this->threshold_, 100); 
       node.param("min_score", this->blob_.min_score, 7);
       node.getParam("landmark_diagonal", this->landmark_diag_); 

        // Base descriptor parameters
       node.getParam("base_shape", base.invariant.shape);
       node.getParam("base_area", base.invariant.area);
       node.getParam("base_hu", base.invariant.hu);
        
       // (Similarity) range descriptor parameters
       node.getParam("range_shape", range.invariant.shape);
       node.getParam("range_area", range.invariant.area);
       node.getParam("range_hu", range.invariant.hu);
        
       this->blob_.setBase(base);
       this->blob_.setRange(range);
        
        // Rotation of camera in respect to imu
       double roll_deg, pitch_deg, yaw_deg;   
       node.param("rot_x", roll_deg, 0.0);
       node.param("rot_y", pitch_deg, 0.0);
       node.param("rot_z", yaw_deg, 0.0);
       
       this->imuToCamRot_.setRPY(roll_deg*M_PI/180, pitch_deg*M_PI/180, yaw_deg*M_PI/180);     
    }

    /**
    * Reset parameters for default values
    */
    void Generic::resetParameters(void){
        // Reset region of interest
        roi_.x_offset = 0;
        roi_.y_offset = 0;
        roi_.width = this->width_;
        roi_.height = this->height_;
        
    }
             
    /**
    * Pre-processing Image. Crops into ROI and changes image colorspace .
    * @param input_image an original image.
    * @param output_image pos-processed image.
    */
    void Generic::preProcessing(const cv::Mat &input_image, cv::Mat &output_image){
        // Copy only region of interest
        output_image = cv::Mat(input_image, cv::Range(this->roi_.y_offset, this->roi_.y_offset+this->roi_.height-1), cv::Range(this->roi_.x_offset, this->roi_.x_offset+this->roi_.width-1) ).clone();  
        /***************************** BLACK AND WHITE *********************************/
        // Low pass filter
        // cv::GaussianBlur( output_image, output_image, cv::Size( 3, 3 ), 0, 0);
        // Gray to black and white
        // cv::threshold(output_image, output_image, this->threshold_, 255, cv::THRESH_BINARY_INV); 
           
        // Threshold by image patch average
        int r = output_image.rows;
        int c = output_image.cols;
        int n = 2;
        
        if(this->blob_.lock == false)
            n = this->roi_.width/250 + 2;

        int qr = r/n;
        int qc = c/n;
        for (int w = 0; w<n; w++){
            for (int k = 0; k< n; k++){
                // Comput average
                int sum =0 ;
                for (int i=w*qr; i<(w+1)*qr; ++i){
                    for (int j=k*qc; j<(k+1)*qc; ++j){
                        sum += output_image.at<uchar>(i,j);
                    }
                }
                int average = sum/(qr*qc) - 20;
                // Apply threshold for the block
                for (int i=w*qr; i<(w+1)*qr; ++i){
                    for (int j=k*qc; j<(k+1)*qc; ++j){
                        if (output_image.at<uchar>(i,j) > average ){
                            output_image.at<uchar>(i,j) = 0;
                        } else{
                            output_image.at<uchar>(i,j) = 255;
                        }
                    }
                }
            }
        }
    
    
    }
    
    /**
    * Image segmentation. Segments the landmark and computes its descriptor. 
    * @param input_image source image that contains landmark 
    */
    bool Generic::segmentation(const cv::Mat &input_image){
        
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat mat = input_image.clone();
        
        // Contours
        cv::findContours( mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(this->roi_.x_offset, this->roi_.y_offset) );
        // Landmark detection
        int result = this->blob_.detect(contours, this->descriptor); 
        
        // If result is geq to zero, then a landmark has been detected
        if (result < 0)
            return false;

        // Transform from pixel coordinate to camera frame coordinate
        int numel = contours[result].size();
        cv::Mat M(3, numel, CV_64F, 1.0);
        double ax = - this->cx_ / this->fx_ ;
        double ay = - this->cy_ / this->fy_ ;
        
        for (int j=0; j<numel; j++){
                M.at<double>(0, j) = contours[result].at(j).x / this->fx_ + ax ;
                M.at<double>(1, j) = contours[result].at(j).y / this->fy_ + ay ;
        }
       
        // Find center of the oblique circular cone;
        this->xc_c_(0) = sum(M.row(0))[0]/numel;
        this->xc_c_(1) = sum(M.row(1))[0]/numel;
        this->xc_c_(2) = 1;
        
        return true;
    }
    
    /**
    * Pose estimation. Computes the relative pose between the camera and the landmark
    * @param odom_estimated cointaner for estimated pose.
    */
    bool Generic::poseEstimation(const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated){       
        // Compute transformation from camera to speed frame using IMU data
        Eigen::Matrix3d Rc, R_roll, R_pitch, R_yaw;
        double roll=0, pitch=0, yaw=0;
     
        // The lines below are for computing rotation matrix from camera to world frame from IMU data
        tf::Quaternion   q(imu.orientation.x,
                           imu.orientation.y,
                           imu.orientation.z,
                           imu.orientation.w);  
       
        tf::Matrix3x3 R(q);   
        //R = R*this->imuToCamRot_;
        R.getRPY(roll, pitch, yaw);
        
        // Use computed orientation
        yaw = this->descriptor.dynamic.theta;
        
        R_roll << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);
        R_pitch << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
        R_yaw << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        Rc << R_yaw*R_pitch*R_roll;
        
        Eigen::Vector3d xc_h;
        xc_h = Rc*this->xc_c_;

       // Estimate height
       double height = 0.5*( this->landmark_diag_) / std::sqrt(      
           pow((this->descriptor.dynamic.radius)*cos(this->descriptor.dynamic.theta)/this->fx_,2) +
           pow((this->descriptor.dynamic.radius)*sin(this->descriptor.dynamic.theta)/this->fy_,2) 
           ) ;
        // Adjust normalized vector in respect to height and assemble ROS message        
        odom_estimated.pose.pose.position.x = height*xc_h(0)/xc_h(2);
        odom_estimated.pose.pose.position.y = height*xc_h(1)/xc_h(2);
        odom_estimated.pose.pose.position.z = -height/xc_h(2);
        
        Eigen::Quaterniond qt(Rc);
        odom_estimated.pose.pose.orientation.x = qt.x();
        odom_estimated.pose.pose.orientation.y = qt.y();
        odom_estimated.pose.pose.orientation.z = qt.z();
        odom_estimated.pose.pose.orientation.w = qt.w();
                
        return true;
    }
    
    /**
    * Pose estimation. Computes the relative pose between the camera and the landmark
    * @param odom_estimated cointaner for estimated pose.
    */
    bool Generic::poseEstimationOld(nav_msgs::Odometry &odom_estimated){
       double height;
       
       // Estimate height
       height = 0.5*( this->landmark_diag_) / std::sqrt(      
           pow((this->descriptor.dynamic.radius)*cos(this->descriptor.dynamic.theta)/this->fx_,2) +
           pow((this->descriptor.dynamic.radius)*sin(this->descriptor.dynamic.theta)/this->fy_,2) 
           ) ;
       
       // Estimate translation
       double theta = this->descriptor.dynamic.theta;
       double xb = height*(this->descriptor.dynamic.uc - this->cx_)/this->fx_;
       double yb = height*(this->descriptor.dynamic.vc - this->cy_)/this->fy_;   
              
       odom_estimated.pose.pose.position.x = xb*cos(theta) - yb*sin(theta); 
       odom_estimated.pose.pose.position.y = xb*sin(theta) + yb*cos(theta);   
       odom_estimated.pose.pose.position.z = height;       
          
        theta = this->descriptor.dynamic.theta;
        if(theta > M_PI)
            theta -= 2*M_PI;
        else if (theta < -M_PI)
            theta += 2*M_PI;
        
        this->descriptor.dynamic.theta = theta;
        
       odom_estimated.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, this->descriptor.dynamic.theta );
       
       return true;
    }
    
    /**
    * ROI assembling. Computes the region of interest of an image
    * @param size the desired squared ROI half size
    */
    void Generic::assembleROI(const int size){
        int span = std::max(size, (int) (2*this->descriptor.dynamic.radius) );
        this->roi_.x_offset = std::max(0, (int) this->descriptor.dynamic.uc - span);
        this->roi_.y_offset = std::max(0, (int) this->descriptor.dynamic.vc - span);
        this->roi_.height   = std::min( (int) (this->height_ - this->roi_.y_offset), 2*span);
        this->roi_.width    = std::min( (int) (this->width_ - this->roi_.x_offset), 2*span);     
    }

    /**
    * Low pass filter. Assures the pose estimation process is robust by preventing leaps
    * \returns {True when the estimation is correct}
    * @param odom_estimated the current estimation
    * @param initialize indicates that no previous estimation is available (default value: false)
    */
    bool Generic::lowPassFilter(nav_msgs::Odometry &odom_estimated, bool initialize){
        static geometry_msgs::PointStamped previous_estimation;
        
        if (initialize == true){
            previous_estimation.point = odom_estimated.pose.pose.position;
            previous_estimation.header.stamp = odom_estimated.header.stamp;
            return true;            
        }
            
        // Time interval between estimation 
        double dt = 0.001*(0.001*(0.001*(odom_estimated.header.stamp.nsec - previous_estimation.header.stamp.nsec)))             +              (odom_estimated.header.stamp.sec - previous_estimation.header.stamp.sec); 
        
        // Average velocity during dt
        double dx = (odom_estimated.pose.pose.position.x - previous_estimation.point.x)/dt ;
        double dy = (odom_estimated.pose.pose.position.y - previous_estimation.point.y)/dt ;
        double dz = (odom_estimated.pose.pose.position.z - previous_estimation.point.z)/dt ;    
        
        // Check if estimated speeds are within a threshold
        if (std::fabs(dx) > 1 || std::fabs(dy) > 1 || std::fabs(dz) > 1 )
            return false;
        
        // Update speed
        odom_estimated.twist.twist.linear.x = dx ;
        odom_estimated.twist.twist.linear.y = dy ;
        odom_estimated.twist.twist.linear.z = dz ;
        
        // Update previous value
        previous_estimation.point = odom_estimated.pose.pose.position;
        previous_estimation.header.stamp = odom_estimated.header.stamp;
        return true;
    }
    
} // landmark namespace


/**************************** ROS program *****************************/

int main(int argc, char **argv){
    // Intialize ROS
    ros::init(argc,argv,"");

    // Visual odometry algorithm
    landmark::Generic algorithm;

    // Monocular VO Node
    visoOdom::MonocularNode node(algorithm);
    
    node.spin();
}