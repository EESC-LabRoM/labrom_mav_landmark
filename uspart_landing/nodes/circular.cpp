#include <uspart_landing/circular.h> 

//namespace monocularOdom{
namespace landmark{
    /**
    * RunAlgorithm. See Mono_odom::runAlgorithm()
    */
    int Circular::runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &image_encoding){   
        
        double start_time = ros::Time::now().toSec() ;
        
        static int counter=0, state = 0;   
        bool result = false;
        cv::Mat mat;
        
        // Free memory
        this->contours_.clear();
 
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
                       result =  this->getEllipseParameters();
                       // Remain in this state until counter greater than a threshold
                       ++counter;
                       if (counter > 5){
                            // Estimate pose
                            this->estimatePose(imu, odom_estimated);
                           // Low pass filter (validates estimation)
                           this->lowPassFilter(odom_estimated, true);
                           this->blob_.lock = true;
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
                   result = this->segmentation(mat);   
                    // Incremet counter each time a target has been correctly detected
                   if (result == true){
                       result =  this->getEllipseParameters();
                        // Estimate pose
                        this->estimatePose(imu, odom_estimated);
                        // Low pass filter (validates estimation)
                        result = this->lowPassFilter(odom_estimated);
                        if (result == true)
                            counter = std::min(5, counter+1);
                        else
                            --counter; 
                    } else 
                        --counter; 
                    
                        
        }
        
        // Check if counter reached minimum value
        if (counter < 0)
            state = 0;    
        
        //this->preProcessing(input_image, output_image);
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
    Circular::Circular() : MonocularOdom() {};

    /**
    * Destructor (empty).
    */
    Circular::~Circular(){};
    
    /**
    * Initialize parameters from launch file
    * @param node a ROS node handle
    */
    void Circular::initializeParameters(ros::NodeHandle &node){   
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

       // Rotation of camera in respect to imu
       double roll_deg, pitch_deg, yaw_deg;   
       node.param("rot_x", roll_deg, 0.0);
       node.param("rot_y", pitch_deg, 0.0);
       node.param("rot_z", yaw_deg, 0.0);
       
       this->imuToCamRot_.setRPY(roll_deg*M_PI/180, pitch_deg*M_PI/180, yaw_deg*M_PI/180); 
        
        
       this->blob_.setBase(base);
       this->blob_.setRange(range);

    }

    /**
    * Reset parameters for default values
    */
    void Circular::resetParameters(void){
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
    void Circular::preProcessing(const cv::Mat &input_image, cv::Mat &output_image){
        // Copy only region of interest
        output_image = cv::Mat(input_image, cv::Range(this->roi_.y_offset, this->roi_.y_offset+this->roi_.height-1), cv::Range(this->roi_.x_offset, this->roi_.x_offset+this->roi_.width-1) ).clone();  
       /****************************** COLORFUL TARGET *********************************/
        // Colorspace transform
        //std::vector<cv::Mat> rgb;
        //cv::split(output_image, rgb);
        //output_image = 5*(rgb[1]-rgb[0]) + (rgb[1]-rgb[2]);
        /***************************** BLACK AND WHITE *********************************/
        // Colorspace transform
        //cv::cvtColor(output_image, output_image, CV_RGB2GRAY);
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
    bool Circular::segmentation(const cv::Mat &input_image){
        
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat mat = input_image.clone();
        
        // Contours
        cv::findContours( mat, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cv::Point(this->roi_.x_offset, this->roi_.y_offset) );
        // Landmark detection
        int result = this->blob_.detect(contours, this->descriptor_); 
        // If result is geq to zero, then a landmark has been detected
        if (result < 0)
            return false;
        else{
          /*  int child = hierarchy[result][2];
            Descriptor child_descriptor; 
            child_descriptor.assemble(contours[child], (INVARIANT | DYNAMIC) );
            this->descriptor_.dynamic.theta = child_descriptor.dynamic.theta;*/
            this->contours_ = contours[result];
            return true;
        }

    }
    
    /**
    * Computes the parameters of an ellipse from contours
    */
    bool Circular::getEllipseParameters(void){
        
        // Transform from pixel coordinate to image frame coordinate
        int numel = this->contours_.size();
        cv::Mat M(3, numel, CV_64F, 1.0);
        double ax = - this->cx_ / this->fx_ ;
        double ay = - this->cy_ / this->fy_ ;
        for (int j=0; j<numel; j++){
                M.at<double>(0, j) = this->contours_.at(j).x / this->fx_ + ax ;
                M.at<double>(1, j) = this->contours_.at(j).y / this->fy_ + ay ;
        }
       
        // Find center of the oblique circular cone;
        this->xc_c_(0) = sum(M.row(0))[0]/numel;
        this->xc_c_(1) = sum(M.row(1))[0]/numel;
        this->xc_c_(2) = 1;

        // Computing ellipse parameters
        double alpha = -atan2(xc_c_(1), -xc_c_(2));
        double beta  = -atan2(xc_c_(0), -xc_c_(1)*sin(alpha)-xc_c_(2)*cos(alpha) );
        cv::Mat R    = (cv::Mat_<double>(3,3) << cos(beta), 0, -sin(beta),0, 1, 0, sin(beta), 0, cos(beta) ) * 
                       (cv::Mat_<double>(3,3) << 1, 0, 0, 0, cos(alpha), -sin(alpha), 0, sin(alpha), cos(alpha) );
        
        cv::Mat Mh(R*M);
        std::vector<cv::Point2f> contours;

        for (int j=0; j<Mh.cols; j++){
            contours.push_back(cv::Point2f(  Mh.at<double>(0, j) / Mh.at<double>(2, j), 
                                             Mh.at<double>(1, j) / Mh.at<double>(2, j) 
                                          ) ); 
        }
     
        this->ellipse_ = cv::fitEllipse(contours);
        
        return true;
    }
    
    /**
    * Estimate pose from a rotatedRect ellipse object
    * @param imu necesssary for disabiguating
    * @param odom_estimated output data
    */
    bool Circular::estimatePose(const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated){
        
        // Compute transformation from camera to speed frame using IMU data
        Eigen::Matrix3d Rc, R_roll, R_pitch, R_yaw;
        double roll=0, pitch=0, yaw=0;
     

        // The lines below are for computing rotation matrix from camera to world frame from IMU data
        tf::Quaternion   q(imu.orientation.x,
                           imu.orientation.y,
                           imu.orientation.z,
                           imu.orientation.w);  
       
        tf::Matrix3x3 R(q);           
        R.getRPY(roll, pitch, yaw);
        yaw = this->descriptor_.dynamic.theta;

        R_roll << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);
        R_pitch << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
        R_yaw << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        Rc << R_yaw*R_pitch*R_roll;
        
        Eigen::Vector3d xc_h;
        xc_h = Rc*this->xc_c_;
        // Compute height
        double major_axis = std::max(this->ellipse_.size.height, this->ellipse_.size.width );
        double minor_axis = std::min(this->ellipse_.size.height, this->ellipse_.size.width );
        double n = this->landmark_diag_ * minor_axis / (major_axis * major_axis);
        // Adjust normalized vector in respect to height and assemble ROS message        
        odom_estimated.pose.pose.position.x = n*xc_h(0)/xc_h(2);
        odom_estimated.pose.pose.position.y = n*xc_h(1)/xc_h(2);
        odom_estimated.pose.pose.position.z = n;
        
        Eigen::Quaterniond qt(Rc);
        odom_estimated.pose.pose.orientation.x = qt.x();
        odom_estimated.pose.pose.orientation.y = qt.y();
        odom_estimated.pose.pose.orientation.z = qt.z();
        odom_estimated.pose.pose.orientation.w = qt.w();
        
        return true;
    }
    
    
    /**
    * Computes the pose using homography matrix (NOT WORKING PROPERLY)
    * @param contours contours from ellipse
    */
    bool Circular::estimatePoseHom(nav_msgs::Odometry &odom_estimated)
    {
        
        std::vector<cv::Point3f> circle;
        cv::Mat rvec, tvec;
        
        // Assemble circle in the object plane
        double radius = this->landmark_diag_ / 2.0;
        
        for (int i=0; i<4; ++i){
            double theta = atan2(this->contours_.at(i).y - this->descriptor_.dynamic.vc, 
                                 this->contours_.at(i).x - this->descriptor_.dynamic.uc);
            circle.push_back(cv::Point3f(radius*cos(theta),-radius*sin(theta),0.0));    
        }
        
        // Camera Calibration Matrix
        cv::Matx33f cameraMatrix(
                                 this->fx_,    0     , this->cx_,
                                     0    ,this->fy_ , this->cy_,
                                     0    ,    0     ,     1    );
        // Distortion parameters
        cv::Vec4f distParam(this->k1_, this->k2_, this->p1_, this->p2_); // distortion parameters
        // PnP solver
        cv::solvePnP(circle, this->contours_, cameraMatrix, distParam, rvec, tvec);
        
        // Rodrigues to rotation matrix
        cv::Matx33d r;
        cv::Rodrigues(rvec, r);
        Eigen::Matrix3d wRo;
        wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

        Eigen::Matrix4d T, transform; 
        transform.topLeftCorner(3,3) = wRo;
        transform.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
        transform.row(3) << 0,0,0,1;

        Eigen::Matrix3d rot = wRo;
        Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

        
        odom_estimated.pose.pose.position.x = transform(0,3);
        odom_estimated.pose.pose.position.y = transform(1,3);
        odom_estimated.pose.pose.position.z = transform(2,3);
        
        odom_estimated.pose.pose.orientation.x = rot_quaternion.x();
        odom_estimated.pose.pose.orientation.y = rot_quaternion.y();
        odom_estimated.pose.pose.orientation.z = rot_quaternion.z();
        odom_estimated.pose.pose.orientation.w = rot_quaternion.w();     
        
    }
    
     /**
    * ROI assembling. Computes the region of interest of an image
    * @param size the desired squared ROI half size
    */
    void Circular::assembleROI(const int size){
        int span = std::max(size, (int) (2*this->descriptor_.dynamic.radius) );
        this->roi_.x_offset = std::max(0, (int) this->descriptor_.dynamic.uc - span);
        this->roi_.y_offset = std::max(0, (int) this->descriptor_.dynamic.vc - span);
        this->roi_.height   = std::min( (int) (this->height_ - this->roi_.y_offset), 2*span);
        this->roi_.width    = std::min( (int) (this->width_ - this->roi_.x_offset), 2*span);     
    }

    
    /**
    * Low pass filter. Assures the pose estimation process is robust by preventing leaps
    * \returns {True when the estimation is correct}
    * @param odom_estimated the current estimation
    * @param initialize indicates that no previous estimation is available (default value: false)
    */
    bool Circular::lowPassFilter(nav_msgs::Odometry &odom_estimated, bool initialize){
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
    landmark::Circular algorithm;

    // Monocular VO Node
    visoOdom::MonocularNode node(algorithm);
    
    node.spin();
}
    