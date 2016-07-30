#include <uspart_landing/learning.h> 

//namespace monocularOdom{
namespace landmark{
    /**
    * RunAlgorithm. See Mono_odom::runAlgorithm()
    */
    int Learning::runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &image_encoding){   
        static int state = 0;   
        
        cv::Mat mat, bw;
        std::vector<cv::Point> contours;
        landmark::Descriptor descriptor;
        
        // state 0: camera is not calibrated
        // state 1: camera is calibrated 
        if (state==0){
            if(this->camera_calibrated_ == true){
                this->resetParameters();
                this->begin_ = ros::Time::now();
                state = 1;
            }else
                return -1;
        }

        // Converting image colorspace to gray
        if (image_encoding == "rgb8"){
            this->rgbToGray(input_image, mat);
        } else if (image_encoding != "mono8"){
            return -1;
        }
                
        // Converting gray image  to black and white.
        this->grayToBW(mat, bw);
    
        // Crop original image into ROI
        this->cropImage(bw, mat);
        
        // Learning process.
        cv::Point offset = cv::Point(this->roi_.x_offset,roi_.y_offset);
        bool result = (this->blob_).learnMinMax(mat, offset, descriptor, contours);
        
        // Adjust ROI when successfully found landmark.
        if (result == true){
            int span = std::max(30, (int) (2*descriptor.dynamic.radius) );
            this->roi_.x_offset = std::max(0, (int) descriptor.dynamic.uc - span);
            this->roi_.y_offset = std::max(0, (int) descriptor.dynamic.vc - span);
            this->roi_.height   = std::min( (int) (this->height_ - this->roi_.y_offset), 2*span);
            this->roi_.width    = std::min( (int) (this->width_ - this->roi_.x_offset), 2*span);
        }
          
        // Draw ROI rectangle on output image
        output_image = bw; 

       /* cv::rectangle(output_image,
                    cv::Point(this->roi_.x_offset, this->roi_.y_offset), cv::Point(this->roi_.x_offset+this->roi_.width , this->roi_.y_offset+this->roi_.height ), 
                    cv::Scalar( 255,255,255), 2 );
       */
        // Specify output image encoding
        image_encoding = "mono8";
       
        // Check if learning time is over
        double elapsed_time = ros::Time::now().toSec() - this->begin_.toSec();
        if ( elapsed_time > this->duration_ ) {
            std::cout << "********** LEARNING PROCESS FINISHED *************" << std::endl;
            std::cout << "-------- Base descriptor --------" << std::endl;
            this->blob_.getBase();
            std::cout << "-------- Range descriptor --------" << std::endl;
            this->blob_.getRange();
            
            this->begin_ = ros::Time::now();
        }
        
        return 0;
        
    }

        
    /**
    * Constructor (empty).
    */
    Learning::Learning() : MonocularOdom() {}

    /**
    * Destructor (empty).
    */
    Learning::~Learning(){;}

    /**
    * Initialize parameters from launch file
    * @param node a ROS node handle
    */
    void Learning::initializeParameters(ros::NodeHandle &node){   
       // Loading parameters from a launch file
	   node.param("threshold", this->threshold_, 100); 
	   node.param("learning_time", this->duration_, 20); 
    }

    /**
    * Reset parameters for default values
    */
    void Learning::resetParameters(void){
        // Reset region of interest
        roi_.x_offset = 0;
        roi_.y_offset = 0;
        roi_.width = this->width_;
        roi_.height = this->height_;
        
    }
             
    
    /**
    * Crop Image. Crops the image into the region of interest (RoI) area.
    * @param input_image an original image to be cropped.
    * @param output_image a new image that contains the ROI of original image.
    */
    void Learning::cropImage(const cv::Mat &input_image, cv::Mat &output_image){
        // Copy only region of interest
        
        output_image = cv::Mat(input_image, cv::Range(this->roi_.y_offset, this->roi_.y_offset+this->roi_.height-1), cv::Range(this->roi_.x_offset, this->roi_.x_offset+this->roi_.width-1) ).clone();                    
    }
    
    /**
    * RGB to Gray. Transforms color images to grayscale
    * @param input_image a RGB image 
    * @param output_image a grayscale output image
    */
    void Learning::rgbToGray(const cv::Mat &input_image, cv::Mat &output_image){
        // openCV function for colorspace transformation
        cv::cvtColor(input_image, output_image, CV_RGB2GRAY);
    }
    
    /**
    * Gray to black and white. Transforms grayscale images to black and white (binary).
    * @param input_image a grayscale image 
    * @param output_image a black and white output image
    */    
    void Learning::grayToBW(const cv::Mat &input_image, cv::Mat &output_image){
        // openCV function for thresholding
       cv::GaussianBlur( input_image, output_image, cv::Size( 3, 3 ), 0, 0);
       cv::threshold(input_image, output_image, this->threshold_, 255, cv::THRESH_BINARY_INV);

    }
    

} // landmark namespace
//} // monocularOdom namespace

/**************************** ROS program *****************************/

int main(int argc, char **argv){
    // Intialize ROS
    ros::init(argc,argv,"");

    // Visual odometry algorithm
    landmark::Learning algorithm;

    // Monocular VO Node
    visoOdom::MonocularNode node(algorithm);
    
    node.spin();
}