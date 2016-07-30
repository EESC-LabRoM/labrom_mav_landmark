#include <uspart_viso_odom/vo_mono_example.h> 

namespace monocularOdom{
namespace voMonoExample{
    /**
    * Constructor (empty).
    */
    VOMonoExample::VOMonoExample() : MonocularOdom() {;}

    /**
    * Destructor (empty).
    */
    VOMonoExample::~VOMonoExample(){;}
    
    /**
    * Overridable function inherited from base class (MonocularOdom). Sets parameters from .launch or .yaml file
    */
    void VOMonoExample::initializeParameters(ros::NodeHandle &node){
       	// Loading parameters from a launch file
	   node.param("threshold", threshold_, 100); 
    }

    /**
    * Overridable function inherited from base class (MonocularOdom). Reset parameters to default values
    */
    void VOMonoExample::resetParameters(void){;}
    
    
    /**
    * RunAlgorithm. You should call your algorithm routine here. Input_image is the last frame captured by the sensor. 
    * Compute the odometry and send it using the odom_estimated message. If you want to send an image processed by your algorithm, use output_image. Finally, imu and odom may contain useful information, when topics are active in ROS envinroment. 
    *  CHECK IF topics names are CORRECTLY ASSIGNED.
    *  
    *  In this example we use random numbers to generate the position. And the output image is a binary image.
    */
    int VOMonoExample::runAlgorithm(const cv::Mat &input_image, const sensor_msgs::Imu &imu, nav_msgs::Odometry &odom_estimated, cv::Mat &output_image, std::string &imageEncoding){
    
       cv::Mat image;
        
       srand (time(NULL));
	   odom_estimated.pose.pose.position.x =  rand() % 10;
	   odom_estimated.pose.pose.position.y =  rand() % 10;
	   odom_estimated.pose.pose.position.z =  rand() % 10;
	   
       // RGB image to grayscale    
	   cv::cvtColor( input_image, image, CV_RGB2GRAY );
	   // Grayscale image to black and white
       cv::threshold(image, output_image, threshold_, 255, cv::THRESH_BINARY_INV);
       imageEncoding = "mono8";    
    }
    
} // voMonoExample namespace
} // monocularOdom namespace

/**************************** ROS program *****************************/

int main(int argc, char **argv){
    // Intialize ROS
    ros::init(argc,argv,"");

    // Visual odometry algorithm
    monocularOdom::voMonoExample::VOMonoExample vo_algorithm;

    // Monocular VO Node
    visoOdom::MonocularNode node(vo_algorithm);
    
    node.spin();
}