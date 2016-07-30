#include <uspart_viso_odom/monocular_odom.h>


namespace monocularOdom{
    /**
    * Constructor (empty).
    */
    MonocularOdom::MonocularOdom() : camera_calibrated_(false){;}
    
    /**
    * Destructor (empty).
    */
    MonocularOdom::~MonocularOdom(){;}
    
    /**
    * Camera calibration function handle. It loads the camera calibration parameters into variables defined within the base class. It assumes plumb distortion model (k1,k2,k3 - radial and p1,p2 - tangential)
    * 
    * Virtual class. It is overriddable, but you dont have to if all you need is calibration. 
    */
    void MonocularOdom::cameraCalibration(const sensor_msgs::CameraInfoConstPtr &camera_info){
        // Camera intrisic parameters
        fx_ =  camera_info->K[0];
        fy_ =  camera_info->K[4];
        cx_ =  camera_info->K[2];
        cy_ =  camera_info->K[5];      
        
        // Plumb distortion model parameters
        k1_ = camera_info->D[0];
        k2_ = camera_info->D[1];
        p1_ = camera_info->D[2];
        p2_ = camera_info->D[3];
        k3_ = camera_info->D[4];
        
        // Image resolution
        width_ = camera_info->width;
        height_ = camera_info->height;

        // Camera calibration flag
        camera_calibrated_ = true;
    }
    
}