#include <uspart_landing/descriptor.h>

namespace landmark{
    /**
    * Empty constructor
    */
    Descriptor::Descriptor(){
        for (int i=0;i<7;++i){
            this->invariant.hu.push_back(0);
        }
    }
    
    /**
    * Empty destructor
    */
    Descriptor::~Descriptor(){;}

    /**
    * Operator= : Method for overlaoding operator assignment
    * @param descriptor operand to be copied
    */
    void Descriptor::operator=(const Descriptor &descriptor){

         // invariant components
        this->invariant.shape = descriptor.invariant.shape ;
        this->invariant.area = descriptor.invariant.area ;
        for (int i=0;i<7;i++)
            this->invariant.hu[i] = descriptor.invariant.hu[i]  ;
        
        // Dynamic components
        this->dynamic.uc     = descriptor.dynamic.uc;
        this->dynamic.vc     = descriptor.dynamic.vc; 
        this->dynamic.theta  = descriptor.dynamic.theta;
        this->dynamic.area   = descriptor.dynamic.area;
        this->dynamic.radius = descriptor.dynamic.radius;  
        
        // Normalize angle
        if (this->dynamic.theta > M_PI){
            this->dynamic.theta -= 2*M_PI;
        } else if (this->dynamic.theta > M_PI) {
            this->dynamic.theta += 2*M_PI; 
        }  
        return;
    }       
    
    
    /**
    * Operator+ : Method for overlaoding operator sum
    * @param descriptor right hand side operand 
    */
    Descriptor Descriptor::operator+(const Descriptor &descriptor){
        Descriptor result;
        
         // invariant components
        result.invariant.shape = this->invariant.shape + descriptor.invariant.shape ;
        result.invariant.area  = this->invariant.area + descriptor.invariant.area;
        for (int i=0;i<7;i++)
            result.invariant.hu[i] = this->invariant.hu[i] + descriptor.invariant.hu[i]  ;
        
        // Dynamic components
        result.dynamic.uc     = this->dynamic.uc + descriptor.dynamic.uc;
        result.dynamic.vc     = this->dynamic.vc + descriptor.dynamic.vc; 
        result.dynamic.theta  = this->dynamic.theta + descriptor.dynamic.theta;
        result.dynamic.area   = this->dynamic.area + descriptor.dynamic.area;
        result.dynamic.radius = this->dynamic.radius + descriptor.dynamic.radius;  
        
        // Normalize angle
        if (result.dynamic.theta > M_PI){
            result.dynamic.theta -= 2*M_PI;
        } else if (result.dynamic.theta > M_PI) {
            result.dynamic.theta += 2*M_PI; 
        }  
        return result;
    }    
    
    /**
    * Operator- : Method for overlaoding operator subtraction
    * @param descriptor right hand side operand 
    */
    Descriptor Descriptor::operator-(const Descriptor &descriptor){
        Descriptor result;
        
         // invariant components
        result.invariant.shape = this->invariant.shape - descriptor.invariant.shape ;
        result.invariant.area = this->invariant.area - descriptor.invariant.area;
        for (int i=0;i<7;i++)
            result.invariant.hu[i] = this->invariant.hu[i] - descriptor.invariant.hu[i] ;
        
        // Dynamic components
        result.dynamic.uc     = this->dynamic.uc - descriptor.dynamic.uc;
        result.dynamic.vc     = this->dynamic.vc - descriptor.dynamic.vc; 
        result.dynamic.theta  = this->dynamic.theta - descriptor.dynamic.theta;
        result.dynamic.area   = this->dynamic.area - descriptor.dynamic.area; 
        result.dynamic.radius = this->dynamic.radius - descriptor.dynamic.radius;  
        
        // Normalize angle
        if (result.dynamic.theta > M_PI){
            result.dynamic.theta -= 2*M_PI;
        } else if (result.dynamic.theta > M_PI) {
            result.dynamic.theta += 2*M_PI; 
        }
        
        return result;
    }   
    
    /**
    * Operator* : Scalar multiplication
    * @param scalar multiply descriptor values by
    */
    Descriptor Descriptor::operator*(const double scalar){
        Descriptor result;
        
        // invariant components
        result.invariant.shape = scalar * this->invariant.shape;
        result.invariant.area  = scalar * this->invariant.area ;
        for (int i=0;i<7;i++)
            result.invariant.hu[i] = scalar * this->invariant.hu[i]  ;
        
        // Dynamic components
        result.dynamic.uc     = scalar * this->dynamic.uc ;
        result.dynamic.vc     = scalar * this->dynamic.vc ; 
        result.dynamic.theta  = scalar * this->dynamic.theta ;
        result.dynamic.area   = scalar * this->dynamic.area ;
        result.dynamic.radius = scalar * this->dynamic.radius ;  
        
        // Normalize angle
        if (result.dynamic.theta > M_PI){
            result.dynamic.theta -= 2*M_PI;
        } else if (result.dynamic.theta > M_PI) {
            result.dynamic.theta += 2*M_PI; 
        }  
        return result;   
    }
    
    /**
    * Maximum values: Return a variable containing the maximum values. Only invariant part matters
    * @param descriptor a variable that contains the values to be compared with
    */
    Descriptor Descriptor::max(const Descriptor &descriptor){
        Descriptor result;
        
         // Check for maximum values
        result.invariant.shape = std::max(descriptor.invariant.shape, this->invariant.shape);
        result.invariant.area = std::max(descriptor.invariant.area, this->invariant.area);
        for (int i=0;i<7;++i){
            result.invariant.hu[i] = std::max(descriptor.invariant.hu[i], this->invariant.hu[i]);
        }
        
        return result;
        
    }

    /**
    * Minimum values: Return a variable containing the minimum values. Only invariant part matters
    * @param descriptor a variable that contains the values to be compared with
    */
    Descriptor Descriptor::min(const Descriptor &descriptor){
        Descriptor result;
        
        // Check for minimum values
        result.invariant.shape = std::min(descriptor.invariant.shape, this->invariant.shape);
        result.invariant.area  = std::min(descriptor.invariant.area, this->invariant.area);
        for (int i=0;i<7;++i){
            result.invariant.hu[i] = std::min( descriptor.invariant.hu[i], this->invariant.hu[i]) ;
        } 

        return result;
    }
    
    
    /**
    * Assemble descriptor. Given a vector of contour points (1st input), computes the descriptor parameters (output). For better performance, when necessary, compute the DYNAMIC components after the target has been detected.
    *    @param contours the edges points of a blob. 
    *    @param option indicates which descriptor component shall be computed (invariant and/or dynamic). 
    */
    void Descriptor::assemble(std::vector<cv::Point> &contours, int option = (INVARIANT | DYNAMIC) ){    
        double hu[7];
        
        // Compute regular, centra, normalized moments, centroid and invariante moments (Hu).
        cv::Moments moment = cv::moments( contours, true );
        cv::Point2f centroid = cv::Point2f( moment.m10/moment.m00 , moment.m01/moment.m00 );
        HuMoments(moment, hu);                    
    
        // Compute image inertia moment matrix, its eigenvalues and eigenvectors.
        // ps: eigenvector stores in descending order in a column vector. Eigenvector are stored in rows, same order as their respective eigenvalues.
        cv::Mat eigenvalues, eigenvectors, J;
        J = (cv::Mat_<double>(2,2)<<moment.mu20, moment.mu11, moment.mu11, moment.mu02 );
        cv::eigen(J,eigenvalues, eigenvectors);
               
        // Descriptor invariant components
        if (option & INVARIANT){
            this->invariant.shape = eigenvalues.at<double>(0,1)/eigenvalues.at<double>(0,0);
            this->invariant.area  = 4*M_PI*std::sqrt(moment.nu02*moment.nu20 - moment.nu11*moment.nu11);
            for (int i=0;i<7;++i)
                this->invariant.hu[i] = hu[i];         
        }

        // Descriptor dynamic components        
        if (option & DYNAMIC){
            cv::Point2f circleCenter;
            float radius;
            
            cv::minEnclosingCircle( contours, circleCenter, radius );
            
            this->dynamic.uc = circleCenter.x; //ellipse.center.x ;//circleCenter.x;
            this->dynamic.vc = circleCenter.y; //ellipse.center.y  ;//circleCenter.y;
            this->dynamic.area = moment.m00;
            this->dynamic.theta = std::atan2( eigenvectors.at<double>(0,1) ,   eigenvectors.at<double>(0,0) )  ;
            
            // Find furthest point from the circle center that belongs to the contour.
            double max_dist = 0;
            for (int i=0; i<contours.size();++i){
                double dist = pow(contours[i].x - circleCenter.x,2) + pow(contours[i].y - circleCenter.y,2);
                if (dist > max_dist) {
                    max_dist = dist;
                }
            }
            this->dynamic.radius = std::sqrt(max_dist);   
    
            // The following code lines are for fixing the angle. It is based in the relative position between the confined circle center and the landmark mass centroid. It only makes sensse if both are not in the same place.            
            if ( std::abs(centroid.y - circleCenter.y) > std::abs(centroid.x - circleCenter.x) ){  // Accuracy along the v-axis is higher       
                if ( (centroid.y - circleCenter.y) > 0){    // expects positive angle
                    if (this->dynamic.theta < 0)
                        this->dynamic.theta += M_PI;
                } else {                                    // expects negative angle
                     if (this->dynamic.theta > 0)
                        this->dynamic.theta -= M_PI;
                }
  
            } else{             // Accuracy along the u-axis is higher
                if ( (centroid.x - circleCenter.x) > 0){    // expects angle < 90o.
                     if (std::abs(this->dynamic.theta) > M_PI/2)
                        this->dynamic.theta -= sign(this->dynamic.theta)*M_PI;
                } else {                                    // expects angle > 90o.
                     if (std::abs(this->dynamic.theta) < M_PI/2)
                         this->dynamic.theta -= sign(this->dynamic.theta)*M_PI;
                }                          
            }
            

            
        }
    }
    
    
     /**
    * Computes the similarity of two descriptors. First, the function computes the distance between the current descriptor handle and the one specified by 'base'. Then, the overall similarity score is computed based on the values specified by 'range'. If a given characteristic is within the interval defined by range, then the score is incremented. The higher the score (returning values passed as parameters), higher the similarity.   
    * @param base a descriptor containing the reference values
    * @param distLimits a descriptor containing the maximum distance for similiraty measurements
    * @param invariant_score the computed score for the invariant part
    * @param dynamic_score the compute score for the dynamic part
    *
    */      
    void Descriptor::similarity(Descriptor &base, Descriptor &distLimits, int &invariant_score, double &dynamic_score ){
        invariant_score = 0;
        dynamic_score = 0;
        
        // Compute distance between current descriptor and base descriptor
        Descriptor distance = (*this) - base;

        // Compare and score according to threshold defined in limits
        // Shape
        if ( checkDistance(distance.invariant.shape, distLimits.invariant.shape)  )
            invariant_score++; 
        
        // Area
        if ( checkDistance(distance.invariant.area, distLimits.invariant.area)  ){
            invariant_score++; 
        }
        
        // Hu moments
        for (int i=0;i<7;i++) {
            if ( checkDistance(distance.invariant.hu[i], distLimits.invariant.hu[i])  ){
                invariant_score++;
            }
        }
        
        // Dynamic score (More weight is given to position in respect to the last (uc,vc) observed)
        dynamic_score = 1.0/( 0.5*(std::sqrt(pow(distance.dynamic.uc ,2) +  pow(distance.dynamic.vc ,2))) + 0.01) +
                        1.0/(0.1*std::abs(distance.dynamic.area) + 0.01 ) +
                        1.0/(0.1*std::abs(distance.dynamic.radius) + 0.01 );

        
        
    }
    
    /**
    * Print descriptor specified by current handle to standart output (screen).
    */
    void Descriptor::printFormatted(void){
        // invariant
        std::cout << "invariant characteristics: "  << std::endl; 
        std::cout << "shape: " << this->invariant.shape << std::endl;
        std::cout << "area: " << this->invariant.area << std::endl;
        std::cout << "hu moments: [" ;
        for (int i=0;i<7;++i){
            std::cout << this->invariant.hu[i] << " " ;
        }  
        std::cout << "]" << std::endl;
        
        // Dynamic
        std::cout << "Dynamic characterists: " << std::endl; 
        std::cout << "(uc,vc): (" << this->dynamic.uc << " , " << this->dynamic.vc << ")" << std::endl;
        std::cout << "theta (deg): " << this->dynamic.theta*180/M_PI << std::endl;
        std::cout << "area (pixels): " << this->dynamic.area << std::endl;
        std::cout << "radius: " << this->dynamic.radius << std::endl;   
    }

    /**
    * Print descriptor specified by current handle in a single line. Good for data analysis.
    */    
    void  Descriptor::printNonFormatted(void){
        // invariant
        std::cout << this->invariant.shape << " ";
        std::cout << this->invariant.area << " ";
        for (int i=0;i<7;++i){
            std::cout << this->invariant.hu[i] << " ";
        }  
        
        // Dynamic
        std::cout << this->dynamic.uc << " " << this->dynamic.vc<< " ";
        std::cout << this->dynamic.theta<< " ";
        std::cout << this->dynamic.area<< " ";
        std::cout << this->dynamic.radius<< " ";

        std::cout << std::endl;
    }
    
    /**
    * Sign. Check if the variable 'n' is positive or nergative. The function returns +1 or -1. Zero is considered as a positive number.
    * @param n a number
    */
    int sign(double n){
        if (n<0){
            return -1;
        }else
            return 1;
    }
    
    /**
    * Check distance. Returns true if input variable 'n' has absolute numerical value smaller than 'thresh' value. Fasle, otherwise
    * @param n a number.
    * @param thresh a threshold value to evaluate n
    */
    bool checkDistance(double &n, double &thresh){
        if ( (n < thresh) && (n > -thresh) )        
            return true;
        else
            return false;
    }
} // landmark namespace
