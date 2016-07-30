#include <uspart_landing/landmark.h>

namespace landmark{
    /**
    * Empty Constructor
    */
    Landmark::Landmark(){
        this->learning_ = false;
        this->lock      = false;
        this->min_score = 7;
    };
    
    /**
    * Empty Destructor
    */
    Landmark::~Landmark(){};
    
    /**
    *  Set private member base (descriptor).
    * @param base a reference value.
    */
    void Landmark::setBase(Descriptor &base){
        this->base_ = base;
    }
    
    /**
    *  Set private member range (descriptor).
    * @param range a reference value.
    */
    void Landmark::setRange(Descriptor &range){
        this->range_ = range;

    }
 
    /**
    * Print base descriptor using formatted style
    */
    void Landmark::getBase(void){
        this->base_.printFormatted();
    }
    
    /**
    * Print range descriptor using formatted style
    */
    void Landmark::getRange(void){
        this->range_.printFormatted();
    }        
    
    /**
    * Detect landmark: Extract a singular landmark from a contour vector. Returns the contour index of the landmark detected
    * @param an image to extract blobs from.
    * @param contours extracted from image frame
    * @param descriptor corresponding landmark descriptor (returning value).
    */
     int Landmark::detect(std::vector<std::vector<cv::Point> > &contours, Descriptor &descriptor){
        // Allocate memory for computing descriptors and similarity for each contour
        std::vector<Descriptor> candidate(contours.size());
        std::vector<int>        similarity(contours.size());
        // Miscellaneous variables and flags
        int index=-1;
        double max_score = 0;
        bool landmark_detected  = false;

        
        // Search for landmark!         
        for( int i=0; i<contours.size(); i++){
           if (contours[i].size() < 5)
                continue;
            
            int inv_score=0;
            double dyn_score=0;
            
            // Assemble dynamic descriptor 
            candidate[i].assemble(contours[i], (INVARIANT | DYNAMIC) );
            if (candidate[i].dynamic.area < 20)  // discard small blob (noise)
                 continue;
            // Compute smilarity in respect to base descriptor
            candidate[i].similarity(this->base_, this->range_,  inv_score, dyn_score );  
            
            // Have we already detected and locked on the landmark?
            if( this->lock == false ){ // No! Do not account for the dynamic score, conservative in repesct to invariant characteristics
                if( (inv_score > this->min_score) && (inv_score > max_score) ){
                    max_score = inv_score;
                    index     = i;
                    landmark_detected = true;    
                }
            }else{                    // Yes! Account for dynamic score (tracking component), liberal in respect to invariant characteristics
                if ( (inv_score > (this->min_score-3)) && (dyn_score > max_score) ){
                    max_score = dyn_score;
                    index     = i;
                    landmark_detected = true;  
                }
            }
 
        }
         
         // Save descriptor when landmark detected
         if (landmark_detected){
             descriptor = candidate[index];
             this->base_.dynamic.uc = candidate[index].dynamic.uc;
             this->base_.dynamic.vc = candidate[index].dynamic.vc; 
             this->base_.dynamic.radius = candidate[index].dynamic.radius;
             this->base_.dynamic.area   = candidate[index].dynamic.area;     
         } 
         return index;
         
         
     }
    
    
    /**
    * learn landmark: Learn a singular landmark descriptor from an image. In the very first image, the desired landmark must be the largest blob in the scene. Returns true when succeed, otherwise false.
    * @param image an image to extract blobs from
    * @param offset an offset in respect to original image. If not using ROI, set to (0,0).
    * @param descriptor current landmark descriptor (returning value).
    * @param blob_contours contours of the detected landmark (returning value).
    */
    bool Landmark::learnMinMax(const cv::Mat image, const cv::Point offset, Descriptor &descriptor, std::vector<cv::Point> blob_contours){
        // Number of learning iterations
        static Descriptor min, max;

        // Detect edges using canny
        cv::Mat mat =image.clone();

        // Find contours
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE , offset );
      
        // Allocate memory for computing descriptors and similarity for each contour
        std::vector<Descriptor> candidate(contours.size());
        std::vector<int>        similarity(contours.size());
        // Miscellaneous variables and flags
        int index;
        double score, max_score = 0;
        bool landmark_detected  = false;
        
        // Search for landmark!
        for( int i=0; i<contours.size(); i++){
            if (contours[i].size() < 5)
                continue;
            
            int inv_score=0;
            double dyn_score=0;
            // Assemble dynamic descriptor (we just want the area)
            candidate[i].assemble(contours[i], (INVARIANT | DYNAMIC) );
            candidate[i].similarity(this->base_, this->range_,  inv_score, dyn_score );    
            // Is this the first learning iteration?
            if(this->learning_ == false) // Yes! Then we are searching for the largest blob in the scene (score == area).
                score = candidate[i].dynamic.area;
            else                         // No! Then, we are searching for the most similar blob from last time observed.
                score = dyn_score;
            
            // Is this the most similar descriptor so far?
            if( score > max_score ){        // Yes! Then save the current index
                max_score = score;
                index = i;
                landmark_detected = true;
            }else if (score == max_score){  // The same as the hightest. We can not say anything..
                landmark_detected = false;
            }  
        }
        
        // Did we find the landmark?
        if (landmark_detected) {  // Yes! Then update the min, max, base and range descriptors.
            // First time detecting the landmark?
            if(this->learning_ == false){ // Yes! Start learing
                max = candidate[index];
                min = candidate[index];
                this->base_ = candidate[index];
                this->learning_ = true;     
            } else{                // No! Update base and range values
                max = max.max(candidate[index]);
                min = min.min(candidate[index]);
            
                this->base_   = max*0.5 + min*0.5;
                this->range_  = this->base_ - min;
            }                    
    
            // Saving current dynamic values
            this->base_.dynamic.uc = candidate[index].dynamic.uc;
            this->base_.dynamic.vc = candidate[index].dynamic.vc; 
            this->base_.dynamic.radius = candidate[index].dynamic.radius;
            this->base_.dynamic.area   = candidate[index].dynamic.area;
            
            descriptor = this->base_;   
        }
        
        return landmark_detected;

    }
    
} // closing landmark namespace