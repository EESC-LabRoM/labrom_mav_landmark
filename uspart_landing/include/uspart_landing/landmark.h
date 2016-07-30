#ifndef LANDMARK_H
#define LANDMARK_H

// Landmark libraries
#include <uspart_landing/descriptor.h> 

// OpenCV libraries
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// C++ libraries

namespace landmark{
    class Landmark{
         private:
            Descriptor base_;          //!< Reference values for landmark descriptor. 
            Descriptor range_;         //!< Reference values for for similarity measurements.   
            bool learning_;            //!< Flag that indicates learning process is active.
         public:
            bool lock;                  //!< Flah that indicates that landmark was found
            int min_score;             //!< Minimum score required to be valid
        
            //! Empty constructor.
            Landmark();
            //! Empty destructor.
            ~Landmark();
        
            //! Set base descriptor.
            void setBase(Descriptor &base);

            //! Set range descriptor.
            void setRange(Descriptor &range);
        
            //! Print base descriptor
            void getBase(void);
        
            //! Print range descriptor
            void getRange(void);

            //! Detects landmark from contours points.
            int detect(std::vector<std::vector<cv::Point> > &contours, Descriptor &descriptor);
        
            //! Learn landmark detection from image.
            bool learnMinMax(const cv::Mat image, const cv::Point offset, Descriptor &descriptor, std::vector<cv::Point> blob_contours);
        
         
    }; // closing class Landmark
    
} // closing landmark namespace

#endif