#ifndef H_DESCRIPTOR_H
#define H_DESCRIPTOR_H

// Opencv Libraries
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// Math libraries
#include <cmath>
#include <math.h>
#include <iostream>

// CPP libraries
#include <vector>

namespace landmark{
    #define INVARIANT 0x01
    #define DYNAMIC 0x02
    
    class Descriptor{
        public:
            struct Invariant{
                double shape;
                double area;
                std::vector<double> hu;
            } invariant;               //! Invariant part of the descriptor
            
            struct Dynamic{
                double uc,vc;
                double theta;
                double area;
                double radius;
            } dynamic;                  //! Dynamic part of the descriptor
            
            //! Constructor
            Descriptor();
            //! Destructor
            ~Descriptor();
        
             //! Overload operator =
            void operator=(const Descriptor &descriptor); 
        
            //! Overload operator +
            Descriptor operator+(const Descriptor &descriptor);
    
            //! Overload operator -
            Descriptor operator-(const Descriptor &descriptor);
            
            //! Overload operator * 
            Descriptor operator*(const double scalar);
                
            //! Max values of two descriptors
            Descriptor max(const Descriptor &descriptor);
        
            //! Min values of two descriptors
            Descriptor min(const Descriptor &descriptor);
        
            //! Assemble descriptor
            void assemble(std::vector<cv::Point> &contours, int option);
        
            //! Similarity evaluation
            void similarity(Descriptor &base, Descriptor &distLimits, int &invariant_score, double &dynamic_score);

            //! Print human readble descriptor to screen
            void printFormatted(void);
        
            //! Print descriptor as a vector
            void printNonFormatted(void);
     
        
    };
     
    //! Computes the sign
    int sign(double n);
    
    //! Check if ||n|| < thresh
    bool checkDistance(double &n, double &thresh);
    
} // landmark namespace


#endif