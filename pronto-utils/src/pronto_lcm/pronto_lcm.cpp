#include <iostream>
#include <zlib.h>

#include <lcm/lcm.h>

#include "pronto_lcm.hpp"
#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;

pronto_lcm::pronto_lcm (lcm_t* publish_lcm):
        publish_lcm_(publish_lcm){
}


// Copied from kinect-lcm
static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
    double result[3])
{
  result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
  result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
  result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}
