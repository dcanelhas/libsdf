#include "libsdf.h"
#include <iostream>

namespace sdf
{
  Eigen::Vector3d 
  makeRay(int row, int column, float depth, float fx, float fy, float cx, float cy)
  {

    Eigen::Vector3d ret(double(column-cx)*depth/(fx),
                        double(row-cy)*depth/(fy),
                        double(depth));
    return ret;
  };

  double 
  SDF(const Eigen::Vector3d &location, const std::vector<Primitive*> &primitives)
  {
    double d_value = 0.0;

    for(std::vector<Primitive*>::const_iterator it = primitives.begin(); it != primitives.end(); ++it)
    {
      Eigen::Vector3d location_warped = it[0]->transformation.inverse()*location;
      if( it == primitives.begin() ) d_value = it[0]->signedDistance(location_warped);
      else
      {
        switch(it[0]->operation)
        {
          case bool_union :
            d_value = std::min( it[0]->signedDistance(location_warped), d_value);
          
            break;
          case bool_intersection :
            d_value = std::max( it[0]->signedDistance(location_warped), d_value);
            break;
          
          case bool_subtraction :
            d_value = std::max( -it[0]->signedDistance(location_warped), d_value);
            break;
        } 
      }
    }
    return d_value;
  }
 
  Eigen::Vector3d 
  SDFGradientPosNorm(const Eigen::Vector3d &location,  std::vector<Primitive*> &primitives)
  {
    double delta=1e-05;
    Eigen::Vector3d v = Eigen::Vector3d(
           fabs(SDF(location + Eigen::Vector3d(delta,0,0), primitives) - SDF(location - Eigen::Vector3d(delta,0,0), primitives)),
           fabs(SDF(location + Eigen::Vector3d(0,delta,0), primitives) - SDF(location - Eigen::Vector3d(0,delta,0), primitives)),
           fabs(SDF(location + Eigen::Vector3d(0,0,delta), primitives) - SDF(location - Eigen::Vector3d(0,0,delta), primitives))/(2.0*delta));
    return v.normalized();
  } 
 
  Eigen::Vector3d 
  SDFGradient(const Eigen::Vector3d &location,  std::vector<Primitive*> &primitives)
  {
    double delta=1e-05;
    return Eigen::Vector3d(
           SDF(location + Eigen::Vector3d(delta,0,0), primitives) - SDF(location - Eigen::Vector3d(delta,0,0), primitives),
           SDF(location + Eigen::Vector3d(0,delta,0), primitives) - SDF(location - Eigen::Vector3d(0,delta,0), primitives),
           SDF(location + Eigen::Vector3d(0,0,delta), primitives) - SDF(location - Eigen::Vector3d(0,0,delta), primitives))/(2.0*delta);
  }

  void RenderNormal(
    const Eigen::Affine3d &transformation, 
    const int rows, 
    const int cols, 
    Eigen::Vector4f &cam_params, 
    const int max_steps, 
    const float max_ray_length, 
    const float precision, 
    std::vector<Primitive*> &primitives,
    CImg<float> &Normals)
  {
    
    const Eigen::Vector3d camera = transformation.translation();
    const Eigen::Vector3d viewAxis = (transformation * Eigen::Vector3d(0.0,0.0,1.0+1e-12) - camera).normalized(); 
    
    //Rendering loop
   #pragma omp parallel for collapse(2)
    for(int u = 0; u < rows; ++u)
    {
      for(int v = 0; v < cols; ++v)
      {
        bool hit = false;

        Eigen::Vector3d p = transformation*makeRay(u, v, 1.0, cam_params(0), cam_params(1), cam_params(2), cam_params(3)) - camera;
        p.normalize();
              
        double scaling = 0.4;
        double scaling_prev=0;
        double D = 1.0;
        int steps=0;
        while(steps<max_steps && scaling < max_ray_length && !hit)
        { 

          double D_prev = D;
          D = SDF(camera + p*scaling, primitives);
          if(D < precision)
          {
            scaling = scaling_prev + (scaling-scaling_prev)*D_prev/(D_prev - (D - precision));

            hit = true;
            Eigen::Vector3d normal_vector = SDFGradient(camera + p*scaling,primitives);            
            normal_vector.normalize();

            Normals(v,u,0,0)=normal_vector(0);
            Normals(v,u,0,1)=normal_vector(1);
            Normals(v,u,0,2)=normal_vector(2);

            break;
          }
          scaling_prev = scaling;
          scaling += D;  
          ++steps;        
        }//ray
        if(!hit)     
        {
          Normals(v,u,0,0)=std::numeric_limits<float>::quiet_NaN();
          Normals(v,u,0,1)=std::numeric_limits<float>::quiet_NaN();
          Normals(v,u,0,2)=std::numeric_limits<float>::quiet_NaN();
        
        }//no hit
      }//col
    }//row 

    return;
  }

  void RenderDepth(
    const Eigen::Affine3d &transformation, 
    const int rows, 
    const int cols, 
    Eigen::Vector4f &cam_params, 
    const int max_steps, 
    const float max_ray_length,
    const float min_ray_length,
    const float precision, 
    std::vector<Primitive*> &primitives, 
    CImg<float> &Depth)
  {

  // add noise to depth
  // std::mt19937_64 generator;
  // std::normal_distribution<float> distribution(0,RENDER_NOISE_STD);

  // cv::Mat Z_img( rows, cols, CV_32FC1);

   Eigen::Vector3d camera = transformation * Eigen::Vector3d(0.0,0.0,0.0);
   Eigen::Vector3d viewAxis = (transformation * Eigen::Vector3d(0.0,0.0,1.0+1e-12) - camera).normalized();

  //Rendering loop
  #pragma omp parallel for collapse(2)
  for(int u = 0; u < rows; ++u)
  {
    for(int v = 0; v < cols; ++v)
    {
      bool hit = false;
      Eigen::Vector3d p = transformation*makeRay(u, v, 1.0, cam_params(0), cam_params(1), cam_params(2), cam_params(3)) - camera;


      p.normalize();
            
      double scaling = min_ray_length;
      double scaling_prev=0;
      double D = 1.0;
      int steps=0;
      while(steps<max_steps && scaling < max_ray_length && !hit)
      { 

        double D_prev = D;
        D = sdf::SDF(camera + p*scaling, primitives);
        if(D < precision)
        {
          if(scaling_prev < min_ray_length)
          {
              Depth(v,u)=std::numeric_limits<float>::quiet_NaN();    
              break;
          }
          scaling = scaling_prev + ((scaling+precision) - scaling_prev)*D_prev/(D_prev - (D - precision));

          hit = true;
          float d = scaling*(viewAxis.dot(p));
          
          //conversion from range to depth
          // + scaling*scaling*distribution(generator)); //add noise dependent on the square of the distance
          Depth(v,u)=fabs(d);

          break;
        }
        scaling_prev = scaling;
        scaling += D;  
        ++steps;        
      }//ray
      if(!hit)     
      {
        Depth(v,u)=std::numeric_limits<float>::quiet_NaN();    
      }//no hit
    }//NUM_COLS
  }//NUM_ROWS
  return;
  }
}//sdf
