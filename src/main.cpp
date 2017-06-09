#include "libsdf.h"

int main( int argc, char* argv[] )
{

//-----------------------------------------------------------------------------    
// Define geometry of the world
//-------------------------------------------------------------------------
    std::vector<sdf::Primitive*> geometry;
    Eigen::Vector4d geometryParam;

    // Create the room
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = Eigen::Vector3d(0.0, 0.0, -1.0);
    geometryParam << 4.0, 4.0, 8.0, 0.0; //dimensions in X Y Z
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , true)); //the last flag flips the sign of the box

    // Add a box inside
    T = Eigen::Affine3d::Identity();
    geometryParam << 1.0, 2.0, 0.5, 0.0; //dimensions in X Y Z
    T.translation() = Eigen::Vector3d(-1.0, 1.0, 2.2);
    T.linear() = (  Eigen::AngleAxisd(-M_PI*30/180, Eigen::Vector3d::UnitY())).toRotationMatrix(); 
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
        
    //Add a Sphere
    T = Eigen::Affine3d::Identity();
    T.translation() = Eigen::Vector3d(1.0, 1.5, 2.0);
    geometryParam << 0.5, 0.0, 0.0, 0.0; // Radius
    geometry.push_back(new sdf::Sphere( sdf::bool_union, T, geometryParam , false));
    T = Eigen::Affine3d::Identity();
   
    //Add another Box
    geometryParam << 0.6, 0.6, 0.6, 0.0; //dimensions in X Y Z
    T.translation() = Eigen::Vector3d(0.5, 2-0.6, 0.5);
    T.linear() = (  Eigen::AngleAxisd(M_PI*10/180, Eigen::Vector3d::UnitY())).toRotationMatrix(); 
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
    T = Eigen::Affine3d::Identity();

    //Add a torus
    T.translation() = Eigen::Vector3d(0.5, 0, 2.25);
    geometryParam << 0.50, 0.1, 0.0, 0.0; // radius of ring, radius of cross-section
    geometry.push_back(new sdf::Torus( sdf::bool_union, T, geometryParam , false));
//-----------------------------------------------------------------------------    
//Define a camera to use as sensor
//-----------------------------------------------------------------------------    
    Eigen::Affine3d Tcam = Eigen::Affine3d::Identity();

    float fx = 300;
    float fy = 300;
    float cx = 319.5;
    float cy = 239.5;

    // focalx,focaly,centerx,centery (image size is determined by Render() fcn )
    Eigen::Vector4f cam_params(fx, fy, cx, cy);

    constexpr int im_width = 640;
    constexpr int im_height = 480;

    CImg<float> Depthmap(im_width,im_height,1,1); 
    CImg<float> Normalmap(im_width,im_height,1,3); 

//-----------------------------------------------------------------------------    
//Do some rendering and animation
//-----------------------------------------------------------------------------    
    // A simple animation loop
    CImgDisplay depth_disp(Depthmap,"Depth image"), normal_disp(Normalmap, "Normal Map");

  


    for (int i = -150; i < 60; ++i)
    {
    
      //time steps      
      double t = double(i);

      // move the camera in an elliptic 3D spiral with pure translation (X Y Z)
      Tcam.translation() = Eigen::Vector3d( 
          cos(4*M_PI*t/180), 
          0.6*sin(4*M_PI*t/180), 
          t*0.01);

      //change the rotation for one of the objects at each iteration
      T.linear() = ( 
                    //Eigen::AngleAxisd(i*M_PI/50, Eigen::Vector3d::UnitZ()) *   
                    //Eigen::AngleAxisd(i*M_PI/50, Eigen::Vector3d::UnitY()) * 
                    Eigen::AngleAxisd(i*M_PI/50, Eigen::Vector3d::UnitX()) 
                    ).toRotationMatrix(); //rotation about X 
      geometry[4]->transformation = T;
      
      //Render Normal-map from synthetic environment 
      sdf::RenderNormal(Tcam, im_height, im_width, cam_params, 32, 10, 0.05, geometry,Normalmap);
      Normalmap.display(normal_disp);

      //Render Depth image from synthetic environment     
      sdf::RenderDepth(Tcam, im_height, im_width, cam_params, 32, 10, 0.4, 0.05, geometry,Depthmap);      
      Depthmap.display(depth_disp);
      
      if(normal_disp.is_closed() || depth_disp.is_closed()) exit(0);

    }
    normal_disp.wait();
    return 0;
}