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

    // cv::Mat Depthmap;
    cv::Mat Normalmap;

//-----------------------------------------------------------------------------    
//Do some rendering and animation
//-----------------------------------------------------------------------------    
    // A simple animation loop
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
      sdf::RenderNormal(Tcam, 480, 640, cam_params, 60, 20, 0.02, geometry,Normalmap);
      cv::imshow("Normals", 0.5-Normalmap*0.5);
      char q = cv::waitKey(2);

      //Render Depth image from synthetic environment     
      // sdf::RenderDepth(Tcam, 480, 640, cam_params, 60, 20, 0.02, geometry,Depthmap);      
      // cv::imshow("Depth", Depthmap/8.0);
      // char q = cv::waitKey(2);

    if(q == 'q' || q  == 27 || q  == 71 ) { exit(0); }

    }
    cv::waitKey();
   return 0;
}