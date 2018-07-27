#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/path/path.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs::constants;

std::vector< std::shared_ptr< path::PathSegment > > createMonSegments( const path::PathBounds4d& path_bounds) 
{
  using JoiningSegment_t  = path::CartesianSegment< path::LinearSegment, path::SmoothStep7 >;
  using BlendSegment_t    = path::CartesianSegment< path::CircularBlend, path::SmoothStep7 >;
  
  std::shared_ptr< traxxs::path::PathSegment > seg1, blend1,blend2,blend3, seg2, seg3,seg4, blend_inter;
  
  traxxs::path::CartesianPathWaypoint  wpt_start, wpt_1, wpt_2,wpt_3, wpt_end, wpt_init, wpt_inter;
  double maxDeviation = 0.001;
  
  wpt_init.x.p <<  0.38969, -0.0184061, 0.652811;
  wpt_init.x.q = Eigen::Quaterniond( 1, 0, 0, 0 );
  wpt_init.pathConditionsPosition.dx << 0, 0, 0;
  wpt_init.pathConditionsOrientation.dx << 0;
  wpt_start.x.p <<  0.435,-0.14282015,0.55435536;
  wpt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
  
  wpt_1.x.p << 0.435, -0.10751101, 0.5518846 ;
  wpt_1.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 


  wpt_2.x.p << 0.435, 0.0,  0.56 ;
  wpt_2.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 );  
  

   wpt_3.x.p << 0.435, 0.10751101,  0.5518846 ;
   wpt_3.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 );  
  
  wpt_end.x.p << 0.435, 0.14285015,  0.55435536 ;
  wpt_end.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 );  
  wpt_end.pathConditionsPosition.dx << 0, 0, 0;
  wpt_end.pathConditionsOrientation.dx << 0;
  
//   wpt_inter.x.p <<  0.40,-0.16282015,0.57735536;
//   wpt_inter.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 );  
  

   path::PathBounds4d path_bounds_blend;
   path_bounds_blend.dx << 0.4, 0.4, 0.4, 1.0;
  path_bounds_blend.ddx = 100.0 * path_bounds.dx;
  path_bounds_blend.j = 100.0 * path_bounds.ddx;
  
  traxxs::path::CartesianPathWaypoint blend_1_start, blend_1_end, blend_1_wpt,blend_2_start, blend_2_end, blend_2_wpt,blend_3_start, blend_3_end, blend_3_wpt ;
  traxxs::path::CartesianPathWaypoint blend_inter_start, blend_inter_end, blend_inter_wpt;
  double maxDeviation_1 = 1e-3;
  double maxDeviation_2 = 1e-3;
  // Initialiser les position des blends
  blend_1_start = wpt_start;
  blend_1_end = wpt_2;
  blend_1_wpt = wpt_1;
  blendStartEndFromWaypoints( blend_1_start, blend_1_end, blend_1_wpt );
  blend1 = path::createBlendSegmentPositionOnly< BlendSegment_t, double >( path_bounds_blend, blend_1_start, blend_1_end, blend_1_wpt, maxDeviation_1 );
  blend1->init();
  
  blend_2_start =wpt_1;
  blend_2_end = wpt_3;
  blend_2_wpt = wpt_2; 
  blendStartEndFromWaypoints( blend_2_start, blend_2_end, blend_2_wpt );
  blend2 =path:: createBlendSegmentPositionOnly< BlendSegment_t, double >( path_bounds_blend, blend_2_start, blend_2_end, blend_2_wpt, maxDeviation_2 );
  blend2->init();
  
  blend_3_start = wpt_2	;
  blend_3_end = wpt_end;
  blend_3_wpt = wpt_3;
  blendStartEndFromWaypoints( blend_3_start, blend_3_end, blend_3_wpt );
  blend3 = path::createBlendSegmentPositionOnly< BlendSegment_t, double >( path_bounds_blend, blend_3_start, blend_3_end, blend_3_wpt, maxDeviation_1 );
  blend3->init();
  
//   blend_inter_start = wpt_init; 
//   blend_inter_end = wpt_start;
//   blend_inter_wpt = wpt_inter;
//   blendStartEndFromWaypoints( blend_inter_start, blend_inter_end, blend_inter_wpt );
//   blend_inter = path::createBlendSegmentPositionOnly< BlendSegment_t, double >( path_bounds_blend, blend_inter_start, blend_inter_end, blend_inter_wpt, 0.01 );
//   blend_inter->init();
  std::vector< std::shared_ptr < path::PathSegment > > blends = { blend1, blend2,blend3};
  std::vector< std::shared_ptr < path::PathSegment > > segments ;
  std::shared_ptr< path::PathSegment > my_segment = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_init, wpt_start, path_bounds );
//   std::shared_ptr< path::PathSegment > my_segment_blend = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_init, wpt_start, path_bounds );
  
  traxxs::path::joinBlendSegments< traxxs::path::CartesianPathWaypoint, JoiningSegment_t >( path_bounds, wpt_start, wpt_end, blends, segments );
  segments.insert(segments.begin(),my_segment);
  
//   std::shared_ptr< path::PathSegment > my_segment_2 = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_end, wpt_start, path_bounds );
//   segments.push_back(my_segment_2);
  return segments;
}


std::vector< std::shared_ptr< path::PathSegment > > createNFourth( const path::PathBounds4d& path_bounds) 
{
      path::CartesianPathWaypoint wpt_start, wpt_end;
    
    wpt_start.x.p << 0.5,-0.2,0.4 ;
    wpt_end.x.p << 0.5,0.2,0.4 ;

    wpt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
    wpt_end.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
    wpt_start.pathConditionsPosition.dx << 0, 0, 0;
    wpt_start.pathConditionsOrientation.dx << 0;
    wpt_end.pathConditionsPosition.dx << 0, 0, 0;
    wpt_end.pathConditionsOrientation.dx << 0;
    std::shared_ptr< path::PathSegment > my_segment_1 = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_start, wpt_end, path_bounds );
    my_segment_1->init();
    std::shared_ptr< path::PathSegment > my_segment_2 = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_end, wpt_start, path_bounds );
    my_segment_2->init();
   
     std::vector< std::shared_ptr < path::PathSegment > > segments;

    int compteur;
    compteur = 1;
      do{
      segments.push_back( my_segment_1 );
      segments.push_back( my_segment_2);
        compteur += 1;
      }while(compteur < 5);
      
      return segments;
}


std::vector< std::shared_ptr< path::PathSegment > >montest( const path::PathBounds4d& path_bounds) 
{
 	
    path::CartesianPathWaypoint wpt_start, wpt_end, wpt_start_temp;
    
    wpt_start.x.p << 0.,0.3,0.4 ;
//     wpt_end.x.p << -0.3,0.2,0.4 ; // Utiliser pour simuler la contrainte des articulations
//      wpt_end.x.p <<0.1,0.4,0.3 ;
    wpt_end.x.p << 0.4, 0.3, 0.3; 
    wpt_start_temp.x.p << 0.38969, -0.0184061, 0.652811;    
//     wpt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
//     wpt_end.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
    wpt_start_temp.pathConditionsPosition.dx << 0,0,0;
    wpt_start.pathConditionsPosition.dx << 0, 0, 0;
    wpt_start.pathConditionsOrientation.dx << 0;
    wpt_end.pathConditionsPosition.dx << 0, 0, 0;
    wpt_end.pathConditionsOrientation.dx << 0;
    std::shared_ptr< path::PathSegment > init_segment = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> (wpt_start_temp,wpt_end,path_bounds);
    init_segment -> init();
    std::shared_ptr< path::PathSegment > my_segment_1 = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_start, wpt_end, path_bounds );
    my_segment_1->init();
      std::vector< std::shared_ptr < path::PathSegment > > segments;
    segments.push_back( init_segment);
//     segments.push_back(my_segment_1);
    return   segments;
  }