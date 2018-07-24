// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/

#include <orca/orca.h>
#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>
#include <matplotlibcpp/matplotlibcpp.h>
#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/tracker/tracker.hpp>
#include <traxxs/impl/traxxs_softmotion/traxxs_softmotion.hpp>
using namespace orca::all;
using namespace orca::gazebo;
using namespace std;
using namespace traxxs;


#include "orca_gazebo_pu_reg.hpp"
#include "orca_pu_gazebo_ctrs.hpp"
#include "orca_pu_fonction.hpp"
#include "mon_segments.hpp"
int main(int argc, char const *argv[])
{
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf (optionally -l debug/info/warning/error)" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    orca::utils::Logger::parseArgv(argc, argv);

    auto robot = std::make_shared<RobotModel>();
    robot->loadModelFromFile(urdf_url);
    robot->setBaseFrame("base_link");
    robot->setGravity(Eigen::Vector3d(0,0,-9.81));

    orca::optim::Controller controller(
        "controller"
        ,robot
        ,orca::optim::ResolutionStrategy::OneLevelWeighted
        ,QPSolver::qpOASES
    );


    /////////////////////////// traxxs ///////////////////////////////////////////::
    path::PathBounds4d path_bounds ,path_bounds2;
    path_bounds.dx << 0.2, 0.2, 0.2, 10.0;
    path_bounds.ddx = 100.0 * path_bounds.dx;
    path_bounds.j = 100.0 * path_bounds.ddx;		
    path_bounds2.dx << 0.4, 0.4, 0.4, 10.0;
    path_bounds2.ddx = 100.0 * path_bounds.dx;
    path_bounds2.j = 100.0 * path_bounds.ddx;		
    path::CartesianPathWaypoint wpt_start, wpt_end, wpt_start_temp;
    
    wpt_start.x.p << 0.5,-0.2,0.4 ;
//     wpt_end.x.p << -0.3,0.2,0.4 ; // Utiliser pour simuler la contrainte des articulations
     wpt_end.x.p <<0.5,0.2,0.4 ;
//     wpt_end.x.p << 0.4, 0.14, 0.3; 
    wpt_start_temp.x.p << 0.38969, -0.0184061, 0.652811;    
//     wpt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
//     wpt_end.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); 
    wpt_start_temp.pathConditionsPosition.dx << 0,0,0;
    wpt_start.pathConditionsPosition.dx << 0, 0, 0;
    wpt_start.pathConditionsOrientation.dx << 0;
    wpt_end.pathConditionsPosition.dx << 0, 0, 0;
    wpt_end.pathConditionsOrientation.dx << 0;
    std::shared_ptr< path::PathSegment > init_segment = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> (wpt_start_temp,wpt_start,path_bounds);
    init_segment -> init();
    std::shared_ptr< path::PathSegment > back_segment = std::make_shared<path::CartesianSegment<path::LinearSegment,path::SmoothStep7>>(wpt_start,wpt_end,path_bounds2);
    back_segment -> init();
    std::shared_ptr< path::PathSegment > come_segment = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_end, wpt_start, path_bounds2 );
    back_segment -> init();
//     std::vector< std::shared_ptr < path::PathSegment > > segments = createMonSegments(path_bounds) ;
    std::vector<std::shared_ptr<path::PathSegment>> segments;
    segments.push_back( init_segment);
    int compteur = 1;
    do{
//     
//        
//       
	segments.push_back( back_segment );
	segments.push_back( come_segment);
        compteur += 1;
     
        }while(compteur < 3);
    /////////////////////////////////////////////////////////////////////////////

    
    auto trajectory = std::make_shared< trajectory::Trajectory >();
    if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
    return 1;
  
    trajectory::TrajectoryState state, state_new;
      // the status of the tracker
    tracker::TrackerStatus status;
    auto validator = std::make_shared< tracker::TrackerSpaceValidatorPose4d >(8*1e-3, 100 );
    tracker::TrackerSpacePursuit tracker(validator);
//     tracker::TrackerTimePursuit tracker;
    tracker.reset( trajectory );
    double dt = 0.001;
    double t = 0 - dt;
    trajectory->getState( 0.0, state);

    if ( !trajectory->getState( 0.0, state) ) {
    cerr << "Could not get state !\n";
    return 1;
    }
    
    
    status = tracker.next( dt, state, state_new );
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////:
    auto cart_task = std::make_shared<CartesianTask>("CartTask-EE");
    controller.addTask(cart_task);
    cart_task->setRampDuration(0); // Activate immediately
    cart_task->setControlFrame("link_7"); //
    Eigen::Affine3d cart_pos_ref;
//     cart_pos_ref.translation() = Eigen::Vector3d( 0.38969, -0.0184061, 0.652811);
    Eigen::Quaterniond quat = orca::math::quatFromRPY(M_PI,0,0);
    cart_pos_ref.linear() = quat.toRotationMatrix();
    Vector6d cart_vel_ref = Vector6d::Zero();
    Vector6d cart_acc_ref = Vector6d::Zero();

    Vector6d P;
    P<<  2280.,2280.,2280,2280.,2280.,2280.;
    cart_task->servoController()->pid()->setProportionalGain(P);
    Vector6d I;
//     I << 26, 26, 26, 25 ,25 ,25;
//     cart_task->servoController()->pid()->setIntegralGain(I);

    Vector6d D;
    D << 47, 47, 47, 25.4, 25.4, 25.4;
    cart_task->servoController()->pid()->setDerivativeGain(D);
    
    cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);

    const int ndof = robot->getNrOfDegreesOfFreedom();

//     gravity regularisation tasks
     std::shared_ptr<gravityRegular> gravity_reg =  controller.addTask<gravityRegular>("gravity_regularisation");   
     gravity_reg->onResize();
     gravity_reg-> setWeight(10E-5);

//

    // Régularisation sur les contraintes 
    std::shared_ptr<ctrsRegTask> ctrs_reg = controller.addTask<ctrsRegTask>("contrainte régularisation");
    ctrs_reg->setWeight(10E-6);
//     ctrs_reg->set_nbr_ctrs(1);
     
     std::shared_ptr<JointTorqueLimitConstraint> jnt_trq_cstr =controller.addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");
    Eigen::VectorXd jntTrqMax(ndof);
    jntTrqMax.setConstant(200.0);
    jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax);
//
    std::shared_ptr<JointPositionLimitConstraint>  jnt_pos_cstr = controller.addConstraint<JointPositionLimitConstraint>("JointPositionLimit");
//
    auto jnt_vel_cstr = std::make_shared<JointVelocityLimitConstraint>("JointVelocityLimit");
    controller.addConstraint(jnt_vel_cstr);
    Eigen::VectorXd jntVelMax(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);
    
    // Energy constraint
    std::shared_ptr<EnergyConstraint>  energy_ctrs = std::make_shared<EnergyConstraint>("energyLimit");
   controller.addConstraint(energy_ctrs);
    energy_ctrs->onResize();

   Eigen::VectorXd cart_acc_des(6);
    cart_acc_des.setZero();

    GazeboServer gzserver(argc,argv);
    auto gzrobot = GazeboModel(gzserver.insertModelFromURDFFile(urdf_url));
    gzrobot.setModelConfiguration( { "joint_0","joint_1" ,"joint_2", "joint_3","joint_4","joint_5","joint_6"} , {1.0,0.,0.0,-1.57,0.0,1.57,0.});
     gazebo::event::Events::pause.Signal(true);

    std::vector<double> ec; 
    const int n_horizon_steps=2;
    Vector6d  ee_vel,  delta_X;
    ee_vel.setZero();
    delta_X.setZero();
    Eigen::MatrixXd Masse(ndof,ndof);
    Masse.setZero();
    Eigen::MatrixXd Lambda_(6,6);
    Lambda_.setZero();
    Eigen::MatrixXd J(6,ndof);
    J.setZero();
    Eigen::VectorXd Joint_torq_out(ndof);
    Joint_torq_out.setZero();
    Eigen::Vector3d Pos_laser;
    Pos_laser << 0.5,0.0,0.013;
    Eigen::MatrixXd rotMat(3,3);
    rotMat.setZero();
    Eigen::Vector3d ee_pos ;
    ee_pos.setZero();
    Eigen::Vector3d Z_7, Z_P, OM;
    Eigen::VectorXd jdotqdot, gravity_Torque;
    jdotqdot.resize(6);
    gravity_Torque.resize(7);

   double ec_predicted=0;
   double ec_next = 0;
   double temp = 0;
   bool indice = true;
   bool pointage_active = false;
        controller.globalRegularization()->euclidianNorm().setWeight(1.e-8);
     ////////////////////////////////////////////////////////////////////////
    
        cart_task->onActivationCallback([](){
        std::cout << "Activating CartesianTask..." << '\n';
    });
    std::ofstream f ("/home/administrateur/Bureau/plot/Ec.txt");
    std::ofstream f2("/home/administrateur/Bureau/plot/temps.txt");
    std::ofstream f3("/home/administrateur/Bureau/plot/Ec_predic.txt");
    std::ofstream f4("/home/administrateur/Bureau/plot/Pose_laser.txt");
    std::ofstream f5("/home/administrateur/Bureau/plot/Angle.txt");
    std::ofstream f6("/home/administrateur/Bureau/plot/dt.txt");
    std::ofstream f7("/home/administrateur/Bureau/plot/ee_pos.txt");
    std::ofstream f8("/home/administrateur/Bureau/plot/traxxs_pos.txt");
    std::ofstream f9("/home/administrateur/Bureau/plot/ee_vel.txt");
    std::ofstream f10("/home/administrateur/Bureau/plot/activation.txt");
    gzrobot.executeAfterWorldUpdate([&](uint32_t n_iter,double current_time,double dt)
    {
      
        robot->setRobotState(gzrobot.getWorldToBaseTransform().matrix()
                            ,gzrobot.getJointPositions()
                            ,gzrobot.getBaseVelocity()
                            ,gzrobot.getJointVelocities()
                            ,gzrobot.getGravity()
                        );

	// All tasks need the robot to be initialized during the activation phase
	  if(n_iter == 1)
	  {
	    controller.activateTasksAndConstraints();
	    jnt_pos_cstr->setHorizon(n_horizon_steps);
	    jnt_vel_cstr->setHorizon(n_horizon_steps);
	    energy_ctrs->setHorizon(n_horizon_steps);
	    controller.globalRegularization()->deactivate();
	  }
	

	ee_pos = ( cart_task->servoController()->getCurrentCartesianPose()).block(0,3,3,1);
	Z_P << 0,0,1;
	Z_7 = ( cart_task->servoController()->getCurrentCartesianPose()).block(0,0,3,3)*Z_P;
	Z_7 = Z_7.normalized();
	OM[0] = ee_pos[0] + Z_7[0]*(0.013-ee_pos[2])/Z_7[2];
	OM[1] = ee_pos[1] + Z_7[1]*(0.013-ee_pos[2])/Z_7[2];
//         controller.update(current_time, dt);
	state.x.segment(0,3) = ( cart_task->servoController()->getCurrentCartesianPose()).block(0,3,3,1);
	status = tracker.next( dt, state, state_new );
        cart_pos_ref.translation() = state_new.x.segment(0,3);
	cout << "state_new \n"<< state_new.x.segment(0,3)<<endl;
	cart_vel_ref.segment(0,3) = (state_new.pathConditions.dx).segment(0,3);
	cart_acc_ref.segment(0,3) = (state_new.pathConditions.ddx).segment(0,3);
	cout<<"ee_pose \n" << ee_pos <<endl;
	if (abs(ee_pos[2] -0.4) < 0.001 && (abs(ee_pos[1])< 0.001) ){
	  pointage_active = true;
	  f10 << current_time << endl;
	}
	
// 	cout << pointage_active<<endl;
// 	if (pointage_active){
// 	    rotMat = MatrixToPointage(cart_task->servoController()->getCurrentCartesianPose(),Pos_laser);
// 	    cart_pos_ref.linear()=rotMat;	  
// 	}
        cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
	cart_acc_des= cart_task->servoController()->getDesiredCartesianAcceleration();
	energy_ctrs->setAccelerationDes(cart_acc_des);	
	
	if (current_time>1.8 && current_time <2.8)
	{
	  indice = false;
	}else{
	  indice = true;}
	  
	energy_ctrs->setIndice(indice);
// 	ctrs_reg->set_matrix_ctrs_lb(1,jnt_pos_cstr->getLowerBound());
// 	ctrs_reg->set_matrix_ctrs_ub(1,jnt_pos_cstr->getUpperBound());	
	
	ee_vel = robot->getFrameVel("link_7");
	Masse = robot->getMassMatrix();
	J =  robot -> getJacobian("link_7");
	Lambda_ = (J*(Masse.inverse())*J.transpose()).inverse();
	delta_X = ee_vel*n_horizon_steps*dt +  0.5*cart_acc_des*n_horizon_steps*dt*n_horizon_steps*dt ;
	
	
 	controller.update(current_time, dt);
	 ec_next = energy_ctrs->getecNext()  ;
	Joint_torq_out = controller.getJointTorqueCommand();
	ec_predicted = delta_X.transpose() * Lambda_ * J *( Masse.inverse())*Joint_torq_out+ ec_next;
	 
	 //// Récupérer la projection de l'effecteur sur la table 

	ee_pos = ( cart_task->servoController()->getCurrentCartesianPose()).block(0,3,3,1);
	Z_P << 0,0,1;
	Z_7 = ( cart_task->servoController()->getCurrentCartesianPose()).block(0,0,3,3)*Z_P;
	Z_7 = Z_7.normalized();
	OM[0] = ee_pos[0] + Z_7[0]*(0.013-ee_pos[2])/Z_7[2];
	OM[1] = ee_pos[1] + Z_7[1]*(0.013-ee_pos[2])/Z_7[2];
	
	/////
	f<< 0.5*ee_vel.transpose()*Lambda_*ee_vel <<endl;
	f2<<current_time <<endl;
	f3 << ec_predicted << endl;
 	f4 << Pos_laser.transpose() <<endl;
	f5<<OM.transpose()<<endl;
	f7 <<(ee_pos).transpose() <<endl;
	f8 <<  (state_new.x.segment(0,3)).transpose() <<endl;
	f9 <<( (cart_task->servoController()->getCurrentCartesianVelocity()).segment(0,3)).transpose() << endl;
//         gzrobot.setJointGravityTorques(robot->getJointGravityTorques());

	if(controller.solutionFound())
	{
	  
	    gzrobot.setJointTorqueCommand( controller.getJointTorqueCommand() );

	}
	else 
	{
	      cout<<"solution not found \n" << endl;
	      exit(EXIT_SUCCESS);
	}
// 	else
// 	{
// 	    gzrobot.setBrakes(true);
// 	}
//         else
//         {
//             gzrobot.setJointGravityTorques(robot->getJointGravityTorques());
//         }
    });

    std::cout << "Simulation running... (GUI with \'gzclient\')" << "\n";
    
    gzserver.run();
    
    

    return 0;
}
