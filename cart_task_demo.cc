#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>
#include <traxxs/tracker/tracker.hpp>
#include <traxxs/impl/traxxs_softmotion/traxxs_softmotion.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/OperationCaller.hpp>
#include "rtt_orca_reg.hpp"
#include "rtt_orca_ctrs.hpp"
#include <std_msgs/Float32.h>
using namespace orca::all;
using namespace orca_ros::all;
using namespace traxxs;

using namespace RTT;
using namespace RTT::os;

class CartTaskDemo : public RTT::TaskContext
{
public:
CartTaskDemo(const std::string& name)
: TaskContext(name)
, spinOnce("spinOnce")
{
    this->requires("ros_init")->addOperationCaller(spinOnce);
    this->provides("robot")->addProperty("name",robot_name_);
    this->provides("robot")->addProperty("base_frame",base_frame_);
    this->provides("robot")->addProperty("robot_description",robot_description_);
    this->provides("robot")->addProperty("robot_compensates_gravity",robot_compensates_gravity_);
    //orca/$(arg robot_name)/orca_controller/JointTorqueLimit
    this->provides("JointTorqueLimit")->addProperty("joint_torque_max",joint_torque_max_);
    this->provides("JointTorqueLimit")->addProperty("joint_velocity_max",joint_velocity_max_);

    this->addPort("JointPosition",port_joint_position_in_).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in_).doc("Current joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_out_).doc("Command joint torques");
    this->addPort("Ec",port_ec_).doc("Kinetic energy");
    this->addPort("Ec_predicted",port_ec_predicted_out_).doc("Predicted kinetic energy");
    this->addPort("Ec_lim",port_ec_lim_out_).doc("Kinetic energy limit");
    this->addProperty("Go",go).doc("go to point");
}

bool configureHook()
{
    // Create the robot model
    robot_kinematics_ = std::make_shared<orca::robot::RobotModel>(robot_name_);
    // Load the urdf file
    robot_kinematics_->loadModelFromString(robot_description_);
    robot_kinematics_->print();
    // Set the base frame (for lwr its usually link_0)
    robot_kinematics_->setBaseFrame(base_frame_);

    const int ndof = robot_kinematics_->getNrOfDegreesOfFreedom();

    // Instanciate and ORCA Controller
    std::cout << "Robot is loaded, loading controller" <<'\n';

    controller_ = std::make_shared<Controller>(
          "orca_controller"
        ,robot_kinematics_
        ,ResolutionStrategy::OneLevelWeighted
        ,QPSolver::qpOASES
    );
    controller_->setPrintLevel(1);
    std::cout << "Controller is loaded" <<'\n';
    // Remve gravity from solution because lwr auto compensates gravity
    controller_->removeGravityTorquesFromSolution(robot_compensates_gravity_);

    // Create the regularization task that will make the robot PID around its first
    // position. This task is very optional
    
    gravity_Reg_ = controller_->addTask<gravityRegular>("GravityTask");
    gravity_Reg_->setWeight(1e-6);
    // constraint regularization task 
//     ctrs_reg_ = std::make_shared<ctrsRegTask>("ctres_regu");
//     controller_->addTask(ctrs_reg_);
//     ctrs_reg_->setWeight(1e-8);
//     
    // Cartesian Task
    cart_task_ = std::make_shared<CartesianTask>("CartTask_EE");
    controller_->addTask(cart_task_);
    cart_task_->setControlFrame("link_7"); // We want to control the link_7
    ///// Traxxs
    path_bounds.dx << 0.3, 0.3, 0.3, 10.0;
    path_bounds.ddx = 20 * path_bounds.dx;
    path_bounds.j = 20 * path_bounds.ddx;		
    
    path_bounds2.dx << 0.8, 0.8, 0.8, 10.0;
    path_bounds2.ddx = 20 * path_bounds.dx;
    path_bounds2.j = 20 * path_bounds.ddx;	
    
    
    wpt_start_temp.x.p << 0.38969, -0.0184061, 0.652811;    
    wpt_start.x.p << 0.5,-0.2,0.4 ;
    wpt_end.x.p <<0.5,0.2,0.4 ;
    wpt_start_temp.pathConditionsPosition.dx << 0,0,0;
    wpt_start.pathConditionsPosition.dx << 0, 0, 0;
    wpt_start.pathConditionsOrientation.dx << 0;
    wpt_end.pathConditionsPosition.dx << 0, 0, 0;
    wpt_end.pathConditionsOrientation.dx << 0;
    init_segment = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> (wpt_start_temp,wpt_start,path_bounds);
    init_segment -> init();
     back_segment = std::make_shared<path::CartesianSegment<path::LinearSegment,path::SmoothStep7>>(wpt_start,wpt_end,path_bounds2);
    back_segment -> init();
    come_segment = std::make_shared< path::CartesianSegment< path::LinearSegment, path::SmoothStep7>> ( wpt_end, wpt_start, path_bounds2 );
    come_segment -> init();
    const int compteur{3};
    segments.push_back( init_segment);
    
    for (int i{0}; i< compteur ; i++)
    {
    segments.push_back( back_segment );
    segments.push_back( come_segment);
        }
    trajectory = std::make_shared< trajectory::Trajectory >();
    if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
      return 1;
    
     space_tracker.reset( trajectory );
    double dt = 0.001;
    double t = 0 - dt;
    trajectory->getState( 0.0, state);

    if ( !trajectory->getState( 0.0, state) ) {
    cerr << "Could not get state !\n";
    return 1;
    }
    
    
    
    
    ///////
    cart_task_->setRampDuration(0.0); // Activate immediately
    // Set the pose desired for the link_7
    Eigen::Affine3d cart_pos_ref;
    Vector6d cart_vel_ref ,   cart_acc_ref;
    
    
    cart_pos_ref.translation() = Eigen::Vector3d( 0.38969, -0.0184061, 0.652811);
    Eigen::Quaterniond quat = orca::math::quatFromRPY(M_PI,0,0);
    cart_pos_ref.linear() = quat.toRotationMatrix();

    // Set the desired cartesian velocity to zero
    cart_vel_ref.setZero();

    // Set the desired cartesian velocity to zero
    cart_acc_ref.setZero();
    
    // Now set the servoing PID
    Vector6d cartP;
    cartP<<  400.,400.,400.,400.,400.,400.;
//     cartP << 1080.,1080.,1080.,1080.,1080.,1080.;
    cart_task_->servoController()->pid()->setProportionalGain(cartP);
    Vector6d cartD;
    cartD << 40.,40.,40.,40.,40.,40.;
//     cartD <<80.,80.,80.,70.,70.,70.;
    cart_task_->servoController()->pid()->setDerivativeGain(cartD);
    cart_task_->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref );

    // The joint torque limit constraint
    joint_torque_constraint_ = controller_->addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");

    joint_torque_constraint_->setLimits(-joint_torque_max_,joint_torque_max_);

    // Joint position limits are automatically extracted from the URDF model. Note that you can set them if you want. by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax).
    joint_position_constraint_ = controller_->addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    // Joint velocity limits are usually given by the robot manufacturer
    joint_velocity_constraint_ = controller_->addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");
    joint_velocity_constraint_->setLimits(-joint_velocity_max_,joint_velocity_max_);
    // Energy constraint
    energy_ctrs_ = std::make_shared<EnergyConstraint>("EnergyConstraint");
    controller_->addConstraint(energy_ctrs_);
    energy_ctrs_->onResize();
      // Resize 
    Masse_.setZero(7,7);
    J_.setZero(6,7);
    Lambda_.setZero(6,6);
    ee_vel.resize(6);
    ee_vel.setZero(6);
    cart_acc_des.resize(6);
    cart_acc_des.setZero();
    jdotqdot_.resize(6);
    jdotqdot_.setZero(6);
    controller_ros_wrapper_ = std::make_shared<orca_ros::optim::RosController>(robot_kinematics_->getName(), controller_);
    cart_task_ros_wrapper_ = std::make_shared<orca_ros::task::RosCartesianTask>(robot_kinematics_->getName(), controller_->getName(), cart_task_); // TODO: take robot_kinematics
    go=false;


    return true;
}

bool startHook()

{
    controller_->activateTasksAndConstraints(); 
    joint_position_constraint_->setHorizon(n_horizon_t);
    joint_velocity_constraint_->setHorizon(n_horizon_t);
    energy_ctrs_->setHorizon(n_horizon_t); 
//     ctrs_reg_->setHorizon(n_horizon_t);
    controller_->globalRegularization()->deactivate();
//     cart_task_->deactivate();
    controller_->print();
  return true;
}

void updateHook()
{
    RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
    RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);

    // Return if not giving anything (might happend during startup)
    if(fp == RTT::NoData || fv == RTT::NoData)
    {
        static bool p = true;
        if(p)
        {
            p = false;
            log(RTT::Info) << getName() << " ------> waiting for (q,dq)" << endlog();
        }
      //log(RTT::Info) << "Robot ports empty !" << endlog();
      return;
    }

    static bool p = true;
    if(p)
    {
        p = false;
        log(RTT::Info) << getName() << " ------> First (q,dq) received !" << endlog();
    }

    robot_kinematics_->setRobotState(this->joint_position_in_,this->joint_velocity_in_);

    auto current_time = RTT::os::TimeService::Instance()->getNSecs();
    double dt = this->getPeriod(); // the component period
//     validator = std::make_shared< tracker::TrackerSpaceValidatorPose4d >(0.1, 100 );
//     tracker::TrackerSpacePursuit tracker(validator);

    state.x.segment(0,3) = ( cart_task_->servoController()->getCurrentCartesianPose()).block(0,3,3,1);
    status =  space_tracker.next( dt, state, state_new );
    Eigen::Affine3d cart_pos_ref;
    Vector6d cart_vel_ref ,   cart_acc_ref;
    cart_pos_ref.translation() = state_new.x.segment(0,3);
    Eigen::Quaterniond quat = orca::math::quatFromRPY(M_PI,0,0);
    cart_pos_ref.linear() = quat.toRotationMatrix();
    // Set the desired cartesian velocity to zero
    cart_vel_ref.segment(0,3)=(state_new.pathConditions.dx).segment(0,3);
    // Set the desired cartesian velocity to zero
    cart_acc_ref.segment(0,3)=(state_new.pathConditions.ddx).segment(0,3);
    
    cart_task_->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref );
    cart_acc_des= cart_task_->servoController()->getDesiredCartesianAcceleration();
    energy_ctrs_->setAccelerationDes(cart_acc_des);	
//     energy_ctrs->setIndice(indice);
//     Step the controller
    ee_vel = robot_kinematics_->getFrameVel("link_7");
    Masse_ = robot_kinematics_->getMassMatrix();
    J_ =  robot_kinematics_ -> getJacobian("link_7");
    Lambda_ = (J_*(Masse_.inverse())*J_.transpose()).inverse();
    jdotqdot_ = robot_kinematics_->getFrameBiasAcc("link_7");
    Vector6d delta_X;
//     if (go)
// 	cart_task_->activate();
    controller_->update(current_time,dt);
    double ec_next{0}, temp{0} ;
    
   
    delta_X = ee_vel*n_horizon_t*dt +  0.5*cart_acc_des*n_horizon_t*dt*n_horizon_t*dt ;



//     cout << "énergie cinétique \n" << ec <<endl;
    // Method 2 : Break when no solution found
    if(controller_->solutionFound())
    {
        // NOTE : breaks are automatically disabled when sending a command
        // So no need to call gzrobot->setBrakes(false);
        port_joint_torque_out_.write( controller_->getJointTorqueCommand() );
	Joint_torq_out_ec =( controller_->getJointTorqueCommand()+robot_kinematics_->getJointGravityTorques());
	ec = 0.5*ee_vel.transpose()*Lambda_*ee_vel;
	temp = delta_X.transpose()* Lambda_*(jdotqdot_ - J_*(Masse_.inverse())*(robot_kinematics_->getJointGravityAndCoriolisTorques())) ;
	ec_next = ec + temp ;
	ec_predi = delta_X.transpose() * Lambda_ * J_ *( Masse_.inverse())*(Joint_torq_out_ec )+ ec_next;

	ec_msg.data = ec;
	ec_predi_msg.data = ec_predi;
	ec_lim_msg.data = 0.15;
	port_ec_.write(ec_msg);
	port_ec_predicted_out_.write(ec_predi_msg);
	port_ec_lim_out_.write(ec_lim_msg);
    }
    else
    {
        // TODO : send a signal to KRL STOP2
        // No Solution found, breaking hard
        log(Error) << "No Solution found, setting to error" << endlog();
        this->error();
        return;
    }
    spinOnce();
}


private:
    RTT::OutputPort<std_msgs::Float32> port_ec_lim_out_, port_ec_predicted_out_,port_ec_;
    RTT::OperationCaller<void(void)> spinOnce;
    std::string robot_description_;
    std::string robot_name_;
    std::string base_frame_;
    bool robot_compensates_gravity_ = true;
    std::shared_ptr<orca::robot::RobotModel> robot_kinematics_;
    std::shared_ptr<orca::optim::Controller> controller_;
    std::shared_ptr<orca::task::CartesianTask> cart_task_;
    std::shared_ptr<orca::constraint::JointTorqueLimitConstraint> joint_torque_constraint_;
    std::shared_ptr<orca::constraint::JointPositionLimitConstraint> joint_position_constraint_;
    std::shared_ptr<orca::constraint::JointVelocityLimitConstraint> joint_velocity_constraint_;
    std::shared_ptr<orca::constraint::EnergyConstraint>  energy_ctrs_;
    std::shared_ptr<orca::task::gravityRegular> gravity_Reg_;
    std::shared_ptr<orca::task::ctrsRegTask> ctrs_reg_;
    Eigen::VectorXd joint_torque_max_;
    Eigen::VectorXd joint_velocity_max_;
    Eigen::VectorXd cart_acc_des;
    std_msgs::Float32 ec_msg,ec_predi_msg,ec_lim_msg;

    bool indice;
    double ec;
    double ec_predi;
    Eigen::VectorXd ee_vel;
    Eigen::Matrix<double,7,7> Masse_;
    Eigen::Matrix<double,6,7> J_;
    Eigen::Matrix<double,6,6> Lambda_;
    Eigen::VectorXd jdotqdot_;
    Eigen::VectorXd Joint_torq_out_ec;
    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out_;
    const int n_horizon_t = 15;

    Eigen::VectorXd joint_torque_out_,
                    joint_position_in_,
                    joint_velocity_in_;
    bool go;
    
    path::PathBounds4d path_bounds ,path_bounds2;
    path::CartesianPathWaypoint wpt_start, wpt_end, wpt_start_temp;   
    std::vector<std::shared_ptr<path::PathSegment>> segments;
    std::shared_ptr< path::PathSegment > init_segment;
     std::shared_ptr< path::PathSegment > come_segment ;
      std::shared_ptr< path::PathSegment > back_segment; 
    std::shared_ptr<trajectory::Trajectory> trajectory;
    trajectory::TrajectoryState state, state_new;
    tracker::TrackerStatus status;
    std::shared_ptr<tracker::TrackerSpaceValidatorPose4d> validator =std::make_shared< tracker::TrackerSpaceValidatorPose4d >(0.05, 100 );
    tracker::TrackerTimePursuit tracker;
    tracker::TrackerSpacePursuit space_tracker{validator};
    
    std::shared_ptr<orca_ros::optim::RosController> controller_ros_wrapper_;
    std::shared_ptr<orca_ros::task::RosCartesianTask> cart_task_ros_wrapper_;
};

ORO_CREATE_COMPONENT(CartTaskDemo)
