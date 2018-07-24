#include <orca/orca.h>

using namespace orca::constraint;
using namespace orca::optim;
// Add a constraint to Energy 
namespace orca
{
namespace constraint
{
class EnergyConstraint : public GenericConstraint
{
public:
   EnergyConstraint(const std::string& name) : GenericConstraint(name,ControlVariable::JointTorque )
   {

   }

   void setAccelerationDes(Eigen::VectorXd Acc)
   {
     AccelerationDes_ = Acc ;

   }
  
void getJointTorqOut (Eigen::VectorXd A)
{
  Joint_torq_out_ = A;
}

void onUpdateConstraintFunction(double current_time, double dt)
{
      
      const int ndof_ =this-> robot()->getNrOfDegreesOfFreedom();
      Masse_ =this-> robot()->getMassMatrix();
      J_ =this->  robot()->getJacobian("link_7");
      Lambda_ = (J_*(Masse_.inverse())*J_.transpose()).inverse();
      Xd_curr_ =this->  robot()->getFrameVel("link_7");
      gravity_Torque_ =this->  robot()->getJointGravityAndCoriolisTorques();
      jdotqdot_ =this->  robot()->getFrameBiasAcc("link_7");

  /////////
    
    double horizon_temps_ = n_horizon_steps_ * dt  ;

        
    delta_X = Xd_curr_*horizon_temps_ +  0.5*AccelerationDes_*horizon_temps_*horizon_temps_ ;
     
    ec_current =  0.5 * Xd_curr_.transpose() * Lambda_ * Xd_curr_ ;
    
    temp = delta_X.transpose()* Lambda_*(jdotqdot_ - J_*(Masse_.inverse())*gravity_Torque_) ;
    ec_next = ec_current + temp ;

    ec_lim = 0.15;

    constraintFunction().constraintMatrix()= (delta_X.transpose() * Lambda_ * J_ * (Masse_.inverse()));
    constraintFunction().lowerBound() <<  -100 - ec_next ;
    constraintFunction().upperBound() <<  ec_lim - ec_next ;

}
  double getecNext()  
  {
       return ec_next;
    
  }
  
  double setIndice(bool indice)
  {
    indice_ = indice;
  }
	
  void setHorizon(int n)
  {
    n_horizon_steps_ = n;
  }
  
 void onResize()
{
         const int ndof_ =this-> robot()->getNrOfDegreesOfFreedom();
// 	 std::cout << "ndof \n" << ndof_ <<std::endl;
         Masse_.resize(ndof_,ndof_);
	 Masse_.setZero(ndof_,ndof_);
         J_.resize(6,ndof_);
	 J_.setZero(6,ndof_);
	 jdotqdot_.resize(6);
	 jdotqdot_.setZero(6);
	 Lambda_.resize(6,6);
	 Lambda_.setZero(6,6);
	 gravity_Torque_.resize(ndof_);
	 gravity_Torque_.setZero(ndof_);
	 Xd_curr_.resize(6);
	 Xd_curr_.setZero(6);
	 AccelerationDes_.resize(6);
	 AccelerationDes_.setZero(6);
	 delta_X.resize(6);
	 delta_X.setZero(6);
	 Joint_torq_out_.resize(ndof_);
	 Joint_torq_out_.setZero(ndof_);
  	constraintFunction().resize(1,ndof_); // cette fonction resize Deja lb,C et ub, mets C à zero, lb à -inf, ub a +inf

	ec_current=0.;
	temp=0.;
	ec_next=0.;
	ec_lim=0.15;
	
    }
  

 
protected:
     Eigen::VectorXd  AccelerationDes_;
     bool indice_= true;
     std::string name_of_cstr ;
     double ec_current, temp, ec_next ;
     Eigen::MatrixXd Masse_, J_, J_transp_, Lambda_ ;
     Eigen::VectorXd Xd_curr_, gravity_Torque_, jdotqdot_, delta_X ,  Joint_torq_out_;
     double ec_lim; 
     int n_horizon_steps_=15;

};
}
}