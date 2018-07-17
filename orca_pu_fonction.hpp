
#include <orca/orca.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo_client.hh>
#include <sdf/parser_urdf.hh>
#include <fstream>
#include <thread>
#include <iostream>
#include <fstream>

using namespace Eigen ;
using namespace std;
struct OrthBasis{
  Eigen::MatrixXd B;
  Eigen::VectorXd origin;
  int r;
  };
///////////////////////////////////////////////////////////////////////////////////////////////////////::
Eigen::MatrixXd MatrixToPointage(Eigen::MatrixXd T, Eigen::Vector3d Pos_laser)
{
	  
	  
	  
	  Eigen::Vector3d Z_P ;
	  Z_P << 0,0,1;		// This vector define the vector Z in the base of end-effector
	  
	  Eigen::Vector3d Pose_laser;
	  Pose_laser = Pos_laser;     

	  
	  Eigen::Vector3d Z_7;		// This vector define Z in the base 
	  Z_7 = (T.block(0,0,3,3))*Z_P;
	  Z_7.normalize();
	  Eigen::Vector3d OM, Pose_ee;		// The projection of the end-effector in the plan of base 
	  Pose_ee = T.block(0,3,3,1);
	  OM[0] = Pose_ee[0] + Z_7[0]*(0.013-Pose_ee[2])/Z_7[2];
	  OM[1] = Pose_ee[1] + Z_7[1]*(0.013-Pose_ee[2])/Z_7[2];
	  OM[2] = Pose_laser[2];
	  
	  Eigen::Vector3d Z_des;		// Compute the Z desired		
	  Z_des = (-Pose_ee + Pose_laser) ;
	  Z_des.normalize();
	  Eigen::Vector3d Z_P_O;
	  Z_P_O = T.block(0,0,3,3)*Z_P;
	  Z_P_O.normalize();
	  double theta =- acos(Z_des.dot(Z_P_O));
	  Eigen::Vector3d axe_rot = Z_des.cross(Z_P_O);
	  double norm;
	  norm = axe_rot.norm();
	  axe_rot.normalize();
	  double x,y,z;
	  x = axe_rot[0];
	  y = axe_rot[1];
	  z = axe_rot[2];
	  Eigen::Matrix<double,3,3>mat, rot_base, Id;
	  mat.setZero();

	  // Rotation de Rodrigue
	
	  mat << 0, -axe_rot[2], axe_rot[1] ,
		      axe_rot[2], 0,  -axe_rot[0],
		      -axe_rot[1], axe_rot[0], 0; 
	  Id.setIdentity();	     
	  mat = Id + sin(theta)*mat + (1 - cos(theta))*mat*mat;
	
	
	  return mat*T.block(0,0,3,3);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////    

Eigen::MatrixXd GetRowsIndexDescOrder( Eigen::MatrixXd A) 
 {
   int col = A.rows();   
   Eigen::VectorXd Diag(col);
   double temp;
   bool permut(false);
   int iteration(0);
   
   // Vecteur à trier 
   for (int i(0); i<col; i++){
     Diag.segment(i,1) << A.block(i,i,1,1);
     }
 
 // Sauvegarder ce vecteur d'abord pour supprmier les terms identique 
  Eigen::VectorXd InitVector(col);
  InitVector = Diag; 
  
  //Permuter le diagonale pour mettre en ordre décroissant 
   do{
     permut=false;
   for (int j(0); j<col-1;j++){
     if (Diag[j] < Diag[j+1]) // inférieur si en ordre croissant
     {
      temp = Diag[j] ;
      Diag[j] = Diag[j+1];
      Diag[j+1] = temp;
      permut = true;
      }
   }
   iteration ++;
   }while (permut == true);
//    

   // Trouver les différents valeurs, et stocker le vecteur de taille réduite
  Eigen::VectorXd differentValue(1);
  differentValue[0] = Diag[0];
  for ( int i(0) ; i< col; i ++){
	for (int j(i); j<col; j++){
	  if (Diag[i]!= Diag[j])
	  {
	    differentValue.conservativeResize(differentValue.size()+1);
	    differentValue[differentValue.size()-1] =Diag[j] ; 
	    i =j;
	  }
	}
  }
  // Trouver combien de nombre correspont pour chaque valeur
  Eigen::VectorXd nbrDiffValue(differentValue.size());
  int compte(0);
  for (int i(0); i< differentValue.size(); i++){
    for (int j(0); j < Diag.size(); j++){
      if (differentValue[i] == Diag[j])
      {compte ++;
	
      }
    }
      nbrDiffValue[i] = compte;
      compte = 0;
      
  }
 
  // Réduire la taille de vecteur init
  Eigen::VectorXd InitRedu(1);
  InitRedu[0] = InitVector[0];
  for ( int i(0) ; i< col; i ++){
	for (int j(i); j<col; j++){
	  if (InitVector[i]!= InitVector[j])
	  {
	    InitRedu.conservativeResize(InitRedu.size()+1);
	    InitRedu[InitRedu.size()-1] =InitVector[j] ; 
	    i =j;
	  }
	}
  }
 Eigen::VectorXd Index(differentValue.size());
 
     for (int i(0); i<differentValue.size();i++)
   {
      int lieu(0);
      for (int k(0); k<differentValue.size();k++)
      {
	if (differentValue[i] == InitRedu[k])
	{
	  Index[i] = lieu+1;
	}
	lieu ++;
	}     
   }
  Eigen::MatrixXd globe(differentValue.size(),2);
  globe.block(0,0,differentValue.size(),1) = Index;
globe.block(0,1,differentValue.size(),1) = nbrDiffValue;

  return globe;
 }
 
////////////////////////////////////////////////////////////////////////////////////////////////////    
 
 Eigen::MatrixXd SortRows(Eigen::MatrixXd A, Eigen::MatrixXd b)
 {
   Eigen::MatrixXd As(A.rows(),A.cols());
   int n = A.cols();
   As.setZero();
    int ligne(0);
    int saut(0);
   for (int i(0); i < b.rows(); i++){
//    As.block(b(i,1)*i,0,b(i,1),n)= A.block((b(i,0)-1)*b(i,1),0,b(i,1),n) ;
     if(b(i,0) == 1){
     As.block(ligne,0,b(i,1),n) = A.block(0 ,0 , b(i,1),n);
     ligne += b(i,1);     
    }else{
     int somme(0);
      for (int k(0); k< b.rows(); k++){
	if (b(k,0) < b(i,0)){
	  somme += b(k,1);
	}
      }
 
       As.block(ligne,0,b(i,1),n) = A.block(somme ,0 , b(i,1),n);
       ligne += b(i,1);
  }

   }
   return As ; 
 }
 ////////////////////////////////////////////////////////////////////////////////////////////////////    
   
OrthBasis GetOrthBasis(Eigen::MatrixXd J) 
{
    int n(J.cols()),m(J.rows()) ; 
    int i(0), r ;  
   
    Eigen::MatrixXd B(1,n); 
    Eigen::VectorXd Origin;
    Origin.resize(1);
        for (int k(0); k< m- 1; k++)  // m-1 ou m ?
    {
      
      if (i > n || i ==n)
      {
	break;
      }
      B.conservativeResize(i+1,n);
      B.block(i,0,1,n)=J.block(k,0,1,n);
      for (int j(0); j < i-1; j++)
      {
	B.block(i,0,1,n)= B.block(i,0,1,n) - (B.block(i,0,1,n)*(B.block(j,0,1,n)).transpose())*B.block(j,0,1,n);
      }
      
      if ((B.block(i,0,1,n)).norm() > 1E-18)
      {
	B.block(i,0,1,n) = B.block(i,0,1,n) / (B.block(i,0,1,n)).norm();
	Origin.conservativeResize(i+1);
	Origin[i] = k;
	i++;
      }
	     
     }
  
    r = i ;
    OrthBasis data;
    data.B = B;
    data.origin = Origin;
    data.r = r;
    return data;
 }
    
  /////////////////////////////////////////////////////////////////////////////////////////////////////    

  Eigen::MatrixXd getSubDiagMatrix(Eigen::MatrixXd As, OrthBasis data)
  {
    Eigen::VectorXd origine=data.origin;
    int r = data.r;
    int n = As.cols();
    Eigen::MatrixXd Asr(r,n);
    for (int i(0); i < r ; i++)
    {
      Asr.block(i,0,1,n) = As.block(origine[i],0,1,n);
     }
           
    return Asr;
  }
  
 ////////////////////////////////////////////////////////////////////////////////////////////////////    

  Eigen::MatrixXd getGeneralizedPojector(Eigen::MatrixXd A, Eigen::MatrixXd J)
  {
    int ndof = 7;
      Eigen::MatrixXd Index;
  Eigen::MatrixXd As;
  Eigen::MatrixXd Js;
  Eigen::MatrixXd Asr;
  Eigen::MatrixXd Jtot(6+ndof, ndof);
  Eigen::MatrixXd Id(ndof,ndof);
  Id.setIdentity();
  Eigen::MatrixXd G_projector(ndof,ndof);
  OrthBasis data;
    Jtot.block(0,0,6,7) = J;
    (Jtot.block(6,0,7,7)).setIdentity();
    
  Index = GetRowsIndexDescOrder(A);
  As = SortRows(A,Index);
//   cout << "As \n" << As <<endl;
  Js = SortRows(Jtot,Index);
  
  data = GetOrthBasis(Js);
//   cout << "sortir data \n" << '\n'
// 	  << "B\n" << data.B << '\n'
// 	  << "origine \n" << data.origin << '\n'
// 	  << "r \n" << data.r << '\n' ;
  Asr = getSubDiagMatrix(As,data);
   
        
  G_projector = Id - (data.B).transpose()*Asr*(data.B);
    return G_projector  ;   
  }
//    
   

