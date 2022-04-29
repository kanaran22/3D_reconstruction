
#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <Eigen/Core>
#include <typeinfo>
#include <math.h>
#include <time.h>
#include <iostream>

using namespace std ;
using namespace Eigen ;

float threshold = 0.1 ;

void tempRotate(igl::opengl::glfw::Viewer viewer,int objectIndex,RowVector3d v,double theta)
{
  MatrixXd V = viewer.data(objectIndex).V ;
  Quaterniond q(AngleAxisd(theta,v)) ;
  cout << endl ;
  cout << "Temp Rotate" << endl ;
  cout << "v : " << v << endl ;
  cout << "theta : " << theta << endl ;
  // cout << "Q : " << q << endl ;
  // cout << q << endl ;
  // cout << "V" << endl ;
  // cout << V ;
  // V = q * V ;
  MatrixXi F = viewer.data(objectIndex).F ;

  viewer.data(objectIndex).set_mesh(V,F) ;
}

void setup3D(igl::opengl::glfw::Viewer viewer)
{
  int R = 100;
  int G = 100;
  int B = 100;

  const Eigen::MatrixXd P_ = (Eigen::MatrixXd(1, 3) << 0, 0, 0).finished();
  const Eigen::MatrixXd P1 = (Eigen::MatrixXd(1, 3) << 1000, 0, 0).finished();
  const Eigen::MatrixXd P2 = (Eigen::MatrixXd(1, 3) << -1000, 0, 0).finished();

  const Eigen::MatrixXd P3 = (Eigen::MatrixXd(1, 3) << 0, 1000, 0).finished();
  const Eigen::MatrixXd P4 = (Eigen::MatrixXd(1, 3) << 0, -1000, 0).finished();

  const Eigen::MatrixXd P5 = (Eigen::MatrixXd(1, 3) << 0, 0, 1000).finished();
  const Eigen::MatrixXd P6 = (Eigen::MatrixXd(1, 3) << 0, 0, -1000).finished();

  cout<< "viewer: " << viewer.data_list.size() << "/" << viewer.selected_data_index << "\n";

  viewer.data().add_points(P_,Eigen::RowVector3d(R,G,B));
  R = 100;
  G = 0;
  B = 0;
  viewer.data(0).add_edges(P1, P2, Eigen::RowVector3d(R, G, B));
  R = 0;
  G = 100;
  B = 0;
  viewer.data(0).add_edges(P3, P4, Eigen::RowVector3d(R, G, B));
  R = 0;
  G = 0;
  B = 100;
  viewer.data(0).add_edges(P5, P6, Eigen::RowVector3d(R, G, B));
  
  cout<< "viewer: " << viewer.data_list.size() << "/" << viewer.selected_data_index << "\n";
}

void translate(igl::opengl::glfw::Viewer &viewer,int index,int direction,float translation)
{
  MatrixXd P = viewer.data(index).V ;
  for (int i = 0; i < P.rows(); i += 1)
  {
    P.row(i)[direction] = (P.row(i)[direction] + translation);
  }
  MatrixXi F = viewer.data(index).F ;
  viewer.data(index).set_mesh(P,F);
  viewer.data(index).set_face_based(true);

  // cout << endl ;
  // cout << "Moved Object : " << index << endl ;
  // cout << "P" << endl ;
  // cout << P << endl ;
  // cout << "V" << endl ;
  // cout << viewer.data(index).V << endl ;
  // cout << "F" << endl ;
  // cout << viewer.data(index).F << endl ;
}

void scale(igl::opengl::glfw::Viewer &viewer,int index,float scl)
{
  MatrixXd P = viewer.data(index).V ;
  MatrixXd avg = P.colwise().mean();
  avg.row(0)[1] = 0;
  for (int i = 0; i < P.rows(); i += 1)
  {
    P.row(i) = avg + scl * (P.row(i) - avg);
  }
  MatrixXi F = viewer.data(index).F ;
  viewer.data(index).set_mesh(P,F);
  viewer.data(index).set_face_based(true);
}

void Rotate(igl::opengl::glfw::Viewer &viewer,int objectIndex,int direction,float d_theta)
{
  MatrixXd P = viewer.data(objectIndex).V;
  double theta = d_theta;
  cout << endl ;
  cout << "Rotate : " << direction << "\t" 
        << "Angle : " << d_theta << endl ;
  switch(direction)
  {
    case 0 : for (int i = 0; i < P.rows(); ++i)
            {
              double x = P(i, 0);
              double y = P(i, 1);
              double z = P(i, 2);
              double x_, y_, z_;

              x_ = P(i, 0);
              y_ = (y * cos(theta) - z * sin(theta));
              z_ = (y * sin(theta) + z * cos(theta));

              P(i, 0) = x_;
              P(i, 1) = y_;
              P(i, 2) = z_;
            }
            break ;
    case 1 :for (int i = 0; i < P.rows(); ++i)
            {
              double x = P(i, 0);
              double y = P(i, 1);
              double z = P(i, 2);
              double x_, y_, z_;

              x_ = (x * cos(theta) + z * sin(theta));
              y_ = P(i, 1);
              z_ = (z * cos(theta) - x * sin(theta));

              P(i, 0) = x_;
              P(i, 1) = y_;
              P(i, 2) = z_;
            }
            break ;
    case 2 :for (int i = 0; i < P.rows(); ++i)
            {
              double x = P(i, 0);
              double y = P(i, 1);
              double x_, y_, z_;

              x_ = (x * cos(theta) - y * sin(theta));
              y_ = (y * cos(theta) + x * sin(theta));
              z_ = P(i, 2);

              P(i, 0) = x_;
              P(i, 1) = y_;
              P(i, 2) = z_;
            }
            break ;
    default: break ;
  }
  MatrixXi F = viewer.data(objectIndex).F ;
  viewer.data(objectIndex).set_mesh(P,F);
  viewer.data(objectIndex).set_face_based(true);
}

void newRotation(igl::opengl::glfw::Viewer &viewer,int objectIndex,int direction,float d_theta)
{
  MatrixXd P = viewer.data(objectIndex).V;
  double theta = d_theta;

}

MatrixXi FConcat(MatrixXi F1,MatrixXi F2,int x = 10)
{
  MatrixXi Temp(F2.rows(),F2.cols());
  Temp << F2 ;
  MatrixXi Points = (Eigen::MatrixXi(1, 3) << x , x , x ).finished().array() ;
  for(int i=0;i<F2.rows();i+=1)
  {
    F2.row(i) += Points ;
  }
  MatrixXi Out(F1.rows()+F2.rows(),F1.cols()) ; 
  Out << F1 , F2 ;
  return Out ;
}

MatrixXd VConcat(MatrixXd V1,MatrixXd V2)
{
  MatrixXd Out(V1.rows()+V2.rows(),V1.cols()) ;
  Out << V1 , V2 ;
  return Out ;
}

void saveMesh(igl::opengl::glfw::Viewer &viewer)
{ 
  MatrixXd Vt , Vt_1 , Vt_2 ;
  MatrixXi Ft , Ft_1 , Ft_2 ;

  int size = viewer.data_list.size() ;
  int x ;

  cout << endl << "Number of meshes in viewer : " << size << endl ;

  Vt = viewer.data(1).V ;
  Ft = viewer.data(1).F ;
  for(int i=2;i<=size;i+=1)
  {
    Vt_1 = Vt ;
    Vt_2 = viewer.data(i).V ;
    x = Vt.rows() ;
    Vt = VConcat(Vt_1,Vt_2) ;
    // cout << endl << "New Rows Before 2nd Merge : " << x << endl ;
    Ft_1 = Ft ;
    Ft_2 = viewer.data(i).F ;
    Ft = FConcat(Ft_1,Ft_2,x) ;
  }
  // cout << "[+] Writing : " << V << endl ;
  // cout << "[+] F : " << F << endl ;
  igl::writeOFF("OUTPUT.off",Vt,Ft);
  igl::writeOBJ("OUTPUT.obj",Vt,Ft);
  cout << "[+] Written to File" << endl ;
}

bool placeObject(igl::opengl::glfw::Viewer &viewer,int objectIndex,int faceIndex) 
{
  MatrixXi Face ;
  MatrixXd P_1 , P_2 , P_3 , Mid ;

  // Pi Values
  float pi2 = acos(0.0);
  float pi = pi2 * 2;
  float pi4 = pi2 / 2 ;

  // Getting Points
  Face = viewer.data(1).F.row(faceIndex) ;
  P_1 = viewer.data(1).V.row(Face.row(0)[0]) ;
  P_2 = viewer.data(1).V.row(Face.row(0)[1]) ;
  P_3 = viewer.data(1).V.row(Face.row(0)[2]) ;

  // Plane angle calculation
  float xa = P_2.row(0)[0] - P_1.row(0)[0];
  float ya = P_2.row(0)[1] - P_1.row(0)[1];
  float za = P_2.row(0)[2] - P_1.row(0)[2];
  float xb = P_3.row(0)[0] - P_1.row(0)[0];
  float yb = P_3.row(0)[1] - P_1.row(0)[1];
  float zb = P_3.row(0)[2] - P_1.row(0)[2];

  Mid = (P_1+P_2+P_3) / 3 ;

  // if(Mid(0,1) > 1)
  //   return false ;
  float prob = (float) rand()/RAND_MAX ;
  cout << endl ;
  cout << "[+] Probablity : " << prob << endl ;
  if(prob > threshold) 
  {
    cout << "[-] Greater than" ;
    return false ;
  }

  float xcoff = ya*zb - za*yb ;
  float ycoff = za*xb - xa*zb;
  float zcoff = xa*yb - ya*xb ;
  
  // if(!(xcoff>0&&ycoff>0&&zcoff>0))
  // {
  //   cout << "Returning False" <<endl ;
  //   return false ;
  // }

  float A = sqrt(pow(xcoff,2)+pow(ycoff,2)+pow(zcoff,2)) ;

  float x2 = ((xcoff/A) * (-Mid(0,2)/(zcoff/A))) + Mid(0,0) ;
  float y2 = ((ycoff/A) * (-Mid(0,2)/(zcoff/A))) + Mid(0,1) ;
  float z2 = 0 ;

  float x1 = Mid(0,0) ;
  float y1 = Mid(0,1) ;
  float z1 = Mid(0,1) ;

  float ry = pow(x2-x1,2)+pow(y2-y1,2) ;
  ry /= (pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2)) ;
  ry = sqrt(ry) ;
  ry = pi2 - ry ;

  float rz = abs(y2 - y1) ;
  rz /= sqrt(pow(x2-x1,2)+pow(y2-y1,2)) ;

  float tx = xcoff / A ;
  tx = acos(tx);
  float ty = ycoff / A ;
  ty = acos(ty);
  float tz = zcoff / A ;
  tz = acos(tz);

  float txy = sqrt(pow(xcoff,2)+pow(ycoff,2))/A ;
  txy = acos(txy) ;
  float tyz = sqrt(pow(ycoff,2)+pow(zcoff,2))/A ;
  tyz = acos(tyz) ;
  float tzx = sqrt(pow(zcoff,2)+pow(xcoff,2))/A ;
  tzx = acos(tzx) ;
  
  float tsz = ycoff / sqrt(pow(xcoff,2)+pow(ycoff,2)) ;
  tsz = asin(tsz) ;

  float t1 = pi4 ;
  float t2 = tz - pi + t1 ;
  float t3 = tx - t2 ;

  float nry = xcoff / sqrt(pow(xcoff,2)+pow(zcoff,2)) ;
  nry = pi2 - acos(nry) ;

  float nrz = sqrt(pow(xcoff,2)+pow(zcoff,2)) / A ;
  nrz = acos(nrz) ;

  cout << endl ;
  cout << "Calculated Angles " << endl ; 
  cout << "X : " << tx << endl ;
  cout << "Y : " << ty << endl ;
  cout << "Z : " << tz << endl ;
  cout << "XY : " << txy << endl ;
  cout << "YZ : " << tyz << endl ;
  cout << "Zx : " << tzx << endl ;

  // Perform Rotation
  // Rotate(viewer,objectIndex,0,tyz) ;
  // Rotate(viewer,objectIndex,1,tzx) ;
  // Rotate(viewer,objectIndex,2,txy) ;
  
  Rotate(viewer,objectIndex,0,pi2) ;
  Rotate(viewer,objectIndex,1,nry) ;
  Rotate(viewer,objectIndex,2,nrz) ;

  // Perform Translation
  translate(viewer,objectIndex,0,Mid.row(0)[0]) ;
  translate(viewer,objectIndex,1,Mid.row(0)[1]) ;
  translate(viewer,objectIndex,2,Mid.row(0)[2]) ;
  cout << "Done Translations " << endl ;

  return true ;
}