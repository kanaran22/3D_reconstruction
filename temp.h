
#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <Eigen/Core>
#include <typeinfo>

using namespace std ;
using namespace Eigen ;

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
  for (int i = 0; i < P.rows(); i += 1)
  {
    P.row(i) = avg + scl * (P.row(i) - avg);
  }
  MatrixXi F = viewer.data(index).F ;
  viewer.data(index).set_mesh(P,F);
  viewer.data(index).set_face_based(true);
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