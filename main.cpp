#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <Eigen/Core>
#include <typeinfo>

#include "temp.h"

//#include <igl/MeshBooleanType.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

using namespace std;
using namespace Eigen ;


int main(int argc, char *argv[])
{
  int t = atoi(argv[1]) ;
  cout << "Running using " << t << endl ;
  // Inline mesh of a cube
  Eigen::MatrixXd V; //= (Eigen::MatrixXd(8, 3) << -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0).finished();
  Eigen::MatrixXi F; //= (Eigen::MatrixXi(12, 3) << 1, 7, 5, 1, 3, 7, 1, 4, 3, 1, 2, 4, 3, 8, 7, 3, 4, 8, 5, 7, 8, 5, 8, 6, 1, 5, 6, 1, 6, 2, 2, 6, 8, 2, 8, 4).finished().array() - 1;

  igl::readOFF("star.off", V, F); // Set up viewer
  
  Eigen::MatrixXd VC;
  Eigen::MatrixXd FC;

  Eigen::MatrixXd Vt;
  Eigen::MatrixXi Ft;

  Eigen::MatrixXd V2; 
  Eigen::MatrixXi F2; 
  Eigen::MatrixXd V3; 
  Eigen::MatrixXi F3; 

  Eigen::MatrixXd avg = V.colwise().mean();

  Eigen::VectorXi J;
  
  const Eigen::MatrixXd C;
  igl::opengl::glfw::Viewer viewer;
  Eigen::MatrixXd P = V;

  int Selected_mesh=0;
  viewer.data(Selected_mesh).set_mesh(P, F);
  viewer.data(Selected_mesh).set_face_based(true);

  viewer.load_mesh_from_file("cube.obj");
  viewer.load_mesh_from_file("cube.obj");
  viewer.load_mesh_from_file("star.off");
  viewer.load_mesh_from_file("star.off");


  int i ;
  for(i=0;i<10;i+=1)
    translate(viewer,1,0,-0.05) ;
  for(i=0;i<10;i+=1)
    translate(viewer,2,0,0.05) ;
  // for(i=0;i<10;i+=1)
  //   translate(viewer,4,1,0.05) ;
  scale(viewer,4,0.1) ;
  scale(viewer,5,0.1) ;

  placeObject(viewer,4,2) ;
  placeObject(viewer,5,3) ;

  MatrixXd Vt_1 , Vt_2 ;
  MatrixXi Ft_1 , Ft_2 ;
  int x ;

  int size = viewer.data_list.size() ;

  cout << endl << "Number of meshes in viewer : " << size << endl ;

  saveMesh(viewer) ;

  // Set mesh
  viewer.core().is_animating = true;
  
  igl::MeshBooleanType boolean_type(igl::MESH_BOOLEAN_TYPE_UNION);

  setup3D(viewer) ;

  double theta = 0;
  double d_theta = 0.087;
  double r = 1.732;
  double upscale = 1.1;
  double downscale = 0.93;
  float trans_scale = 0.05;

  // viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool
  viewer.callback_key_pressed = [&](decltype(viewer) &, unsigned int key, int mod)
  {
    // Create orbiting animation
    std::cout << key << " " << mod << "\n";

    if (key == 'Q')
      translate(viewer,3,0,trans_scale) ;
    if (key == 'A')
      translate(viewer,3,0,-trans_scale) ;
    if (key == 'W')
      translate(viewer,3,1,trans_scale) ;
    if (key == 'S')
      translate(viewer,3,1,-trans_scale) ;
    if (key == 'E')
      translate(viewer,3,2,trans_scale) ;
    if (key == 'D')
      translate(viewer,3,2,-trans_scale) ;

    if (key == '9')
      scale(viewer,3,upscale) ;
    if (key == '0')
      scale(viewer,3,downscale) ;
    if (key == 'Y')
    {
      theta = d_theta;
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double x_, y_, z_;

        x_ = (x * cos(theta) - y * sin(theta)) - avg(0) ;
        y_ = (y * cos(theta) + x * sin(theta)) - avg(1) ;
        z_ = P(i, 2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'U')
    {
      theta = -(d_theta);
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double x_, y_, z_;

        x_ = (x * cos(theta) - y * sin(theta)) - avg(0);
        y_ = (y * cos(theta) + x * sin(theta)) - avg(1);
        z_ = P(i, 2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'H')
    {
      theta = d_theta;
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double z = P(i, 2);
        double x_, y_, z_;

        x_ = (x * cos(theta) + z * sin(theta)) - avg(0);
        y_ = P(i, 1);
        z_ = (z * cos(theta) - x * sin(theta)) - avg(2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'J')
    {
      theta = -(d_theta);
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double z = P(i, 2);
        double x_, y_, z_;

        x_ = (x * cos(theta) + z * sin(theta)) - avg(0);
        y_ = P(i, 1);
        z_ = (z * cos(theta) - x * sin(theta)) - avg(2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'N')
    {
      theta = d_theta;
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double z = P(i, 2);
        double x_, y_, z_;

        x_ = P(i, 0);
        y_ = (y * cos(theta) - z * sin(theta)) - avg(1);
        z_ = (y * sin(theta) + z * cos(theta)) - avg(2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'M')
    {
      theta = -(d_theta);
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double z = P(i, 2);
        double x_, y_, z_;

        x_ = P(i, 0);
        y_ = (y * cos(theta) - z * sin(theta)) - avg(1);
        z_ = (y * sin(theta) + z * cos(theta)) - avg(2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }

    if (key == '1')
    {
       if(Selected_mesh != 0)
       {
        //  cout << "V2 data() : " << V3.data() << endl;
        //  bool valid = igl::copyleft::cgal::mesh_boolean(P,F,V2,F2,boolean_type,V3,F3);
        //  cout<<"valid ---------------> "<<valid<<"\n";
        //  igl::writeOFF("OUTPUT.off",V3,F3);
        //  cout << "Union Done !" << V3.data() << endl ;
       }
      viewer.load_mesh_from_file("tree.off");
      igl::readOFF("tree.off", P, F);
      Selected_mesh++;
    }

    cout << endl ;
    cout << "Selected Mesh : " << viewer.selected_data_index << endl;
    cout << "Data size " << viewer.data_list.size() << endl;
    cout<< " Viewer: " << viewer.data_list.size() << "/" << viewer.selected_data_index << "\n";

    // viewer.data(Selected_mesh).set_mesh(P, F);
    // viewer.data(Selected_mesh).set_face_based(true);
  
    viewer.data().point_size = argc > 2 ? std::stoi(argv[2]) : 7;

    if(key == 'X')
    {
      igl::writeOFF("OUTPUT.off",Vt,Ft);
      igl::writeOBJ("OUTPUT.obj",Vt,Ft);
    }
    
    if(key == 'C')
    {
      MatrixXd Vt_1 = viewer.data(1).V ;
      MatrixXd Vt_2 = viewer.data(2).V ;
      Vt = VConcat(Vt_1,Vt_2) ;
      int x = viewer.data(1).V.rows() ;
      MatrixXi Ft_1 = viewer.data(1).F ;
      MatrixXi Ft_2 = viewer.data(2).F ;
      Ft = FConcat(Ft_1,Ft_2,x) ;
    }

    return false;
  };
  viewer.launch();
  
}