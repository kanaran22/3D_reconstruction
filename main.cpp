#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <Eigen/Core>
#include <typeinfo>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <GLFW/glfw3.h>
#include <time.h>
#include <iostream>

#include "temp.h"

//#include <igl/MeshBooleanType.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

using namespace std;
using namespace Eigen ;


int main(int argc, char *argv[])
{
  int t = atoi(argv[1]) ;
  cout << "Running using " << t << endl ;
  srand( (unsigned)time( NULL ) );
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

  viewer.load_mesh_from_file("branchnew.off");
  scale(viewer,1,0.5) ;
  scale(viewer,2,0.0001) ;
  Selected_mesh = 2 ;

  int i ;
  int numberOfStars = 20 ;
  float threshold = 0.5 ;
  float prob ;

  // Automatic
  if(t==0)
  {
    cout << "[+] Using Automatic Loading" << endl ;
    for(i=0;i<numberOfStars;i+=1)
      viewer.load_mesh_from_file("star.off");
    for(int i=0;i<numberOfStars;i+=1)
      scale(viewer,3+i,0.005) ;
    for(i=0;i<numberOfStars;i+=1)
    {
      while(!placeObject(viewer,3+i,t+i))
      {
        t += 1 ;
        cout << "[+] Checking for face : " << t << endl ;
      };
    }
  }

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
  float trans_scale = 0.005;

  RowVector3d x_rot(1.0,0,0) ;
  RowVector3d y_rot(0,1.0,0) ;
  RowVector3d z_rot(0,0,1.0) ;

  // viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool
  viewer.callback_key_pressed = [&](decltype(viewer) &, unsigned int key, int mod)
  {
    // Create orbiting animation
    std::cout << key << " " << mod << "\n";

    if (key == 'Q')
      translate(viewer,Selected_mesh,0,trans_scale) ;
    if (key == 'A')
      translate(viewer,Selected_mesh,0,-trans_scale) ;
    if (key == 'W')
      translate(viewer,Selected_mesh,1,trans_scale) ;
    if (key == 'S')
      translate(viewer,Selected_mesh,1,-trans_scale) ;
    if (key == 'E')
      translate(viewer,Selected_mesh,2,trans_scale) ;
    if (key == 'D')
      translate(viewer,Selected_mesh,2,-trans_scale) ;

    if (key == '9')
      scale(viewer,Selected_mesh,upscale) ;
    if (key == '0')
      scale(viewer,Selected_mesh,downscale) ;

    if (key == 'Y')
      Rotate(viewer,Selected_mesh,0,d_theta) ;
    if (key == 'U')
      Rotate(viewer,Selected_mesh,0,-d_theta) ;
    if (key == 'H')
      Rotate(viewer,Selected_mesh,1,d_theta) ;
    if (key == 'J')
      Rotate(viewer,Selected_mesh,1,-d_theta) ;
    if (key == 'N')
      Rotate(viewer,Selected_mesh,2,d_theta) ;
    if (key == 'M')
      Rotate(viewer,Selected_mesh,2,-d_theta) ;

    if(key=='=')
    {
      viewer.load_mesh_from_file("star.off") ; 
      scale(viewer,viewer.data_list.size()-1,0.005) ;
      Selected_mesh = viewer.data_list.size() - 1 ;
      viewer.selected_data_index = Selected_mesh ;
    }

    if (key == '1')
    {
      //  if(Selected_mesh != 0)
      //  {
      //   //  cout << "V2 data() : " << V3.data() << endl;
      //   //  bool valid = igl::copyleft::cgal::mesh_boolean(P,F,V2,F2,boolean_type,V3,F3);
      //   //  cout<<"valid ---------------> "<<valid<<"\n";
      //   //  igl::writeOFF("OUTPUT.off",V3,F3);
      //   //  cout << "Union Done !" << V3.data() << endl ;
      //  }
      // viewer.load_mesh_from_file("tree.off");
      // igl::readOFF("tree.off", P, F);
      // Selected_mesh++;
      Selected_mesh-- ;
      if(Selected_mesh < 2)
        Selected_mesh = 2 ;
      viewer.selected_data_index = Selected_mesh ;
    }

    if(key == '2')
    {
      Selected_mesh++ ;
      if(Selected_mesh == viewer.data_list.size())
        Selected_mesh = viewer.data_list.size() - 1 ;
      viewer.selected_data_index = Selected_mesh ;
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
      saveMesh(viewer) ;
    }
    return false;
  };
  viewer.launch();
  
}