#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/cgal/mesh_boolean.h>

// #include <igl/MeshBooleanType.h>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>

using namespace std;

int main(int argc, char *argv[])
{
  // Inline mesh of a cube
  Eigen::MatrixXd V = (Eigen::MatrixXd(8, 3) << -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0).finished();
  Eigen::MatrixXi F = (Eigen::MatrixXi(12, 3) << 1, 7, 5, 1, 3, 7, 1, 4, 3, 1, 2, 4, 3, 8, 7, 3, 4, 8, 5, 7, 8, 5, 8, 6, 1, 5, 6, 1, 6, 2, 2, 6, 8, 2, 8, 4).finished().array() - 1;

  igl::readOFF("star.off", V, F); // Set up viewer
  Eigen::MatrixXd V2; 
  Eigen::MatrixXd F2; 

  Eigen::MatrixXd avg = V.colwise().mean();

  
  const Eigen::MatrixXi C;
  igl::opengl::glfw::Viewer viewer;

  int Selected_mesh=0;

  // Set mesh
  viewer.core().is_animating = true;
  // Initialize point
  // Eigen::MatrixXd P = (Eigen::MatrixXd(1,3)<<1.5,0,0).finished();
  Eigen::MatrixXd P = V;
  // function will be  called before every draw
  // viewer.callback_mouse_down =
  //  [&V,&F,&C](igl::opengl::glfw::Viewer& viewer, int, int)->bool

  // cout << P.row(3)[0]<<'\n';
  // XYZ Axis Generation
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

  viewer.load_mesh_from_file("tree.off");
  // viewer.load_mesh_from_file("star.off");


  // viewer.data().add_points(P_,Eigen::RowVector3d(R,G,B));
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


  double theta = 0;
  double d_theta = 0.001;
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
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i)[0] = (P.row(i)[0] + trans_scale);
      }
    if (key == 'A')
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i)[0] = (P.row(i)[0] - trans_scale);
      }
    if (key == 'W')
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i)[1] = (P.row(i)[1] + trans_scale);
      }
    if (key == 'S')
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i)[1] = (P.row(i)[1] - trans_scale);
      }
    if (key == 'E')
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i)[2] = (P.row(i)[2] + trans_scale);
      }
    if (key == 'C')
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i)[2] = (P.row(i)[2] - trans_scale);
      }
    if (key == '9')
    {
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i) = avg + downscale * (P.row(i) - avg);
      }
    }
    if (key == '0')
    {
      for (int i = 0; i < V.rows(); i += 1)
      {
        P.row(i) = avg + upscale * (P.row(i) - avg);
      }
    }
    if (key == 'Y')
    {
      theta += d_theta;
      for (int i = 0; i < V.rows(); ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double x_, y_, z_;

        x_ = (x * cos(theta) + y * sin(theta)) - avg(0);
        y_ = (y * cos(theta) - x * sin(theta)) - avg(1);
        z_ = P(i, 2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'U')
    {
      theta -= d_theta;
      for (int i = 0; i < 22; ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double x_, y_, z_;

        x_ = (x * cos(theta) + y * sin(theta)) - avg(0);
        y_ = (y * cos(theta) - x * sin(theta)) - avg(1);
        z_ = P(i, 2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'H')
    {
      theta += d_theta;
      for (int i = 0; i < 22; ++i)
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
      theta -= d_theta;
      for (int i = 0; i < 22; ++i)
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
      theta += d_theta;
      for (int i = 0; i < 22; ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double z = P(i, 2);
        double x_, y_, z_;

        x_ = P(i, 0);
        y_ = (y * cos(theta) - z * sin(theta)) - avg(1);
        z_ = (z * sin(theta) + y * cos(theta)) - avg(2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }
    if (key == 'M')
    {
      theta -= d_theta;
      for (int i = 0; i < 22; ++i)
      {
        double x = P(i, 0);
        double y = P(i, 1);
        double z = P(i, 2);
        double x_, y_, z_;

        x_ = P(i, 0);
        y_ = (y * cos(theta) - z * sin(theta)) - avg(1);
        z_ = (z * sin(theta) + y * cos(theta)) - avg(2);

        P(i, 0) = x_;
        P(i, 1) = y_;
        P(i, 2) = z_;
      }
    }

    if (key == '1')
    {
      viewer.load_mesh_from_file("star.off");
      Selected_mesh++;
    }

    cout << "Selected Mesh : " << viewer.selected_data_index << endl;
    cout << "Data size " << viewer.data_list.size() << endl;
    viewer.data(Selected_mesh).set_mesh(P, F);
    viewer.data(Selected_mesh).set_face_based(true);

    // viewer.data().clear();
    // viewer.data().add_points(P, Eigen::RowVector3d(0.0, 1.0, 0.0));
  
    viewer.data().point_size = argc > 2 ? std::stoi(argv[2]) : 7;
    // viewer.core().orthographic = !(viewer.core().orthographic);

    std::cout << P;
    // viewer.data().set_points(P, Eigen::RowVector3d(1, 1, 1));
    if( key == 'L')
    {
      igl::writeOBJ("OUTPUT.obj",P,F);
    }

    return false;
  };
  viewer.launch();
  
}