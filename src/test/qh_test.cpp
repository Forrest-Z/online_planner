#define QUICKHULL_IMPLEMENTATION
#include <thirdparty/3d-quickhull/quickhull.h>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <time.h>

using namespace std;
using namespace std::chrono;

int main(int argc, char** argv){
    ros::init(argc, argv, "qh_test");
    ros::NodeHandle nh("~");

    int n = nh.param("qh_n", 1000);
    float radius = nh.param("qh_r", 3.0f);
    srand((unsigned)time(NULL));
    qh_vertex_t vertices[n];

    for (int i = 0; i < n; ++i) {
        float a0 = (static_cast<float>(rand())/static_cast<float>(RAND_MAX) * M_PI * 2);
        float a1 = (static_cast<float>(rand())/static_cast<float>(RAND_MAX) * M_PI * 2);
        vertices[i].z = sin(a0) * radius;
        vertices[i].x = cos(a1) * cos(a0) * static_cast<float>(rand())/static_cast<float>(RAND_MAX) *radius ;
        vertices[i].y = sin(a1) * cos(a0) * static_cast<float>(rand())/static_cast<float>(RAND_MAX) *radius ;
    }
    
    cout<<"random "<<n<<" vertices generated"<<endl;
    auto start = high_resolution_clock::now();
    qh_mesh_t mesh = qh_quickhull3d(vertices, n);

    auto end1 = high_resolution_clock::now();
    cout<<"qh_quickhull3d done in "<<duration_cast<microseconds>(end1 - start).count()<<" mu sec"<<endl;

    unsigned int num_vertices = mesh.nvertices;
    Eigen::MatrixXd eig_points = Eigen::MatrixXd::Zero(num_vertices, 3);
    int num_inserted = 0;
    for(unsigned int i=0; i< num_vertices; ++i){
        Eigen::RowVector3d v;
        v(0) = mesh.vertices[i].x;
        v(1) = mesh.vertices[i].y;
        v(2) = mesh.vertices[i].z;
        bool is_same = false;
        for(int j=num_inserted - 1; j >= 0; --j){
            if((eig_points.row(j) - v).norm() < 0.05){ // Ignore too close points
                is_same = true;
                break;
            }
        }
        if(!is_same) eig_points.row(num_inserted++) = v;
    }
    auto end2 = high_resolution_clock::now();
    Eigen::MatrixXd true_vertices = eig_points.block(0, 0, num_inserted, 3);
    cout<<"converting to eigen done in "<<duration_cast<microseconds>(end2 - end1).count()<<" mu sec"<<endl;
    cout<<"convex hull vertices : "<<true_vertices<<endl;
    cout<<"total "<<true_vertices.rows()<<" points"<<endl;

    qh_vertex_t vertices2[8];
    for(int i=0; i<8;++i){
        vertices2[i].x = (i >= 4)? -1.0 : 1.0;
        vertices2[i].y = (i%4 >= 2)? -1.0 : 1.0;
        vertices2[i].z = (i%2)? -1.0: 1.0;
    }
    start = high_resolution_clock::now();
    qh_mesh_t mesh2 = qh_quickhull3d(vertices2, 8);
    auto end3 = high_resolution_clock::now();
    cout<<"qh_quickhull3d of 8 cube vertices done in "<<duration_cast<nanoseconds>(end3 - start).count()<<" nsec"<<endl;
    qh_vec3_t p;
    p.x = nh.param("px", 0.0);
    p.y = nh.param("py", 0.0);
    p.z = nh.param("pz", 0.0);
    start = high_resolution_clock::now();
    bool isinside = is_inside_qhull(&mesh2, p);
    auto end4 = high_resolution_clock::now();
    cout<<"is inside qhull took " <<duration_cast<nanoseconds>(end4 - start).count()<<" nsec"<<endl;
    cout<<p.x<<", "<<p.y<<", "<<p.z<<" is inside the cube? > "<< (isinside? "true" : "false") <<endl;


    return 1;
}