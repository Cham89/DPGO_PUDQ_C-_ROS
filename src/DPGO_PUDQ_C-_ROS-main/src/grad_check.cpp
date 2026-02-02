#include "pudq_lib.hpp"
#include "graph.hpp"
#include "optimization.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

Eigen::VectorXd generate_random_tangent(const pudqlib::LocalGraph& G) {
    int N = G.vertices_pudq.size();
    Eigen::VectorXd eta(4 * N);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> d(0.0, 1.0);

    for (int i = 0; i < N; ++i) {
        Eigen::Vector4d rand_vec;
        rand_vec << d(gen), d(gen), d(gen), d(gen);
        
        Eigen::Matrix4d Pi = pudqlib::P_x_new(G.vertices_pudq[i]);
        eta.segment<4>(4 * i) = Pi * rand_vec;
    }

    double norm = pudqlib::riemannian_norm(G, eta, 0);
    return eta / norm;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: ./grad_check <path_to_graph_folder>" << std::endl;
        return 1;
    }

    std::string graph_dir = argv[1];
    std::cout << "Loading graph from: " << graph_dir << "..." << std::endl;
    pudqlib::Graph global_graph = graph::load_graph_from_matlab_export(graph_dir);
    std::vector<pudqlib::MultiGraph> mgs = graph::distributebiggraph_withinverseedge(global_graph, 1);
    pudqlib::LocalGraph G = graph::build_local_subgraph(mgs, 0);
    std::cout << "Graph Loaded. Vertices: " << G.vertices.size() << std::endl;

    pudqlib::Result res0 = pudqlib::rgn_gradhess_multi_J_forinverse(G);
    double f0 = res0.F_X;
    Eigen::VectorXd grad = res0.rgrad_X;
    Eigen::VectorXd x0 = pudqlib::G_get_X(G); 
    std::cout << "Initial Cost f(x): " << f0 << std::endl;
    std::cout << "Gradient Norm: " << pudqlib::riemannian_norm(G, grad, 0) << std::endl;

    Eigen::VectorXd eta = generate_random_tangent(G);
    double slope = pudqlib::riemannian_dot(G, grad, eta, 0);
    
    std::cout << "\nStarting Gradient Check..." << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << std::setw(10) << "Step (h)" 
              << std::setw(20) << "Error" 
              << std::setw(20) << "Error / h^2" 
              << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;

    double h = 1.0; 
    for (int k = 0; k < 15; ++k) {
        
        Eigen::VectorXd step_vec = h * eta;
        Eigen::VectorXd x_new_vec = pudqlib::Exp_X_N(x0, step_vec);

        pudqlib::LocalGraph G_new = G;
        pudqlib::G_set_X(G_new, x_new_vec); 
        pudqlib::Result res_new = pudqlib::rgn_gradhess_multi_J_forinverse(G_new);
        double f_new = res_new.F_X;

        double f_approx = f0 + h * slope;
        double error = std::abs(f_new - f_approx);

        std::cout << std::scientific << std::setprecision(5) 
                  << std::setw(10) << h 
                  << std::setw(20) << error 
                  << std::setw(20) << (error / (h*h)) 
                  << std::endl;

        h *= 0.1;
    }
    
    return 0;
}