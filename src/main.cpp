#include "pudq_lib.hpp"
#include "graph.hpp"  
#include "optimization.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cstdio>

int main(){
    // Parameters
    const int num_robots = 5;
    const double lambda = 1.0;
    const double alpha_init = 1e-1;
    const double grad_tol = 1e-2;
    //const double lambda_reg = 1e-2;
    const double mu_reg = 1e-1;

    // const std::string gt_g2o_file = "C:/Users/tcche/OneDrive/桌面/2D_distributedPGO_Sandy/2D_distributedPGO_Sandy/data/Grid1000_ground_truth.g2o";
    // const std::string odom_g2o_file = "C:/Users/tcche/OneDrive/桌面/2D_distributedPGO_Sandy/2D_distributedPGO_Sandy/data/Grid1000_1.g2o";
    
    // std::cout << "Loading ground truth graph..." << std::endl;
    // pudqlib::Graph GT_graph = data::load_g2o_file_pudq_gt(gt_g2o_file);

    // std::cout << "Loading odometry graph..." << std::endl;
    // // pudqlib::Graph odom_graph = data::load_g2o_file_pudq(odom_g2o_file); // for normal g2o file
    // pudqlib::Graph odom_graph = data::load_g2o_file_pudq_V2(odom_g2o_file);

    // std::cout << "Combining graphs..." << std::endl;
    // pudqlib::Graph graph = data::combine_gt_odom_g2o(odom_graph, GT_graph);
    // pudqlib::Graph graph_RGN = graph; // 想想這個有沒有必要

    const std::string matlab_export_dir = "/home/hsuanpin/Desktop/2D_distributedPGO_Sandy/temp/matlab_export_c";

    std::cout << "Loading graph from MATLAB export..." << std::endl;
    pudqlib::Graph graph = graph::load_graph_from_matlab_export(matlab_export_dir);

    const int num_vertices = static_cast<int>(graph.vertices.size());
    std::cout << "Number of vertices: " << num_vertices << std::endl;
    
    // Add in chordal initialization later

    std::cout << "Distributing graph among " << num_robots << " robots..." << std::endl;
    std::vector<pudqlib::MultiGraph> multi_graph_RGN = graph::distributebiggraph_withinverseedge(graph, num_robots);

    std::vector<pudqlib::Robot> robots(num_robots);
    for (int r = 0; r < num_robots; ++r){
        pudqlib::LocalGraph local_graph = graph::build_local_subgraph(multi_graph_RGN, r);

        pudqlib::Result result = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph);
        double F_0 = result.F_X;

        const bool anchor_first_vertex = local_graph.anchor_first && (local_graph.robot_id == 0);
        const int num_local_vertices = static_cast<int>(local_graph.vertices.size());
        const int total_dim = 4 * num_local_vertices;
        const int offset = anchor_first_vertex ? 4 : 0;
        const int dim = total_dim - offset;

        double gradnorm_0 = result.rgrad_X.segment(offset, dim).norm();

        robots[r].id = r;
        robots[r].local_graph = local_graph;
        robots[r].rgn_cost.push_back(F_0);
        robots[r].rgn_gradnorm.push_back(gradnorm_0);
        robots[r].converge = false;

        std::printf("Robot %d initialized: F=%.6g, grad_norm=%.6g\n", r, F_0, gradnorm_0);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::exponential_distribution<double> exp_dist(lambda);
    
    // Initialize robot times
    std::vector<double> robot_time(num_robots);
    double global_time = 0.0;
    
    for (int r = 0; r < num_robots; ++r) {
        robot_time[r] = exp_dist(gen);
    }

    int iter = 0;
    std::cout << "\nStarting asynchronous optimization..." << std::endl;

    while (true){
        ++iter;

        auto min_it = std::min_element(robot_time.begin(), robot_time.end());
        int r = static_cast<int>(std::distance(robot_time.begin(), min_it));
        global_time = *min_it;

        auto [F_now, norm_grad_F] = opt::Optimization_RLM_inverse_edge(robots[r].local_graph, mu_reg, grad_tol);
        robots[r].rgn_cost.push_back(F_now);
        robots[r].rgn_gradnorm.push_back(norm_grad_F);
        std::printf("Iter=%3d -> Robot %d updated (F=%.3g, grad=%.3g)\n", iter, r, F_now, norm_grad_F);

        opt::shareupdateinfo(r, robots[r].local_graph, robots);

        robot_time[r] = global_time + exp_dist(gen);
        opt::check_all_robotgrad_inverse_edge(robots, grad_tol);

        bool all_converged = true;
        for (const auto& robot : robots) {
            if (!robot.converge) {
                all_converged = false;
                break;
            }
        }
        
        if (all_converged) {
            std::printf("All %d robots converged -> terminating at iter=%d.\n", num_robots, iter);
            break;
        }
    }
    std::printf("\n*** Done with asynchronous update events! ***\n");
    std::cout << "\nFinal Results:" << std::endl;
    for (int r = 0; r < num_robots; ++r) {
        const auto& robot = robots[r];
        std::printf("Robot %d: Final cost=%.6g, Final grad_norm=%.6g\n", r, robot.rgn_cost.back(), robot.rgn_gradnorm.back());
    }

    std::cout << "\nReconstructing final global graph..." << std::endl;
    pudqlib::Graph graph_optimized = graph::reconstructglobalgraph(robots, graph);
    std::cout << "Global graph reconstruction complete." << std::endl;
    
    const pudqlib::Result result_opt = pudqlib::rgn_gradhess_J(graph_optimized);
    const double& F_opt = result_opt.F_X;
    const Eigen::VectorXd& rgrad_opt = result_opt.rgrad_X;
    const double grad_norm = rgrad_opt.norm();
    std::printf("Cost=%.6g, Gradnorm=%.6g\n", F_opt, grad_norm);

    try {
        const std::string output_dir = "cpp_export_full";
        graph::saveFullAnalysisData(output_dir, graph_optimized, robots);
        std::cout << "Export complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error during export: " << e.what() << std::endl;
    }
    
    return 0;
}