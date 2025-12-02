#include "pudq_lib.hpp"
#include "graph.hpp"
#include "optimization.hpp"

#include <iostream>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <sstream>

// Helper to load the exported text files back into memory
std::vector<pudqlib::PUDQ> loadVerticesFromFile(const std::string& filename) {
    std::vector<pudqlib::PUDQ> vertices;
    std::ifstream file(filename);
    if (!file.is_open()) throw std::runtime_error("Cannot open: " + filename);
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        pudqlib::PUDQ q;
        if (iss >> q(0) >> q(1) >> q(2) >> q(3)) {
            vertices.push_back(q);
        }
    }
    return vertices;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: ./evaluate_results <num_robots> <graph_dir>" << std::endl;
        return 1;
    }

    int num_robots = std::stoi(argv[1]);
    std::string graph_dir = argv[2];

    try {
        // ---------------------------------------------------------
        // 1. Load the Original Global Graph Structure
        // ---------------------------------------------------------
        std::cout << "Loading original graph structure from: " << graph_dir << std::endl;
        pudqlib::Graph global_graph = graph::load_graph_from_matlab_export(graph_dir);
        
        // ---------------------------------------------------------
        // 2. Distribute (to replicate ownership logic)
        // ---------------------------------------------------------
        std::cout << "Maping robot ownership..." << std::endl;
        std::vector<pudqlib::MultiGraph> multi_graphs = graph::distributebiggraph_withinverseedge(global_graph, num_robots);

        // ---------------------------------------------------------
        // 3. Load & Merge Distributed Results
        // ---------------------------------------------------------
        std::cout << "Merging distributed results from 'cpp_export_ros/'..." << std::endl;
        
        for (int r = 0; r < num_robots; ++r) {
            std::string file_path = "cpp_export_ros/robot_" + std::to_string(r) + "/vertices_pudq.txt";
            
            if (!std::filesystem::exists(file_path)) {
                std::cerr << "WARNING: Missing result file for Robot " << r << " (" << file_path << ")" << std::endl;
                continue;
            }

            std::vector<pudqlib::PUDQ> local_optimized = loadVerticesFromFile(file_path);
            
            // Get the global indices that this robot owns
            pudqlib::LocalGraph temp_lg = graph::build_local_subgraph(multi_graphs, r);
            const std::vector<int>& global_ids = temp_lg.vertices_interval;

            if (local_optimized.size() != global_ids.size()) {
                std::cerr << "ERROR: Size mismatch for Robot " << r << ". Expected " << global_ids.size() 
                          << ", got " << local_optimized.size() << std::endl;
                continue;
            }

            // Update Global Graph
            for (size_t i = 0; i < global_ids.size(); ++i) {
                int gid = global_ids[i];
                global_graph.vertices_pudq[gid] = local_optimized[i];
                // Sync Vector3d pose
                global_graph.vertices[gid] = pudqlib::pudq_to_pose(local_optimized[i]);
            }
        }
        std::cout << "Global graph merged successfully." << std::endl;

        // ---------------------------------------------------------
        // 4. Global Optimization Metrics (Cost/Grad)
        // ---------------------------------------------------------
        std::cout << "\n--- Global Optimization Stats ---" << std::endl;
        const pudqlib::Result result_opt = pudqlib::rgn_gradhess_J(global_graph);
        const double& F_opt = result_opt.F_X;
        const Eigen::VectorXd& rgrad_opt = result_opt.rgrad_X;
        const double grad_norm = rgrad_opt.norm();
        
        std::printf("Final Cost      = %.6g\n", F_opt);
        std::printf("Final Grad Norm = %.6g\n", grad_norm);

        // ---------------------------------------------------------
        // 5. Trajectory Error Metrics (ATE / RPE)
        // ---------------------------------------------------------
        std::cout << "\n--- Trajectory Metrics ---" << std::endl;

        graph::reorient_graph_to_identity(global_graph); 

        graph::calculate_ATE_Euclidean(global_graph);
        graph::calculate_RPE_Euclidean(global_graph);
        graph::calculate_ATE_PUDQ(global_graph);
        graph::calculate_RPE_PUDQ(global_graph);

        std::cout << "--------------------------" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}