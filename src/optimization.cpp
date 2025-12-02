#include "optimization.hpp"

#include <cstdio>
#include <vector>
#include <stdexcept>
#include <Eigen/SparseCholesky>
// #include "OoqpEigenInterface.hpp"

namespace opt{

/*     std::tuple<pudqlib::LocalGraph, double, double> Optimization_RGN_linesearch_inverse_edge(pudqlib::LocalGraph& local_graph, double alpha_init, double grad_tol, double lambda_reg){
        const double tau = 1e-4;
        const double beta = 0.5;
        bool         found = false;
        int          l = 0;
        double       alpha = alpha_init;

        const int num_vertices = static_cast<int>(local_graph.vertices.size());
        const bool anchor_first_vertex = local_graph.anchor_first && (local_graph.robot_id == 0);

        const int total_dim = 4 * num_vertices;
        const int offset = anchor_first_vertex ? 4 : 0;
        const int dim = total_dim - offset;

        pudqlib::Result result0 = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph);
        const Eigen::VectorXd& rgrad = result0.rgrad_X;
        const Eigen::SparseMatrix<double>& rgnhess_X= result0.rgnhess_X;
        double F = result0.F_X;

        const double grad_norm = rgrad.segment(offset, dim).norm(); // check segment / last index correct or not
        if (grad_norm < grad_tol){
            const double F_now = F;
            const double norm_grad_F = grad_norm;
            std::printf("[LineSearch] grad norm below threshold -> no update.\n");
            return {local_graph, F_now, norm_grad_F};
        }

        // const Eigen::MatrixXd H_k = rgnhess_X.block(offset, offset, dim, dim); // check block function
        const Eigen::VectorXd g_k = rgrad.segment(offset, dim);  // check segment

        Eigen::SparseMatrix<double> H_k_reg = H_k;
        H_k_reg.diagonal().array() += lambda_reg;  // check if this is same as matlab H_k + lambda_reg * eye(size(H_k));
        // Eigen:: VectorXd s_k_trunc = H_k_reg.ldlt().solve(-g_k); // check id ldlt is same as matlab  - H_k_reg \ g_k;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(H_k_reg);
        if(solver.info() != Eigen::Success) {
            std::fprintf(stderr, "WARNING: Sparse factorization failed!\n");
            //加個錯誤警告
        }
        Eigen::VectorXd s_k_trunc = solver.solve(-g_k);

        Eigen::VectorXd s_k(total_dim); // vectorxd() 括號裡的東西是什麼 以及dim對不對
        if (anchor_first_vertex){
            s_k.setZero();
            s_k.segment(offset, dim) = s_k_trunc; // check segment
        } else{
            s_k = s_k_trunc;
        }

        const double dd = g_k.dot(s_k_trunc); // check dot 這樣使用對不對

        while (!found){
            const double pow_beta_l = std::pow(beta, static_cast<double>(l));
            const double scale = pow_beta_l * alpha;

            const Eigen::VectorXd Exp_s_can = pudqlib::Exp_X_N(pudqlib::G_get_X(local_graph), scale * s_k);
            const pudqlib::LocalGraph local_graph_candidate = pudqlib::G_set_X(local_graph, Exp_s_can);
            const pudqlib::Result result_can = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph_candidate);
            double F_can = result_can.F_X;

            if (F_can <= F + tau * scale * dd){
                alpha = scale;
                break;
            } else {
                ++l;
                if (scale < 1e-15){
                    std::fprintf(stderr, "Armijo line search alpha is too small, breaking early.\n");
                    break;
                }
            }
        }

        const Eigen::VectorXd s_k_final = alpha * s_k;
        const Eigen::VectorXd Exp_s = pudqlib::Exp_X_N(pudqlib::G_get_X(local_graph), s_k_final);
        local_graph = pudqlib::G_set_X(local_graph, Exp_s);

        const pudqlib::Result result_final = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph);
        const double norm_grad_F = result_final.rgrad_X.segment(offset, dim).norm();
        double F_now = result_final.F_X;

        return {local_graph, F_now, norm_grad_F};
    } */

    std::tuple<double, double> Optimization_RLM_inverse_edge(pudqlib::LocalGraph& local_graph, double mu_in, double grad_tol) {
        
        // --- RLM Parameters ---
        const double eta = 0.15;
        const double beta = 5.0;
        const int    max_lm_trials = 300;
        // ----------------------

        const int num_vertices = static_cast<int>(local_graph.vertices.size());
        const bool anchor_first_vertex = local_graph.anchor_first && (local_graph.robot_id == 0);

        const int total_dim = 4 * num_vertices;
        const int offset = anchor_first_vertex ? 4 : 0;
        const int dim = total_dim - offset;

        pudqlib::Result result0 = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph);
        const Eigen::VectorXd& rgrad = result0.rgrad_X;
        const Eigen::SparseMatrix<double>& rgnhess = result0.rgnhess_X; 
        const double F_initial = result0.F_X;

        const double grad_norm = rgrad.segment(offset, dim).norm();
        if (grad_norm < grad_tol) {
            std::printf(" Grad norm below threshold -> no update.\n");
            return {F_initial, grad_norm};
        }

        const Eigen::SparseMatrix<double> H_k = rgnhess.block(offset, offset, dim, dim);
        const Eigen::VectorXd g_k = rgrad.segment(offset, dim);
        const double normF2 = 2.0 * F_initial;
        double lambda = mu_in * normF2;

        for (int lm_trial = 0; lm_trial < max_lm_trials; ++lm_trial) {
            Eigen::SparseMatrix<double> H_k_reg = H_k;
            H_k_reg.diagonal().array() += lambda;

            // Eigen::SparseMatrix<double> H_k_reg_full = rgnhess;
            // H_k_reg_full.diagonal().array() += lambda;

            // Eigen::VectorXd s_k_trunc;
            // ooqpei::OoqpEigenInterface::solve(H_k_reg, g_k, s_k_trunc);
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(H_k_reg);
            if(solver.info() != Eigen::Success) {
                std::fprintf(stderr, "WARNING: RLM Sparse factorization failed!\n");
                continue; 
            }
            Eigen::VectorXd s_k_trunc = solver.solve(-g_k);

            Eigen::VectorXd s_k(total_dim);
            if (anchor_first_vertex) {
                s_k.setZero();
                s_k.segment(offset, dim) = s_k_trunc;
            } else {
                s_k = s_k_trunc;
            }

            const double pred_reduction = -(g_k.dot(s_k_trunc) + 0.5 * s_k_trunc.dot(H_k_reg * s_k_trunc));

            pudqlib::LocalGraph graph_candidate = local_graph;
            const Eigen::VectorXd X_candidate = pudqlib::Exp_X_N(pudqlib::G_get_X(graph_candidate), s_k);
            pudqlib::G_set_X(graph_candidate, X_candidate); 

            const pudqlib::Result result_can = pudqlib::rgn_gradhess_multi_J_forinverse(graph_candidate);
            const double F_candidate = result_can.F_X;
            const double actual_reduction = F_initial - F_candidate;

            double rho = -std::numeric_limits<double>::infinity();
            if (pred_reduction > 0) {
                rho = actual_reduction / pred_reduction;
            }

            if (rho > eta) {
                local_graph = graph_candidate; 

                const pudqlib::Result result_final = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph);
                const double norm_grad_F = result_final.rgrad_X.segment(offset, dim).norm();
                
                return {result_final.F_X, norm_grad_F};
            } else {
                mu_in = mu_in * beta;
            }
        }

        std::fprintf(stderr, "RLM failed to find a successful step after %d trials. \n", max_lm_trials);
        return {F_initial, grad_norm};
    }

    void shareupdateinfo(int share_robot_id, const pudqlib::LocalGraph& share_subgraph, std::vector<pudqlib::Robot>& robotList){
        const int num_robots = static_cast<int>(robotList.size());

        for (int receive_robot_id =0; receive_robot_id < num_robots; ++receive_robot_id){
            if (receive_robot_id == share_robot_id){
                continue;
            }

            pudqlib::LocalGraph& receive_robot_graph = robotList[receive_robot_id].local_graph;
            // int update_num = 0;

            for (int L = 0; L < static_cast<int>(receive_robot_graph.lm_vertices.size()); ++L){
                const pudqlib::ForeignInfo& landmark_info = receive_robot_graph.lm_foreign_info[L];

                if (landmark_info.robot == share_robot_id){
                    const int update_local_idx = landmark_info.local_index;

                    if (update_local_idx < static_cast<int>(share_subgraph.vertices.size())){
                        const Eigen::Vector3d& old_pose = receive_robot_graph.lm_vertices[L];
                        const Eigen::Vector3d& new_pose = share_subgraph.vertices[update_local_idx];
                        const pudqlib::PUDQ& new_pose_pudq = share_subgraph.vertices_pudq[update_local_idx];
                        
                        receive_robot_graph.lm_vertices[L] = new_pose;
                        receive_robot_graph.lm_vertices_pudq[L] = new_pose_pudq;
                        // update_num++;

                        // fprintf(" Robot %d: Updated landmark %d (R%d-V%d): [%.6f,%.6f,%.6f] -> [%.6f,%.6f,%.6f]\n",
                        //         receive_robot_id, L, share_robot_id, update_local_idx,
                        //         old_pose(0), old_pose(1), old_pose(2),
                        //         new_pose(0), new_pose(1), new_pose(2));
                    } else{
                         throw std::runtime_error("Invalid vertex index " + std::to_string(update_local_idx) + " for robot " + std::to_string(share_robot_id) + " (max: " + std::to_string(share_subgraph.vertices.size()) + ")");
                    }
                }
            }

            // if (update_num > 0) {
            // fprintf(" Robot %d: Applied %d updates from Robot %d\n", receive_robot_id, update_num, share_robot_id);
            // }
        }
        // fprintf("[SHARING] Robot %d update propagation complete\n", share_robot_id);
    }

    void check_all_robotgrad_inverse_edge(std::vector<pudqlib::Robot>& robotList, double grad_tol){
        // change to void in the future
        const std::size_t num_robots = robotList.size();

        for (std::size_t r = 0; r < num_robots; ++r){
            pudqlib::LocalGraph& local_graph_r = robotList[r].local_graph; 
            
            const bool anchor_first_vertex = local_graph_r.anchor_first && (local_graph_r.robot_id == 0);
            const int num_vertices = static_cast<int>(local_graph_r.vertices.size());
            const int total_dim = 4 * num_vertices;
            const int offset = anchor_first_vertex ? 4 : 0;
            const int dim = total_dim - offset;

            const pudqlib::Result result = pudqlib::rgn_gradhess_multi_J_forinverse(local_graph_r);
            const double norm_grad_F = result.rgrad_X.segment(offset, dim).norm(); //check segment

            if (norm_grad_F < grad_tol){
                robotList[r].converge = true;
                std::printf("Robot %zu converged (grad=%.3g)\n", r, norm_grad_F);
            } else{
                robotList[r].converge = false;
                std::printf("Robot %zu not converged (grad=%.3g)\n", r, norm_grad_F);
            }
        }
    }
    
}
