include "cartographer_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
POSE_GRAPH.optimize_every_n_nodes = 20
-- output map to base_link for evaluation
-- options.provide_odom_frame = false
-- POSE_GRAPH.optimization_problem.log_solver_summary = true

-- fast localization
-- MAP_BUILDER.num_background_threads = 12
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.5 * POSE_GRAPH.constraint_builder.sampling_ratio
-- POSE_GRAPH.global_sampling_ratio = 0.1 * POSE_GRAPH.global_sampling_ratio
-- POSE_GRAPH.max_num_final_iterations = 1

return options
