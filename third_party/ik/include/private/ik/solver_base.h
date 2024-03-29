/* WARNING: This file was auto-generated by /home/kang/monster_mash/third_party/IK/ik/ik_gen_vtable.py */
#ifndef IK_VTABLE_SOLVER_BASE_H
#define IK_VTABLE_SOLVER_BASE_H

#include "ik/config.h"

C_BEGIN

#include "ik/solver.h"

IK_PRIVATE_API uintptr_t ik_solver_base_type_size(void);
IK_PRIVATE_API struct ik_solver_t* ik_solver_base_create(enum ik_algorithm_e algorithm);
IK_PRIVATE_API void ik_solver_base_destroy(struct ik_solver_t* solver);
IK_PRIVATE_API ikret_t ik_solver_base_construct(struct ik_solver_t* solver);
IK_PRIVATE_API void ik_solver_base_destruct(struct ik_solver_t* solver);
IK_PRIVATE_API ikret_t ik_solver_base_rebuild(struct ik_solver_t* solver);
IK_PRIVATE_API void ik_solver_base_update_distances(struct ik_solver_t* solver);
IK_PRIVATE_API ikret_t ik_solver_base_solve(struct ik_solver_t* solver);
IK_PRIVATE_API void ik_solver_base_set_tree(struct ik_solver_t* solver, struct ik_node_t* base);
IK_PRIVATE_API struct ik_node_t* ik_solver_base_unlink_tree(struct ik_solver_t* solver);
IK_PRIVATE_API void ik_solver_base_destroy_tree(struct ik_solver_t* solver);
IK_PRIVATE_API void ik_solver_base_iterate_all_nodes(struct ik_solver_t* solver, ik_solver_iterate_node_cb_func callback);
IK_PRIVATE_API void ik_solver_base_iterate_affected_nodes(struct ik_solver_t* solver, ik_solver_iterate_node_cb_func callback);
IK_PRIVATE_API void ik_solver_base_iterate_base_nodes(struct ik_solver_t* solver, ik_solver_iterate_node_cb_func callback);
#define IK_SOLVER_BASE_IMPL \
    ik_solver_base_type_size, \
    ik_solver_base_create, \
    ik_solver_base_destroy, \
    ik_solver_base_construct, \
    ik_solver_base_destruct, \
    ik_solver_base_rebuild, \
    ik_solver_base_update_distances, \
    ik_solver_base_solve, \
    ik_solver_base_set_tree, \
    ik_solver_base_unlink_tree, \
    ik_solver_base_destroy_tree, \
    ik_solver_base_iterate_all_nodes, \
    ik_solver_base_iterate_affected_nodes, \
    ik_solver_base_iterate_base_nodes
















C_END

#endif /* IK_VTABLE_SOLVER_BASE_H */
