/* WARNING: This file was auto-generated by /home/kang/monster_mash/third_party/IK/ik/ik_gen_vtable.py */
#ifndef IK_VTABLE_LOG_STATIC_H
#define IK_VTABLE_LOG_STATIC_H

#include "ik/config.h"

C_BEGIN

#include "ik/log.h"

IK_PRIVATE_API ikret_t ik_log_static_init(void);
IK_PRIVATE_API void ik_log_static_deinit(void);
IK_PRIVATE_API void ik_log_static_set_severity(enum ik_log_severity_e severity);
IK_PRIVATE_API void ik_log_static_message(const char* fmt, ...);
#define IK_LOG_STATIC_IMPL \
    ik_log_static_init, \
    ik_log_static_deinit, \
    ik_log_static_set_severity, \
    ik_log_static_message






C_END

#endif /* IK_VTABLE_LOG_STATIC_H */