#!/usr/bin/env python

import opengen as og
import casadi.casadi as cs
import math
import numpy as np
import pathlib
import os

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 6)
p = cs.SX.sym("p", 53)

m_tr = u[0]
d_tr = u[1]
a = u[2]
delta = cs.SX.sym('delta',3)
for i in range(3):
    delta[i] = p[3+i]

J_pinv = cs.SX.sym('J_pinv',7,3)
for i in range(7):
    for j in range(3):
        J_pinv[i,j] = p[3*i+j]
                
F = cs.SX.sym('F',3)
for i in range(3):
    F[i] = p[21+i]
        
v_prev = cs.SX.sym('v_prev',3)
for i in range(3):
    v_prev[i] = p[24+i]
    
b = cs.SX.sym('b',7)
for i in range(7):
    b[i] = p[27+i]
    
dq_safe_max = cs.SX.sym('dq_safe_max',7)
for i in range(7):
    dq_safe_max[i] = p[34+i]
    
dq_safe_min = cs.SX.sym('dq_safe_min',7)
for i in range(7):
    dq_safe_min[i] = p[41+i]
    
m_tr_0 = p[48]
d_tr_0 = p[49]
k_1 = p[50]
k_2 = p[51]
k_3 = p[52]

#cost = k_1*pow(m_tr-m_tr_0,2)+k_1*pow(d_tr-d_tr_0,2)+k_2*pow(a,2)+k_3*np.dot(delta[:,0],delta[:,0])
cost = k_1*cs.constpow(m_tr-m_tr_0,2)+k_1*cs.constpow(d_tr-d_tr_0,2)+k_2*cs.constpow(a,2)+k_3*(delta.T@delta)

T = 0.001
v_adm_lim = cs.SX.sym('v_adm_lim',3)
for i in range(3):
    v_adm_lim[i] = (m_tr*v_prev[i]+F[i]*T)/(m_tr+d_tr*T)
        
dq_c_lim = J_pinv@(v_adm_lim+delta)+b*a

cons = cs.vertcat(cs.fmax(0,dq_c_lim[0]-dq_safe_max[0]),
                  cs.fmax(0,dq_c_lim[1]-dq_safe_max[1]),
                  cs.fmax(0,dq_c_lim[2]-dq_safe_max[2]),
                  cs.fmax(0,dq_c_lim[3]-dq_safe_max[3]),
                  cs.fmax(0,dq_c_lim[4]-dq_safe_max[4]),
                  cs.fmax(0,dq_c_lim[5]-dq_safe_max[5]),
                  cs.fmax(0,dq_c_lim[6]-dq_safe_max[6]),
                  cs.fmax(0,dq_safe_min[0]-dq_c_lim[0]),
                  cs.fmax(0,dq_safe_min[1]-dq_c_lim[1]),
                  cs.fmax(0,dq_safe_min[2]-dq_c_lim[2]),
                  cs.fmax(0,dq_safe_min[3]-dq_c_lim[3]),
                  cs.fmax(0,dq_safe_min[4]-dq_c_lim[4]),
                  cs.fmax(0,dq_safe_min[5]-dq_c_lim[5]),
                  cs.fmax(0,dq_safe_min[6]-dq_c_lim[6]),
                  cs.fmax(0,m_tr_0-m_tr),
                  cs.fmax(0,d_tr_0-d_tr))

problem = og.builder.Problem(u, p, cost) \
    .with_penalty_constraints(cons)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting
script_directory = pathlib.Path(__file__).parent.parent.resolve()
os.chdir(script_directory)
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("optimizer6")
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1000)                     \
.with_penalty_weight_update_factor(5)               \
    .with_max_outer_iterations(100)                 \
    .with_max_duration_micros(500)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
#builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          #build_config)
builder.build()
