#!/usr/bin/env python

import pathlib
import opengen as og
import casadi.casadi as cs
import math
import numpy
import os

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 8)
p = cs.SX.sym("p", 73)

s = u[0]
delta_1 = u[1]
delta_2 = u[2]
delta_3 = u[3]
delta_4 = u[4]
delta_5 = u[5]
delta_6 = u[6]
a = u[7]

j_11 = p[0]
j_12 = p[1]
j_13 = p[2]
j_14 = p[3]
j_15 = p[4]
j_16 = p[5]

j_21 = p[6]
j_22 = p[7]
j_23 = p[8]
j_24 = p[9]
j_25 = p[10]
j_26 = p[11]

j_31 = p[12]
j_32 = p[13]
j_33 = p[14]
j_34 = p[15]
j_35 = p[16]
j_36 = p[17]

j_41 = p[18]
j_42 = p[19]
j_43 = p[20]
j_44 = p[21]
j_45 = p[22]
j_46 = p[23]

j_51 = p[24]
j_52 = p[25]
j_53 = p[26]
j_54 = p[27]
j_55 = p[28]
j_56 = p[29]

j_61 = p[30]
j_62 = p[31]
j_63 = p[32]
j_64 = p[33]
j_65 = p[34]
j_66 = p[35]

j_71 = p[36]
j_72 = p[37]
j_73 = p[38]
j_74 = p[39]
j_75 = p[40]
j_76 = p[41]

v_c_1 = p[42]
v_c_2 = p[43]
v_c_3 = p[44]
v_c_4 = p[45]
v_c_5 = p[46]
v_c_6 = p[47]


b_1 = p[48]
b_2 = p[49]
b_3 = p[50]
b_4 = p[51]
b_5 = p[52]
b_6 = p[53]
b_7 = p[54]

dq_max1 = p[55]
dq_max2 = p[56]
dq_max3 = p[57]
dq_max4 = p[58]
dq_max5 = p[59]
dq_max6 = p[60]
dq_max7 = p[61]

dq_min1 = p[62]
dq_min2 = p[63]
dq_min3 = p[64]
dq_min4 = p[65]
dq_min5 = p[66]
dq_min6 = p[67]
dq_min7 = p[68]

m_h = p[69]
m_l = p[70]
k_1 = p[71]
k_2 = p[72]

cost = cs.if_else(s < 1, m_h*pow(s-1,2), m_l*pow(s-1,2),True) + k_1*(pow(delta_1,2)+pow(delta_2,2)+pow(delta_3,2)+pow(delta_4,2)+pow(delta_5,2)+pow(delta_6,2)) + k_2*pow(a,2)

cons = cs.vertcat(cs.fmax(0.0, j_11*(v_c_1/s+delta_1)+j_12*(v_c_2/s+delta_2)+j_13*(v_c_3/s+delta_3)+j_14*(v_c_4/s+delta_4)+j_15*(v_c_5/s+delta_5)+j_16*(v_c_6/s+delta_6)+b_1*a-dq_max1), 
                  cs.fmax(0.0, -(j_11*(v_c_1/s+delta_1)+j_12*(v_c_2/s+delta_2)+j_13*(v_c_3/s+delta_3)+j_14*(v_c_4/s+delta_4)+j_15*(v_c_5/s+delta_5)+j_16*(v_c_6/s+delta_6)+b_1*a)+dq_min1),
                  cs.fmax(0.0, j_21*(v_c_1/s+delta_1)+j_22*(v_c_2/s+delta_2)+j_23*(v_c_3/s+delta_3)+j_24*(v_c_4/s+delta_4)+j_25*(v_c_5/s+delta_5)+j_26*(v_c_6/s+delta_6)+b_2*a-dq_max2), 
                  cs.fmax(0.0, -(j_21*(v_c_1/s+delta_1)+j_22*(v_c_2/s+delta_2)+j_23*(v_c_3/s+delta_3)+j_24*(v_c_4/s+delta_4)+j_25*(v_c_5/s+delta_5)+j_26*(v_c_6/s+delta_6)+b_2*a)+dq_min2),
                  cs.fmax(0.0, j_31*(v_c_1/s+delta_1)+j_32*(v_c_2/s+delta_2)+j_33*(v_c_3/s+delta_3)+j_34*(v_c_4/s+delta_4)+j_35*(v_c_5/s+delta_5)+j_36*(v_c_6/s+delta_6)+b_3*a-dq_max3), 
                  cs.fmax(0.0, -(j_31*(v_c_1/s+delta_1)+j_32*(v_c_2/s+delta_2)+j_33*(v_c_3/s+delta_3)+j_34*(v_c_4/s+delta_4)+j_35*(v_c_5/s+delta_5)+j_36*(v_c_6/s+delta_6)+b_3*a)+dq_min3),
                  cs.fmax(0.0, j_41*(v_c_1/s+delta_1)+j_42*(v_c_2/s+delta_2)+j_43*(v_c_3/s+delta_3)+j_44*(v_c_4/s+delta_4)+j_45*(v_c_5/s+delta_5)+j_46*(v_c_6/s+delta_6)+b_4*a-dq_max4), 
                  cs.fmax(0.0, -(j_41*(v_c_1/s+delta_1)+j_42*(v_c_2/s+delta_2)+j_43*(v_c_3/s+delta_3)+j_44*(v_c_4/s+delta_4)+j_45*(v_c_5/s+delta_5)+j_46*(v_c_6/s+delta_6)+b_4*a)+dq_min4),
                  cs.fmax(0.0, j_51*(v_c_1/s+delta_1)+j_52*(v_c_2/s+delta_2)+j_53*(v_c_3/s+delta_3)+j_54*(v_c_4/s+delta_4)+j_55*(v_c_5/s+delta_5)+j_56*(v_c_6/s+delta_6)+b_5*a-dq_max5), 
                  cs.fmax(0.0, -(j_51*(v_c_1/s+delta_1)+j_52*(v_c_2/s+delta_2)+j_53*(v_c_3/s+delta_3)+j_54*(v_c_4/s+delta_4)+j_55*(v_c_5/s+delta_5)+j_56*(v_c_6/s+delta_6)+b_5*a)+dq_min5),
                  cs.fmax(0.0, j_61*(v_c_1/s+delta_1)+j_62*(v_c_2/s+delta_2)+j_63*(v_c_3/s+delta_3)+j_64*(v_c_4/s+delta_4)+j_65*(v_c_5/s+delta_5)+j_66*(v_c_6/s+delta_6)+b_6*a-dq_max6), 
                  cs.fmax(0.0, -(j_61*(v_c_1/s+delta_1)+j_62*(v_c_2/s+delta_2)+j_63*(v_c_3/s+delta_3)+j_64*(v_c_4/s+delta_4)+j_65*(v_c_5/s+delta_5)+j_66*(v_c_6/s+delta_6)+b_6*a)+dq_min6),
                  cs.fmax(0.0, j_71*(v_c_1/s+delta_1)+j_72*(v_c_2/s+delta_2)+j_73*(v_c_3/s+delta_3)+j_74*(v_c_4/s+delta_4)+j_75*(v_c_5/s+delta_5)+j_76*(v_c_6/s+delta_6)+b_7*a-dq_max7), 
                  cs.fmax(0.0, -(j_71*(v_c_1/s+delta_1)+j_72*(v_c_2/s+delta_2)+j_73*(v_c_3/s+delta_3)+j_74*(v_c_4/s+delta_4)+j_75*(v_c_5/s+delta_5)+j_76*(v_c_6/s+delta_6)+b_7*a)+dq_min7))

problem = og.builder.Problem(u, p, cost) \
    .with_penalty_constraints(cons)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()
script_directory = pathlib.Path(__file__).parent.parent.resolve()
os.chdir(script_directory)
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name('optimizer1')
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1000)                     \
.with_penalty_weight_update_factor(5)               \
    .with_max_outer_iterations(100)                 \
    .with_max_duration_micros(500)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()
