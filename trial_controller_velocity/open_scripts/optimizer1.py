#!/usr/bin/env python

import pathlib
import opengen as og
import casadi.casadi as cs
import math
import numpy
import os

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 50)

d_t = u[0]
a = u[1]
delta_1 = u[2]
delta_2 = u[3]
delta_3 = u[4]

j_11 = p[0]
j_12 = p[1]
j_13 = p[2]

j_21 = p[3]
j_22 = p[4]
j_23 = p[5]

j_31 = p[6]
j_32 = p[7]
j_33 = p[8]

j_41 = p[9]
j_42 = p[10]
j_43 = p[11]

j_51 = p[12]
j_52 = p[13]
j_53 = p[14]

j_61 = p[15]
j_62 = p[16]
j_63 = p[17]

j_71 = p[18]
j_72 = p[19]
j_73 = p[20]

f_1 = p[21]
f_2 = p[22]
f_3 = p[23]

b_1 = p[24]
b_2 = p[25]
b_3 = p[26]
b_4 = p[27]
b_5 = p[28]
b_6 = p[29]
b_7 = p[30]

dq_max1 = p[31]
dq_max2 = p[32]
dq_max3 = p[33]
dq_max4 = p[34]
dq_max5 = p[35]
dq_max6 = p[36]
dq_max7 = p[37]

dq_min1 = p[38]
dq_min2 = p[39]
dq_min3 = p[40]
dq_min4 = p[41]
dq_min5 = p[42]
dq_min6 = p[43]
dq_min7 = p[44]

d_t0 = p[45]
m_h = p[46]
m_l = p[47]
k_1 = p[48]
k_2 = p[49]

cost = cs.if_else(d_t < d_t0, m_h*pow(d_t-d_t0,2), m_l*pow(d_t-d_t0,2),True)
#cost = m_l*pow(d_t-d_t0,2)

cost = cost + k_1*pow(a,2)

squared_sine = k_2*(pow((f_2*(delta_3 + f_3/d_t))/d_t - (f_3*(delta_2 + f_2/d_t))/d_t,2)+pow((f_3*(delta_1 + f_1/d_t))/d_t - (f_1*(delta_3 + f_3/d_t))/d_t,2)+pow((f_1*(delta_2 + f_2/d_t))/d_t - (f_2*(delta_1 + f_1/d_t))/d_t,2)/((pow(f_1/d_t,2)+pow(f_2/d_t,2)+pow(f_3/d_t,2))*(pow(delta_1 + f_1/d_t,2)+pow(delta_2 + f_2/d_t,2)+pow(delta_3 + f_3/d_t,2))))

dot_product = (f_1*(delta_1 + f_1/d_t))/d_t + (f_2*(delta_2 + f_2/d_t))/d_t + (f_3*(delta_3 + f_3/d_t))/d_t

#cost = cost + squared_sine

#cost = cost + k_2*cs.if_else(dot_product > 0, squared_sine, 2-squared_sine,True)
cost = cost + k_2*(pow(delta_1,2)+pow(delta_2,2)+pow(delta_3,2))
#numerator = math.sqrt(pow((f_2*(delta_3 + f_3/d_t))/d_t - (f_3*(delta_2 + f_2/d_t))/d_t,2)+pow((f_3*(delta_1 + f_1/d_t))/d_t - (f_1*(delta_3 + f_3/d_t))/d_t,2)+pow((f_1*(delta_2 + f_2/d_t))/d_t - (f_2*(delta_1 + f_1/d_t))/d_t,2))
#numerator = (f_1*(delta_1 + f_1/d_t))/d_t + (f_2*(delta_2 + f_2/d_t))/d_t + (f_3*(delta_3 + f_3/d_t))/d_t
#denominator = math.sqrt((pow(f_1/d_t,2)+pow(f_2/d_t,2)+pow(f_3/d_t,2))+0.0001)*math.sqrt((pow(delta_1 + f_1/d_t,2)+pow(delta_2 + f_2/d_t,2)+pow(delta_3 + f_3/d_t,2))+0.0001)
#denominator = math.sqrt((pow(delta_1 + f_1/d_t,2)+pow(delta_2 + f_2/d_t,2)+pow(delta_3 + f_3/d_t,2))+0.0001)
#numerator = math.sqrt(pow((-2.14561*(0.0 -0.237886/10))/10 - (-0.237886*(0.0 -2.14561/10))/10,2)+pow((-0.237886*(0.0 -0.367752/10))/10 - (-0.367752*(0.0 -0.237886/10))/10,2)+pow((-0.367752*(0.0 -2.14561/10))/10 - (-2.14561*(0.0 -0.367752/10))/10,2))
#denominator = math.sqrt((pow(-0.367752/10,2)+pow(-2.14561/10,2)+pow(-0.237886/10,2)))+math.sqrt(pow(0.0 -0.367752/10,2)+pow(0.0 -2.14561/10,2)+pow(0.0 -0.237886/10,2))
#theta = numpy.arccos(numerator/denominator)
#print(theta)
#cost = cost+k_2*pow(theta,2)
cons = cs.vertcat(cs.fmax(0.0, j_11*(f_1/d_t+delta_1)+j_12*(f_2/d_t+delta_2)+j_13*(f_3/d_t+delta_3)+b_1*a-dq_max1), 
                  cs.fmax(0.0, -(j_11*(f_1/d_t+delta_1)+j_12*(f_2/d_t+delta_2)+j_13*(f_3/d_t+delta_3)+b_1*a)+dq_min1),
                  cs.fmax(0.0, j_21*(f_1/d_t+delta_1)+j_22*(f_2/d_t+delta_2)+j_23*(f_3/d_t+delta_3)+b_2*a-dq_max2), 
                  cs.fmax(0.0, -(j_21*(f_1/d_t+delta_1)+j_22*(f_2/d_t+delta_2)+j_23*(f_3/d_t+delta_3)+b_2*a)+dq_min2),
                  cs.fmax(0.0, j_31*(f_1/d_t+delta_1)+j_32*(f_2/d_t+delta_2)+j_33*(f_3/d_t+delta_3)+b_3*a-dq_max3), 
                  cs.fmax(0.0, -(j_31*(f_1/d_t+delta_1)+j_32*(f_2/d_t+delta_2)+j_33*(f_3/d_t+delta_3)+b_3*a)+dq_min3),
                  cs.fmax(0.0, j_41*(f_1/d_t+delta_1)+j_42*(f_2/d_t+delta_2)+j_43*(f_3/d_t+delta_3)+b_4*a-dq_max4), 
                  cs.fmax(0.0, -(j_41*(f_1/d_t+delta_1)+j_42*(f_2/d_t+delta_2)+j_43*(f_3/d_t+delta_3)+b_4*a)+dq_min4),
                  cs.fmax(0.0, j_51*(f_1/d_t+delta_1)+j_52*(f_2/d_t+delta_2)+j_53*(f_3/d_t+delta_3)+b_5*a-dq_max5), 
                  cs.fmax(0.0, -(j_51*(f_1/d_t+delta_1)+j_52*(f_2/d_t+delta_2)+j_53*(f_3/d_t+delta_3)+b_5*a)+dq_min5),
                  cs.fmax(0.0, j_61*(f_1/d_t+delta_1)+j_62*(f_2/d_t+delta_2)+j_63*(f_3/d_t+delta_3)+b_6*a-dq_max6), 
                  cs.fmax(0.0, -(j_61*(f_1/d_t+delta_1)+j_62*(f_2/d_t+delta_2)+j_63*(f_3/d_t+delta_3)+b_6*a)+dq_min6),
                  cs.fmax(0.0, j_71*(f_1/d_t+delta_1)+j_72*(f_2/d_t+delta_2)+j_73*(f_3/d_t+delta_3)+b_7*a-dq_max7), 
                  cs.fmax(0.0, -(j_71*(f_1/d_t+delta_1)+j_72*(f_2/d_t+delta_2)+j_73*(f_3/d_t+delta_3)+b_7*a)+dq_min7))

problem = og.builder.Problem(u, p, cost) \
    .with_penalty_constraints(cons)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting
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
#builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          #build_config)
builder.build()
