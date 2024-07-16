#!/usr/bin/env python

import opengen as og
import casadi.casadi as cs
import math
import numpy
import pathlib
import os

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 8)
p = cs.SX.sym("p", 75)

d_t = u[0]
#d_r = u[1]
a = u[1]
delta_1 = u[2]
delta_2 = u[3]
delta_3 = u[4]
delta_4 = u[5]
delta_5 = u[6]
delta_6 = u[7]


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

f_1 = p[42]
f_2 = p[43]
f_3 = p[44]
f_4 = p[45]
f_5 = p[46]
f_6 = p[47]

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

d_t0 = p[69]
d_r0 = p[70]
m_h = p[71]
m_l = p[72]
k_1 = p[73]
k_2 = p[74]

cost = cs.if_else(d_t < d_t0, m_h*pow(d_t-d_t0,2), m_l*pow(d_t-d_t0,2),True)
#cost = cost+cs.if_else(d_r < d_r0, m_h*pow(d_r-d_r0,2), m_l*pow(d_r-d_r0,2),True)
#cost = m_l*pow(d_t-d_t0,2)

cost = cost + k_1*pow(a,2)

squared_sine = k_2*(pow((f_2*(delta_3 + f_3/d_t))/d_t - (f_3*(delta_2 + f_2/d_t))/d_t,2)+pow((f_3*(delta_1 + f_1/d_t))/d_t - (f_1*(delta_3 + f_3/d_t))/d_t,2)+pow((f_1*(delta_2 + f_2/d_t))/d_t - (f_2*(delta_1 + f_1/d_t))/d_t,2)/((pow(f_1/d_t,2)+pow(f_2/d_t,2)+pow(f_3/d_t,2))*(pow(delta_1 + f_1/d_t,2)+pow(delta_2 + f_2/d_t,2)+pow(delta_3 + f_3/d_t,2))))

dot_product = (f_1*(delta_1 + f_1/d_t))/d_t + (f_2*(delta_2 + f_2/d_t))/d_t + (f_3*(delta_3 + f_3/d_t))/d_t

#cost = cost + squared_sine

#cost = cost + k_2*cs.if_else(dot_product > 0, squared_sine, 2-squared_sine,True)
cost = cost + k_2*(pow(delta_1,2)+pow(delta_2,2)+pow(delta_3,2)+pow(delta_4,2)+pow(delta_5,2)+pow(delta_6,2))
#numerator = math.sqrt(pow((f_2*(delta_3 + f_3/d_t))/d_t - (f_3*(delta_2 + f_2/d_t))/d_t,2)+pow((f_3*(delta_1 + f_1/d_t))/d_t - (f_1*(delta_3 + f_3/d_t))/d_t,2)+pow((f_1*(delta_2 + f_2/d_t))/d_t - (f_2*(delta_1 + f_1/d_t))/d_t,2))
#numerator = (f_1*(delta_1 + f_1/d_t))/d_t + (f_2*(delta_2 + f_2/d_t))/d_t + (f_3*(delta_3 + f_3/d_t))/d_t
#denominator = math.sqrt((pow(f_1/d_t,2)+pow(f_2/d_t,2)+pow(f_3/d_t,2))+0.0001)*math.sqrt((pow(delta_1 + f_1/d_t,2)+pow(delta_2 + f_2/d_t,2)+pow(delta_3 + f_3/d_t,2))+0.0001)
#denominator = math.sqrt((pow(delta_1 + f_1/d_t,2)+pow(delta_2 + f_2/d_t,2)+pow(delta_3 + f_3/d_t,2))+0.0001)
#numerator = math.sqrt(pow((-2.14561*(0.0 -0.237886/10))/10 - (-0.237886*(0.0 -2.14561/10))/10,2)+pow((-0.237886*(0.0 -0.367752/10))/10 - (-0.367752*(0.0 -0.237886/10))/10,2)+pow((-0.367752*(0.0 -2.14561/10))/10 - (-2.14561*(0.0 -0.367752/10))/10,2))
#denominator = math.sqrt((pow(-0.367752/10,2)+pow(-2.14561/10,2)+pow(-0.237886/10,2)))+math.sqrt(pow(0.0 -0.367752/10,2)+pow(0.0 -2.14561/10,2)+pow(0.0 -0.237886/10,2))
#theta = numpy.arccos(numerator/denominator)
#print(theta)
#cost = cost+k_2*pow(theta,2)
cons = cs.vertcat(cs.fmax(0.0, j_11*(f_1/d_t+delta_1)+j_12*(f_2/d_t+delta_2)+j_13*(f_3/d_t+delta_3)+j_14*(f_4/d_r0+delta_4)+j_15*(f_5/d_r0+delta_5)+j_16*(f_6/d_r0+delta_6)+b_1*a-dq_max1), 
                  cs.fmax(0.0, -(j_11*(f_1/d_t+delta_1)+j_12*(f_2/d_t+delta_2)+j_13*(f_3/d_t+delta_3)+j_14*(f_4/d_r0+delta_4)+j_15*(f_5/d_r0+delta_5)+j_16*(f_6/d_r0+delta_6)+b_1*a)+dq_min1),
                  cs.fmax(0.0, j_21*(f_1/d_t+delta_1)+j_22*(f_2/d_t+delta_2)+j_23*(f_3/d_t+delta_3)+j_24*(f_4/d_r0+delta_4)+j_25*(f_5/d_r0+delta_5)+j_26*(f_6/d_r0+delta_6)+b_2*a-dq_max2), 
                  cs.fmax(0.0, -(j_21*(f_1/d_t+delta_1)+j_22*(f_2/d_t+delta_2)+j_23*(f_3/d_t+delta_3)+j_24*(f_4/d_r0+delta_4)+j_25*(f_5/d_r0+delta_5)+j_26*(f_6/d_r0+delta_6)+b_2*a)+dq_min2),
                  cs.fmax(0.0, j_31*(f_1/d_t+delta_1)+j_32*(f_2/d_t+delta_2)+j_33*(f_3/d_t+delta_3)+j_34*(f_4/d_r0+delta_4)+j_35*(f_5/d_r0+delta_5)+j_36*(f_6/d_r0+delta_6)+b_3*a-dq_max3), 
                  cs.fmax(0.0, -(j_31*(f_1/d_t+delta_1)+j_32*(f_2/d_t+delta_2)+j_33*(f_3/d_t+delta_3)+j_34*(f_4/d_r0+delta_4)+j_35*(f_5/d_r0+delta_5)+j_36*(f_6/d_r0+delta_6)+b_3*a)+dq_min3),
                  cs.fmax(0.0, j_41*(f_1/d_t+delta_1)+j_42*(f_2/d_t+delta_2)+j_43*(f_3/d_t+delta_3)+j_44*(f_4/d_r0+delta_4)+j_45*(f_5/d_r0+delta_5)+j_46*(f_6/d_r0+delta_6)+b_4*a-dq_max4), 
                  cs.fmax(0.0, -(j_41*(f_1/d_t+delta_1)+j_42*(f_2/d_t+delta_2)+j_43*(f_3/d_t+delta_3)+j_44*(f_4/d_r0+delta_4)+j_45*(f_5/d_r0+delta_5)+j_46*(f_6/d_r0+delta_6)+b_4*a)+dq_min4),
                  cs.fmax(0.0, j_51*(f_1/d_t+delta_1)+j_52*(f_2/d_t+delta_2)+j_53*(f_3/d_t+delta_3)+j_54*(f_4/d_r0+delta_4)+j_55*(f_5/d_r0+delta_5)+j_56*(f_6/d_r0+delta_6)+b_5*a-dq_max5), 
                  cs.fmax(0.0, -(j_51*(f_1/d_t+delta_1)+j_52*(f_2/d_t+delta_2)+j_53*(f_3/d_t+delta_3)+j_54*(f_4/d_r0+delta_4)+j_55*(f_5/d_r0+delta_5)+j_56*(f_6/d_r0+delta_6)+b_5*a)+dq_min5),
                  cs.fmax(0.0, j_61*(f_1/d_t+delta_1)+j_62*(f_2/d_t+delta_2)+j_63*(f_3/d_t+delta_3)+j_64*(f_4/d_r0+delta_4)+j_65*(f_5/d_r0+delta_5)+j_66*(f_6/d_r0+delta_6)+b_6*a-dq_max6), 
                  cs.fmax(0.0, -(j_61*(f_1/d_t+delta_1)+j_62*(f_2/d_t+delta_2)+j_63*(f_3/d_t+delta_3)+j_64*(f_4/d_r0+delta_4)+j_65*(f_5/d_r0+delta_5)+j_66*(f_6/d_r0+delta_6)+b_6*a)+dq_min6),
                  cs.fmax(0.0, j_71*(f_1/d_t+delta_1)+j_72*(f_2/d_t+delta_2)+j_73*(f_3/d_t+delta_3)+j_74*(f_4/d_r0+delta_4)+j_75*(f_5/d_r0+delta_5)+j_76*(f_6/d_r0+delta_6)+b_7*a-dq_max7), 
                  cs.fmax(0.0, -(j_71*(f_1/d_t+delta_1)+j_72*(f_2/d_t+delta_2)+j_73*(f_3/d_t+delta_3)+j_74*(f_4/d_r0+delta_4)+j_75*(f_5/d_r0+delta_5)+j_76*(f_6/d_r0+delta_6)+b_7*a)+dq_min7))

problem = og.builder.Problem(u, p, cost) \
    .with_penalty_constraints(cons)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting
script_directory = pathlib.Path(__file__).parent.parent.resolve()
os.chdir(script_directory)
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("optimizer2")
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


