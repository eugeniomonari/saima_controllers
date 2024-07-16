#!/usr/bin/env python

import opengen as og
import casadi.casadi as cs
import math
import numpy
import pathlib
import os

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 2)
p = cs.SX.sym("p", 83)

m_t = u[0]
#d_r = u[1]
a = u[1]


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

v_prev_1 = p[48]
v_prev_2 = p[49]
v_prev_3 = p[50]
v_prev_4 = p[51]
v_prev_5 = p[52]
v_prev_6 = p[53]

b_1 = p[54]
b_2 = p[55]
b_3 = p[56]
b_4 = p[57]
b_5 = p[58]
b_6 = p[59]
b_7 = p[60]

dq_max1 = p[61]
dq_max2 = p[62]
dq_max3 = p[63]
dq_max4 = p[64]
dq_max5 = p[65]
dq_max6 = p[66]
dq_max7 = p[67]

dq_min1 = p[68]
dq_min2 = p[69]
dq_min3 = p[70]
dq_min4 = p[71]
dq_min5 = p[72]
dq_min6 = p[73]
dq_min7 = p[74]

m_t0 = p[75]
m_r0 = p[76]
d_t0 = p[77]
d_r0 = p[78]
m_h = p[79]
m_l = p[80]
k_1 = p[81]
k_2 = p[82]

cost = cs.if_else(m_t < m_t0, m_h*pow(m_t-m_t0,2), m_l*pow(m_t-m_t0,2),True)
#cost = cost+cs.if_else(d_r < d_r0, m_h*pow(d_r-d_r0,2), m_l*pow(d_r-d_r0,2),True)
#cost = m_l*pow(d_t-d_t0,2)

cost = cost + k_1*pow(a,2)


#cost = cost + squared_sine

#cost = cost + k_2*cs.if_else(dot_product > 0, squared_sine, 2-squared_sine,True)
#cost = cost + k_2*(pow(delta_1,2)+pow(delta_2,2)+pow(delta_3,2)+pow(delta_4,2)+pow(delta_5,2)+pow(delta_6,2))
#numerator = math.sqrt(pow((f_2*(delta_3 + 1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_)))/d_t - (f_3*(delta_2 + 1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_)))/d_t,2)+pow((f_3*(delta_1 + 1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_)))/d_t - (f_1*(delta_3 + 1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_)))/d_t,2)+pow((f_1*(delta_2 + 1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_)))/d_t - (f_2*(delta_1 + 1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_)))/d_t,2))
#numerator = (f_1*(delta_1 + 1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_)))/d_t + (f_2*(delta_2 + 1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_)))/d_t + (f_3*(delta_3 + 1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_)))/d_t
#denominator = math.sqrt((pow(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_),2)+pow(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_),2)+pow(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_),2))+0.0001)*math.sqrt((pow(delta_1 + 1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_),2)+pow(delta_2 + 1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_),2)+pow(delta_3 + 1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_),2))+0.0001)
#denominator = math.sqrt((pow(delta_1 + 1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_),2)+pow(delta_2 + 1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_),2)+pow(delta_3 + 1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_),2))+0.0001)
#numerator = math.sqrt(pow((-2.14561*(0.0 -0.237886/10))/10 - (-0.237886*(0.0 -2.14561/10))/10,2)+pow((-0.237886*(0.0 -0.367752/10))/10 - (-0.367752*(0.0 -0.237886/10))/10,2)+pow((-0.367752*(0.0 -2.14561/10))/10 - (-2.14561*(0.0 -0.367752/10))/10,2))
#denominator = math.sqrt((pow(-0.367752/10,2)+pow(-2.14561/10,2)+pow(-0.237886/10,2)))+math.sqrt(pow(0.0 -0.367752/10,2)+pow(0.0 -2.14561/10,2)+pow(0.0 -0.237886/10,2))
#theta = numpy.arccos(numerator/denominator)
#print(theta)
#cost = cost+k_2*pow(theta,2)
T_ = 0.001

cons = cs.vertcat(cs.fmax(0.0, j_11*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_12*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_13*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_14*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_15*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_16*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_1*a-dq_max1), 
                  cs.fmax(0.0, -(j_11*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_12*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_13*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_14*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_15*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_16*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_1*a)+dq_min1),
                  cs.fmax(0.0, j_21*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_22*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_23*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_24*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_25*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_26*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_2*a-dq_max2), 
                  cs.fmax(0.0, -(j_21*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_22*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_23*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_24*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_25*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_26*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_2*a)+dq_min2),
                  cs.fmax(0.0, j_31*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_32*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_33*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_34*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_35*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_36*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_3*a-dq_max3), 
                  cs.fmax(0.0, -(j_31*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_32*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_33*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_34*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_35*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_36*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_3*a)+dq_min3),
                  cs.fmax(0.0, j_41*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_42*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_43*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_44*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_45*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_46*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_4*a-dq_max4), 
                  cs.fmax(0.0, -(j_41*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_42*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_43*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_44*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_45*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_46*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_4*a)+dq_min4),
                  cs.fmax(0.0, j_51*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_52*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_53*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_54*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_55*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_56*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_5*a-dq_max5), 
                  cs.fmax(0.0, -(j_51*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_52*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_53*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_54*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_55*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_56*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_5*a)+dq_min5),
                  cs.fmax(0.0, j_61*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_62*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_63*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_64*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_65*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_66*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_6*a-dq_max6), 
                  cs.fmax(0.0, -(j_61*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_62*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_63*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_64*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_65*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_66*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_6*a)+dq_min6),
                  cs.fmax(0.0, j_71*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_72*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_73*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_74*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_75*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_76*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_7*a-dq_max7), 
                  cs.fmax(0.0, -(j_71*(1/(1+d_t0/m_t*T_)*(v_prev_1+f_1/m_t*T_))+j_72*(1/(1+d_t0/m_t*T_)*(v_prev_2+f_2/m_t*T_))+j_73*(1/(1+d_t0/m_t*T_)*(v_prev_3+f_3/m_t*T_))+j_74*(1/(1+d_r0/m_r0*T_)*(v_prev_4+f_4/m_r0*T_))+j_75*(1/(1+d_r0/m_r0*T_)*(v_prev_5+f_5/m_r0*T_))+j_76*(1/(1+d_r0/m_r0*T_)*(v_prev_6+f_6/m_r0*T_))+b_7*a)+dq_min7))

problem = og.builder.Problem(u, p, cost) \
    .with_penalty_constraints(cons)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting
script_directory = pathlib.Path(__file__).parent.parent.resolve()
os.chdir(script_directory)
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("optimizer5")
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




