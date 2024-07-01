import opengen as og
import casadi.casadi as cs
import math

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 28)
cons = cs.vertcat(cs.fmax(0.0, p[0]/u[0]+p[7]*u[1]-p[14]),
               cs.fmax(0.0, -(p[0]/u[0]+p[7]*u[1])+p[21]),
               cs.fmax(0.0, p[1]/u[0]+p[8]*u[1]-p[15]),
               cs.fmax(0.0, -(p[1]/u[0]+p[8]*u[1])+p[22]),
               cs.fmax(0.0, p[2]/u[0]+p[9]*u[1]-p[16]),
               cs.fmax(0.0, -(p[2]/u[0]+p[9]*u[1])+p[23]),
               cs.fmax(0.0, p[3]/u[0]+p[10]*u[1]-p[17]),
               cs.fmax(0.0, -(p[3]/u[0]+p[10]*u[1])+p[24]),
               cs.fmax(0.0, p[4]/u[0]+p[11]*u[1]-p[18]),
               cs.fmax(0.0, -(p[4]/u[0]+p[11]*u[1])+p[25]),
               cs.fmax(0.0, p[5]/u[0]+p[12]*u[1]-p[19]),
               cs.fmax(0.0, -(p[5]/u[0]+p[12]*u[1])+p[26]),
               cs.fmax(0.0, p[6]/u[0]+p[13]*u[1]-p[20]),
               cs.fmax(0.0, -(p[6]/u[0]+p[13]*u[1])+p[27]))
#cons = cs.vertcat(cs.fmax(0.0, p[0]*u[0]+p[7]*u[1]-p[14]),
               #cs.fmax(0.0, -(p[0]*u[0]+p[7]*u[1])+p[21]),
               #cs.fmax(0.0, p[1]*u[0]+p[8]*u[1]-p[15]),
               #cs.fmax(0.0, -(p[1]*u[0]+p[8]*u[1])+p[22]),
               #cs.fmax(0.0, p[2]*u[0]+p[9]*u[1]-p[16]),
               #cs.fmax(0.0, -(p[2]*u[0]+p[9]*u[1])+p[23]),
               #cs.fmax(0.0, p[3]*u[0]+p[10]*u[1]-p[17]),
               #cs.fmax(0.0, -(p[3]*u[0]+p[10]*u[1])+p[24]),
               #cs.fmax(0.0, p[4]*u[0]+p[11]*u[1]-p[18]),
               #cs.fmax(0.0, -(p[4]*u[0]+p[11]*u[1])+p[25]),
               #cs.fmax(0.0, p[5]*u[0]+p[12]*u[1]-p[19]),
               #cs.fmax(0.0, -(p[5]*u[0]+p[12]*u[1])+p[26]),
               #cs.fmax(0.0, p[6]*u[0]+p[13]*u[1]-p[20]),
               #cs.fmax(0.0, -(p[6]*u[0]+p[13]*u[1])+p[27]))
b = 1
d_t0 = 10
#alpha_t0 = 0.1
c = 10
d = 50
f = 100
g = 10
cost = cs.if_else(u[0] > d_t0,1/10*(u[0]-d_t0)+pow(u[1],2),-10*(u[0]-d_t0)+pow(u[1],2),True)
#if d > d_t0:
    #cost = 1/10*(u[0]-d_t0)+pow(u[1],2)
#else:
    #cost = -10*(u[0]-d_t0)+pow(u[1],2)
#cost = b*pow(u[0] - alpha_t0,2) + (d*math.exp(f*(u[0] - alpha_t0)))/(math.exp(-c*(u[0] - alpha_t0)) + 1)+g*pow(u[1],2)
#cost = b*pow((u[0] - d_t0),2) + (d*math.exp(-f*(u[0] - d_t0)))/(math.exp(c*(u[0] - d_t0)) + 1)+g*pow(u[1],2)
#cost = b*pow(u[0] - d_t0,2)*((d - 1)/(math.exp(c*(u[0] - d_t0)) + 1) + 1)
#cost = b*pow(u[0] - d_t0,2)+g*pow(u[1],2)
problem = og.builder.Problem(u, p, cost) \
    .with_penalty_constraints(cons)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("optimizer1")
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1000)                     \
.with_penalty_weight_update_factor(5)               \
    .with_max_outer_iterations(100)                 \
    #.with_max_duration_micros(200)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
#builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          #build_config)
builder.build()
