#import casadi.casadi as cs
#import opengen as og

#u = cs.SX.sym("u", 5)  # Decision variabels
#p = cs.SX.sym("p", 2)  # Parameters

#phi = og.functions.rosenbrock(u, p)  # Cost function

## Equality constraints: 1.5 * u[0] = u[1] and u[2] = u[3]
## Can also be inequality constratins using max{c(u, p), 0}, where c(u, p) < 0.
#c = cs.vertcat(1.5*u[0] - u[1], u[2] - u[3])

## Bounds constraints on u
#umin = [-2.0] * 5  # shorthand notation
#umax = [ 2.0] * 5

## Bounds on u: (umin <= u <= umax)
#bounds = og.constraints.Rectangle(umin, umax)

## Define the problem
#problem = og.builder.Problem(u, p, phi)                 \
    #.with_penalty_constraints(c)                        \
    #.with_constraints(bounds)

## Meta information for the solver
#meta = og.config.OptimizerMeta()                        \
    #.with_version("1.0.0")                              \
    #.with_authors(["P. Sopasakis", "E. Fresk"])         \
    #.with_optimizer_name("the_optimizer")

## Lets build in release mode with C bindings
#build_config = og.config.BuildConfiguration()           \
    #.with_build_mode("release")                         \
    #.with_build_c_bindings()            # <--- The important setting

## Solver settings
#solver_config = og.config.SolverConfiguration()         \
    #.with_tolerance(1e-5)                               \
    #.with_max_outer_iterations(15)                      \
    #.with_penalty_weight_update_factor(8.0)             \

## Create the solver!
#builder = og.builder.OpEnOptimizerBuilder(problem,
                                          #metadata=meta,
                                          #build_configuration=build_config,
                                          #solver_configuration=solver_config)

#builder.build()
import opengen as og
import casadi.casadi as cs

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5 * u[0] - u[1],
               cs.fmax(0.0, u[2] - u[3] + 0.1))
bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c)        \
    .with_constraints(bounds)
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("the_optimizer")
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1e3)                     \
    .with_penalty_weight_update_factor(5)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()
