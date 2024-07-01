/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// Number of decision variables
static const uintptr_t OPTIMIZER1_NUM_DECISION_VARIABLES = 5;

/// Number of parameters
static const uintptr_t OPTIMIZER1_NUM_PARAMETERS = 50;

/// Number of parameters associated with augmented Lagrangian
static const uintptr_t OPTIMIZER1_N1 = 0;

/// Number of penalty constraints
static const uintptr_t OPTIMIZER1_N2 = 14;

/// optimizer1 version of ExitStatus
/// Structure: `optimizer1ExitStatus`
enum class optimizer1ExitStatus {
  /// The algorithm has converged
  ///
  /// All termination criteria are satisfied and the algorithm
  /// converged within the available time and number of iterations
  optimizer1Converged,
  /// Failed to converge because the maximum number of iterations was reached
  optimizer1NotConvergedIterations,
  /// Failed to converge because the maximum execution time was reached
  optimizer1NotConvergedOutOfTime,
  /// If the gradient or cost function cannot be evaluated internally
  optimizer1NotConvergedCost,
  /// Computation failed and NaN/Infinite value was obtained
  optimizer1NotConvergedNotFiniteComputation,
};

/// Solver cache (structure `optimizer1Cache`)
///
struct optimizer1Cache;

/// optimizer1 version of AlmOptimizerStatus
/// Structure: `optimizer1SolverStatus`
///
struct optimizer1SolverStatus {
  /// Exit status
  optimizer1ExitStatus exit_status;
  /// Number of outer iterations
  unsigned long num_outer_iterations;
  /// Total number of inner iterations
  ///
  /// This is the sum of the numbers of iterations of
  /// inner solvers
  unsigned long num_inner_iterations;
  /// Norm of the fixed-point residual of the the problem
  double last_problem_norm_fpr;
  /// Total solve time
  unsigned long long solve_time_ns;
  /// Penalty value
  double penalty;
  /// Norm of delta y divided by the penalty parameter
  double delta_y_norm_over_c;
  /// Norm of F2(u)
  double f2_norm;
  /// Value of cost function at solution
  double cost;
  /// Lagrange multipliers
  const double *lagrange;
};

extern "C" {

/// Allocate memory and setup the solver
optimizer1Cache *optimizer1_new();

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// # Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `optimizer1_new` (and should be destroyed once it is not
///   needed using `optimizer1_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `OPTIMIZER1_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `OPTIMIZER1_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `OPTIMIZER1_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
/// .
/// .
/// # Returns:
/// Instance of `optimizer1SolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
/// .
/// .
/// .
/// # Safety
/// All arguments must have been properly initialised
optimizer1SolverStatus optimizer1_solve(optimizer1Cache *instance,
                                        double *u,
                                        const double *params,
                                        const double *y0,
                                        const double *c0);

/// Deallocate the solver's memory, which has been previously allocated
/// using `optimizer1_new`
///
///
/// # Safety
/// All arguments must have been properly initialised
void optimizer1_free(optimizer1Cache *instance);

} // extern "C"