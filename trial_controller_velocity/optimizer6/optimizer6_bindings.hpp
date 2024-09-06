/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// Number of decision variables
static const uintptr_t OPTIMIZER6_NUM_DECISION_VARIABLES = 6;

/// Number of parameters
static const uintptr_t OPTIMIZER6_NUM_PARAMETERS = 53;

/// Number of parameters associated with augmented Lagrangian
static const uintptr_t OPTIMIZER6_N1 = 0;

/// Number of penalty constraints
static const uintptr_t OPTIMIZER6_N2 = 16;

/// optimizer6 version of ExitStatus
/// Structure: `optimizer6ExitStatus`
enum class optimizer6ExitStatus {
  /// The algorithm has converged
  ///
  /// All termination criteria are satisfied and the algorithm
  /// converged within the available time and number of iterations
  optimizer6Converged,
  /// Failed to converge because the maximum number of iterations was reached
  optimizer6NotConvergedIterations,
  /// Failed to converge because the maximum execution time was reached
  optimizer6NotConvergedOutOfTime,
  /// If the gradient or cost function cannot be evaluated internally
  optimizer6NotConvergedCost,
  /// Computation failed and NaN/Infinite value was obtained
  optimizer6NotConvergedNotFiniteComputation,
};

/// Solver cache (structure `optimizer6Cache`)
///
struct optimizer6Cache;

/// optimizer6 version of AlmOptimizerStatus
/// Structure: `optimizer6SolverStatus`
///
struct optimizer6SolverStatus {
  /// Exit status
  optimizer6ExitStatus exit_status;
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
optimizer6Cache *optimizer6_new();

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// # Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `optimizer6_new` (and should be destroyed once it is not
///   needed using `optimizer6_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `OPTIMIZER6_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `OPTIMIZER6_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `OPTIMIZER6_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
/// .
/// .
/// # Returns:
/// Instance of `optimizer6SolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
/// .
/// .
/// .
/// # Safety
/// All arguments must have been properly initialised
optimizer6SolverStatus optimizer6_solve(optimizer6Cache *instance,
                                        double *u,
                                        const double *params,
                                        const double *y0,
                                        const double *c0);

/// Deallocate the solver's memory, which has been previously allocated
/// using `optimizer6_new`
///
///
/// # Safety
/// All arguments must have been properly initialised
void optimizer6_free(optimizer6Cache *instance);

} // extern "C"
