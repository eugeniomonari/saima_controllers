/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Number of decision variables
 */
#define OPTIMIZER6_NUM_DECISION_VARIABLES 6

/**
 * Number of parameters
 */
#define OPTIMIZER6_NUM_PARAMETERS 53

/**
 * Number of parameters associated with augmented Lagrangian
 */
#define OPTIMIZER6_N1 0

/**
 * Number of penalty constraints
 */
#define OPTIMIZER6_N2 16

/**
 * optimizer6 version of ExitStatus
 * Structure: `optimizer6ExitStatus`
 */
typedef enum optimizer6ExitStatus {
  /**
   * The algorithm has converged
   *
   * All termination criteria are satisfied and the algorithm
   * converged within the available time and number of iterations
   */
  optimizer6Converged,
  /**
   * Failed to converge because the maximum number of iterations was reached
   */
  optimizer6NotConvergedIterations,
  /**
   * Failed to converge because the maximum execution time was reached
   */
  optimizer6NotConvergedOutOfTime,
  /**
   * If the gradient or cost function cannot be evaluated internally
   */
  optimizer6NotConvergedCost,
  /**
   * Computation failed and NaN/Infinite value was obtained
   */
  optimizer6NotConvergedNotFiniteComputation,
} optimizer6ExitStatus;

/**
 * Solver cache (structure `optimizer6Cache`)
 *
 */
typedef struct optimizer6Cache optimizer6Cache;

/**
 * optimizer6 version of AlmOptimizerStatus
 * Structure: `optimizer6SolverStatus`
 *
 */
typedef struct optimizer6SolverStatus {
  /**
   * Exit status
   */
  enum optimizer6ExitStatus exit_status;
  /**
   * Number of outer iterations
   */
  unsigned long num_outer_iterations;
  /**
   * Total number of inner iterations
   *
   * This is the sum of the numbers of iterations of
   * inner solvers
   */
  unsigned long num_inner_iterations;
  /**
   * Norm of the fixed-point residual of the the problem
   */
  double last_problem_norm_fpr;
  /**
   * Total solve time
   */
  unsigned long long solve_time_ns;
  /**
   * Penalty value
   */
  double penalty;
  /**
   * Norm of delta y divided by the penalty parameter
   */
  double delta_y_norm_over_c;
  /**
   * Norm of F2(u)
   */
  double f2_norm;
  /**
   * Value of cost function at solution
   */
  double cost;
  /**
   * Lagrange multipliers
   */
  const double *lagrange;
} optimizer6SolverStatus;

/**
 * Allocate memory and setup the solver
 */
struct optimizer6Cache *optimizer6_new(void);

/**
 * Solve the parametric optimization problem for a given parameter
 * .
 * .
 * # Arguments:
 * - `instance`: re-useable instance of AlmCache, which should be created using
 *   `optimizer6_new` (and should be destroyed once it is not
 *   needed using `optimizer6_free`
 * - `u`: (on entry) initial guess of solution, (on exit) solution
 *   (length: `OPTIMIZER6_NUM_DECISION_VARIABLES`)
 * - `params`:  static parameters of the optimizer
 *   (length: `OPTIMIZER6_NUM_PARAMETERS`)
 * - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
 *   be used; length: `OPTIMIZER6_N1`)
 * - `c0`: Initial penalty parameter (provide `0` to use the default initial
 *   penalty parameter
 * .
 * .
 * # Returns:
 * Instance of `optimizer6SolverStatus`, with the solver status
 * (e.g., number of inner/outer iterations, measures of accuracy, solver time,
 * and the array of Lagrange multipliers at the solution).
 * .
 * .
 * .
 * # Safety
 * All arguments must have been properly initialised
 */
struct optimizer6SolverStatus optimizer6_solve(struct optimizer6Cache *instance,
                                               double *u,
                                               const double *params,
                                               const double *y0,
                                               const double *c0);

/**
 * Deallocate the solver's memory, which has been previously allocated
 * using `optimizer6_new`
 *
 *
 * # Safety
 * All arguments must have been properly initialised
 */
void optimizer6_free(struct optimizer6Cache *instance);
