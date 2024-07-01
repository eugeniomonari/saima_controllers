//
// Auto-generated file by OptimizationEngine
// See https://alphaville.github.io/optimization-engine/
//
// Generated at: 2024-06-27 17:21:34.358036
//

use libc::{c_double, c_ulong, c_ulonglong};

use optimization_engine::{constraints::*, panoc::*, alm::*, *};

// ---Private Constants----------------------------------------------------------------------------------

/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = 1e-05;

/// Initial tolerance
const INITIAL_EPSILON_TOLERANCE: f64 = 0.0001;

/// Update factor for inner tolerance
const EPSILON_TOLERANCE_UPDATE_FACTOR: f64 = 0.1;

/// Delta tolerance
const DELTA_TOLERANCE: f64 = 0.0001;

/// LBFGS memory
const LBFGS_MEMORY: usize = 10;

/// Maximum number of iterations of the inner solver
const MAX_INNER_ITERATIONS: usize = 500;

/// Maximum number of outer iterations
const MAX_OUTER_ITERATIONS: usize = 100;

/// Maximum execution duration in microseconds
const MAX_DURATION_MICROS: u64 = 500;

/// Penalty update factor
const PENALTY_UPDATE_FACTOR: f64 = 5.0;

/// Initial penalty
const INITIAL_PENALTY_PARAMETER: f64 = 1000.0;

/// Sufficient decrease coefficient
const SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT: f64 = 0.1;


// ---Public Constants-----------------------------------------------------------------------------------

/// Number of decision variables
pub const OPTIMIZER1_NUM_DECISION_VARIABLES: usize = 5;

/// Number of parameters
pub const OPTIMIZER1_NUM_PARAMETERS: usize = 50;

/// Number of parameters associated with augmented Lagrangian
pub const OPTIMIZER1_N1: usize = 0;

/// Number of penalty constraints
pub const OPTIMIZER1_N2: usize = 14;

// ---Export functionality from Rust to C/C++------------------------------------------------------------

/// Solver cache (structure `optimizer1Cache`)
///
#[allow(non_camel_case_types)]
pub struct optimizer1Cache {
    cache: AlmCache,
}

impl optimizer1Cache {
    pub fn new(cache: AlmCache) -> Self {
        optimizer1Cache { cache }
    }
}

/// optimizer1 version of ExitStatus
/// Structure: `optimizer1ExitStatus`
#[allow(non_camel_case_types)]
#[repr(C)]
pub enum optimizer1ExitStatus {
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
}

/// optimizer1 version of AlmOptimizerStatus
/// Structure: `optimizer1SolverStatus`
///
#[repr(C)]
pub struct optimizer1SolverStatus {
    /// Exit status
    exit_status: optimizer1ExitStatus,
    /// Number of outer iterations
    num_outer_iterations: c_ulong,
    /// Total number of inner iterations
    ///
    /// This is the sum of the numbers of iterations of
    /// inner solvers
    num_inner_iterations: c_ulong,
    /// Norm of the fixed-point residual of the the problem
    last_problem_norm_fpr: c_double,
    /// Total solve time
    solve_time_ns: c_ulonglong,
    /// Penalty value
    penalty: c_double,
    /// Norm of delta y divided by the penalty parameter
    delta_y_norm_over_c: c_double,
    /// Norm of F2(u)
    f2_norm: c_double,
    /// Value of cost function at solution
    cost: c_double,
    /// Lagrange multipliers
    lagrange: *const c_double
    }

/// Allocate memory and setup the solver
#[no_mangle]
pub extern "C" fn optimizer1_new() -> *mut optimizer1Cache {
    Box::into_raw(Box::new(optimizer1Cache::new(initialize_solver())))
}

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
#[no_mangle]
pub unsafe extern "C" fn optimizer1_solve(
    instance: *mut optimizer1Cache,
    u: *mut c_double,
    params: *const c_double,
    y0: *const c_double,
    c0: *const c_double,
) -> optimizer1SolverStatus {

    // Convert all pointers into the required data structures
    let instance: &mut optimizer1Cache = {
        assert!(!instance.is_null());
        &mut *instance
    };

    // "*mut c_double" to "&mut [f64]"
    let u : &mut [f64] = {
        assert!(!u.is_null());
        std::slice::from_raw_parts_mut(u as *mut f64, OPTIMIZER1_NUM_DECISION_VARIABLES)
    };

    // "*const c_double" to "&[f64]"
    let params : &[f64] = {
        assert!(!params.is_null());
        std::slice::from_raw_parts(params as *const f64, OPTIMIZER1_NUM_PARAMETERS)
    };

    let c0_option: Option<f64> = if c0.is_null() {
        None::<f64>
    } else {
        Some(*c0)
    };

    let y0_option: Option<Vec<f64>> = if y0.is_null() {
        None::<Vec<f64>>
    } else {
        Some(std::slice::from_raw_parts(y0 as *mut f64, OPTIMIZER1_N1).to_vec())
    };

    // Invoke `solve`
    let status = solve(params,&mut instance.cache, u, &y0_option, &c0_option);

    // Check solution status and cast it as `optimizer1SolverStatus`
    match status {
        Ok(status) => optimizer1SolverStatus {
            exit_status: match status.exit_status() {
                core::ExitStatus::Converged => optimizer1ExitStatus::optimizer1Converged,
                core::ExitStatus::NotConvergedIterations => optimizer1ExitStatus::optimizer1NotConvergedIterations,
                core::ExitStatus::NotConvergedOutOfTime => optimizer1ExitStatus::optimizer1NotConvergedOutOfTime,
            },
            num_outer_iterations: status.num_outer_iterations() as c_ulong,
            num_inner_iterations: status.num_inner_iterations() as c_ulong,
            last_problem_norm_fpr: status.last_problem_norm_fpr(),
            solve_time_ns: status.solve_time().as_nanos() as c_ulonglong,
            penalty: status.penalty() as c_double,
            delta_y_norm_over_c: status.delta_y_norm_over_c() as c_double,
            f2_norm: status.f2_norm() as c_double,
            cost: status.cost() as c_double,
            lagrange: match status.lagrange_multipliers() {
                Some(_y) => {
                    std::ptr::null::<c_double>()
                
                },
                None => {
                    std::ptr::null::<c_double>()
                }
            }
        },
        Err(e) => optimizer1SolverStatus {
            exit_status: match e {
                SolverError::Cost => optimizer1ExitStatus::optimizer1NotConvergedCost,
                SolverError::NotFiniteComputation => optimizer1ExitStatus::optimizer1NotConvergedNotFiniteComputation,
            },
            num_outer_iterations: std::u64::MAX as c_ulong,
            num_inner_iterations: std::u64::MAX as c_ulong,
            last_problem_norm_fpr: std::f64::INFINITY,
            solve_time_ns: std::u64::MAX as c_ulonglong,
            penalty: std::f64::INFINITY as c_double,
            delta_y_norm_over_c: std::f64::INFINITY as c_double,
            f2_norm: std::f64::INFINITY as c_double,
            cost: std::f64::INFINITY as c_double,
            lagrange:std::ptr::null::<c_double>()
        },
    }
}

/// Deallocate the solver's memory, which has been previously allocated
/// using `optimizer1_new`
/// 
/// 
/// # Safety
/// All arguments must have been properly initialised
#[no_mangle]
pub unsafe extern "C" fn optimizer1_free(instance: *mut optimizer1Cache) {
    // Add impl
    assert!(!instance.is_null());
    Box::from_raw(instance);
}


// ---Parameters of the constraints----------------------------------------------------------------------











// ---Internal private helper functions------------------------------------------------------------------

/// Make constraints U
fn make_constraints() -> impl Constraint {
    // - No constraints (whole Rn):
    NoConstraints::new()
    }





// ---Main public API functions--------------------------------------------------------------------------


/// Initialisation of the solver
pub fn initialize_solver() -> AlmCache {
    let panoc_cache = PANOCCache::new(OPTIMIZER1_NUM_DECISION_VARIABLES, EPSILON_TOLERANCE, LBFGS_MEMORY);
    AlmCache::new(panoc_cache, OPTIMIZER1_N1, OPTIMIZER1_N2)
}


/// Solver interface
pub fn solve(
    p: &[f64],
    alm_cache: &mut AlmCache,
    u: &mut [f64],
    y0: &Option<Vec<f64>>,
    c0: &Option<f64>,
) -> Result<AlmOptimizerStatus, SolverError> {

    assert_eq!(p.len(), OPTIMIZER1_NUM_PARAMETERS, "Wrong number of parameters (p)");
    assert_eq!(u.len(), OPTIMIZER1_NUM_DECISION_VARIABLES, "Wrong number of decision variables (u)");

    let psi = |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi_optimizer1::cost(u, xi, p, cost);
        Ok(())
    };
    let grad_psi = |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        icasadi_optimizer1::grad(u, xi, p, grad);
        Ok(())
    };
    
    let f2 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi_optimizer1::mapping_f2(u, p, res);
        Ok(())
    };let bounds = make_constraints();

    let alm_problem = AlmProblem::new(
        bounds,
        NO_SET,
        NO_SET,
        psi,
        grad_psi,
        NO_MAPPING,
        Some(f2),
        OPTIMIZER1_N1,
        OPTIMIZER1_N2,
    );

    let mut alm_optimizer = AlmOptimizer::new(alm_cache, alm_problem)
        .with_delta_tolerance(DELTA_TOLERANCE)
        .with_epsilon_tolerance(EPSILON_TOLERANCE)
        .with_initial_inner_tolerance(INITIAL_EPSILON_TOLERANCE)
        .with_inner_tolerance_update_factor(EPSILON_TOLERANCE_UPDATE_FACTOR)
        .with_max_duration(std::time::Duration::from_micros(MAX_DURATION_MICROS))
        .with_max_outer_iterations(MAX_OUTER_ITERATIONS)
        .with_max_inner_iterations(MAX_INNER_ITERATIONS)
        .with_initial_penalty(c0.unwrap_or(INITIAL_PENALTY_PARAMETER))
        .with_penalty_update_factor(PENALTY_UPDATE_FACTOR)
        .with_sufficient_decrease_coefficient(SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT);

    // solve the problem using `u` an the initial condition `u` and
    // initial vector of Lagrange multipliers, if provided;
    // returns the problem status (instance of `AlmOptimizerStatus`)
    if let Some(y0_) = y0 {
        let mut alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(y0_);
        alm_optimizer.solve(u)
    } else {
        alm_optimizer.solve(u)
    }

}