import numpy as np
import cvxpy as cp


class LinearMPC:
    def __init__(self, dt, wheel_base, N_horizon=10,
                 Q=None, R=None, Q_N=None,
                 vr_max=0.5, vl_max=0.5,
                 delta_u_max=0.05,
                 s=0.0):
        '''Initialize the LinearMPC class with the given parameters.
        Parameters:
        dt : float
            Time step for the MPC controller.
        wheel_base : float
            Distance between the wheels of the robot.
        N_horizon : int, optional
            Number of time steps in the prediction horizon (default is 10).
        Q : np.ndarray, optional
            State cost matrix (default is None).
        R : np.ndarray, optional
            Control cost matrix (default is None).
        Q_N : np.ndarray, optional
            Terminal state cost matrix (default is None).
        vr_max : float, optional
            Maximum velocity of the right wheel (default is 1.0).
        vl_max : float, optional
            Maximum velocity of the left wheel (default is 1.0).
        s : float, optional
            Slip factor for both wheels (default is 0.0).
        '''

        self.dt  = dt
        self.l   = wheel_base
        self.s   = s
        self.N = N_horizon

        self.Q   = Q   if Q   is not None else np.diag([10.0, 10.0, 2.0])
        self.R   = R   if R   is not None else np.diag([0.1, 0.1])
        self.Q_N = Q_N if Q_N is not None else np.diag([10.0, 10.0, 2.0])


        #define cvxpy variables for the optimization problem
        self.E = cp.Variable((N_horizon + 1, 3))  # State error variables
        self.U = cp.Variable((N_horizon, 2))  # Control input variables

        #define cvxpy parameters for the optimization problem
        self.E0     = cp.Parameter(3)                                        # Initial state error
        self.U_prev = cp.Parameter(2)                                        # Control applied at previous step
        self.A = [cp.Parameter((3, 3)) for _ in range(N_horizon)]  # State transition matrices
        self.B = [cp.Parameter((3, 2)) for _ in range(N_horizon)]  # Control input matrices

        self.problem = self._build_problem(vr_max, vl_max, delta_u_max)

    def _build_problem(self, vr_max, vl_max, delta_u_max):
        '''Build the MPC optimization problem using cvxpy.
        Parameters:
        vr_max : float
            Maximum velocity of the right wheel.
        vl_max : float
            Maximum velocity of the left wheel.
        Cost: The cost function is defined as the sum of the state error costs and control input costs over the prediction horizon.
        Constraints: The constraints include the system dynamics (E[k+1] = A[k] * E[k] + B[k] * U[k]) for each time step in the horizon, and control input limits (|U[k, 0]| <= vr_max and |U[k, 1]| <= vl_max).
        Returns:
        problem : cvxpy.Problem
            The formulated MPC optimization problem.
        '''
        cost = 0
        constraints = [self.E[0] == self.E0] # Initial state error constraint

        for i in range(self.N):
            # Running cost: state error cost + control input cost
            cost += cp.quad_form(self.E[i], self.Q)
            cost += cp.quad_form(self.U[i], self.R)

            # System dynamics constraint
            constraints += [self.E[i + 1] == self.A[i] @ self.E[i] + self.B[i] @ self.U[i]]

        cost += cp.quad_form(self.E[self.N], self.Q_N)  # terminal cost

        # Control input constraints
        constraints += [ self.U[:, 0] <= vr_max,
                         self.U[:, 0] >= -vr_max,
                         self.U[:, 1] <= vl_max,
                         self.U[:, 1] >= -vl_max ]

        # Acceleration (rate-of-change) constraints
        constraints += [ self.U[0] - self.U_prev <=  delta_u_max,
                         self.U[0] - self.U_prev >= -delta_u_max ]
        for i in range(self.N - 1):
            constraints += [ self.U[i + 1] - self.U[i] <=  delta_u_max,
                             self.U[i + 1] - self.U[i] >= -delta_u_max ]

        return cp.Problem(cp.Minimize(cost), constraints)
    

    
    def solve(self, error_state, A_matrices, B_matrices, u_prev=None):
        '''Solve the MPC optimization problem  with the given error state and system matrices.
        Parameters:
        error_state : np.ndarray
            The current state error of the robot [x_error, y_error, theta_error].
        A_matrices : list of np.ndarray
            List of state transition matrices for each time step in the horizon.
        B_matrices : list of np.ndarray
            List of control input matrices for each time step in the horizon.
        u_prev : np.ndarray, optional
            Control input applied at the previous timestep [delta_vr, delta_vl].
            Used to enforce acceleration limits. Defaults to zeros.
        Returns:
        delta_vr: float
            The computed velocity correction for the right wheel.
        delta_vl: float
            The computed velocity correction for the left wheel.
        '''
        self.E0.value     = error_state
        self.U_prev.value = u_prev if u_prev is not None else np.zeros(2)
        for i in range(self.N):
            self.A[i].value = A_matrices[i]
            self.B[i].value = B_matrices[i]
        
        # Solve the optimization problem
        self.problem.solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-4, eps_rel=1e-4)

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            print(f'MPC optimization problem not solved to optimality. Status: {self.problem.status}')
            return 0.0, 0.0  # Return zero corrections if problem is not solved
        
        # Return the first control input correction from the optimized sequence
        delta_vr = self.U.value[0, 0]
        delta_vl = self.U.value[0, 1]
        return delta_vr, delta_vl
    


    def define_AB_matrices(self, theta, vel_right, vel_left):
        '''Compute the A and B matrices for the linearized system.
        Parameters:
        theta : float
            Current reference orientation of the robot in radians.
            Because linearization is done around the reference trajectory.
            Velocity of the right wheel.
        vel_left : float
            Velocity of the left wheel.
        Returns:
        A : np.ndarray
            State transition matrix.
        B : np.ndarray
            Control input matrix.
        '''
        s = self.s
        l = self.l
        dt = self.dt

        v_eff = (1 - s) * (vel_right + vel_left) / 2

        # continuous-time A and B matrices
        A_c = np.zeros((3, 3))
        A_c[0, 2] = -v_eff * np.sin(theta)
        A_c[1, 2] = v_eff * np.cos(theta)

        B_c = np.zeros((3, 2))
        B_c[0, 0] = (1 - s) * np.cos(theta) / 2
        B_c[0, 1] = (1 - s) * np.cos(theta) / 2
        B_c[1, 0] = (1 - s) * np.sin(theta) / 2
        B_c[1, 1] = (1 - s) * np.sin(theta) / 2
        B_c[2, 0] = (1 - s) / (2 * l)
        B_c[2, 1] = -(1 - s) / (2 * l)

        # discretize A and B using Euler method
        A_k = np.eye(3) + A_c * dt
        B_k = B_c * dt

        return A_k, B_k
    
    def compute_error_state(self, actual_state, reference_state):
        '''Compute the error state between the actual and reference states.
        Parameters:
        actual_state : np.ndarray
            The current state of the robot [x, y, theta].
        reference_state : np.ndarray
            The desired state of the robot [x_ref, y_ref, theta_ref].
        Returns:
        error_state : np.ndarray
            The error state [x_error, y_error, theta_error].
        '''
        error_state = actual_state - reference_state
        error_state[2] = np.arctan2(np.sin(error_state[2]), np.cos(error_state[2]))  # wrap heading error
        return error_state


    
