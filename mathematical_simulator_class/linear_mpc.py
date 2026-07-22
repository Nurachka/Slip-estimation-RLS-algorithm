import numpy as np
import cvxpy as cp


class LinearMPC:
    def __init__(self, dt, wheel_base, N_horizon=10,
                 Q=None, R=None, Q_N=None, S=None,
                 vr_max=0.5, vl_max=0.5, s=0.0, du_max=0.05):
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
        S : np.ndarray or None, optional
            Input-change cost matrix penalizing ΔU[k] = v_commanded[k] - v_commanded[k-1].
            None disables the cost term.
        vr_max : float, optional
            Maximum velocity of the right wheel (default is 0.2).
        vl_max : float, optional
            Maximum velocity of the left wheel (default is 0.2).
        s : float, optional
            Slip factor for both wheels (default is 0.0).
        du_max : float or None, optional
            Maximum allowed change in each wheel's delta-velocity per step
            (slew-rate / acceleration constraint). None disables the constraint.
        '''

        self.dt  = dt
        self.l   = wheel_base
        self.s   = s
        self.N = N_horizon

        self.Q   = Q   if Q   is not None else np.diag([50.0, 50.0, 10.0])
        self.R   = R   if R   is not None else np.diag([0.5, 0.5])
        self.Q_N = Q_N if Q_N is not None else np.diag([50.0, 50.0, 10.0])
        self.S   = S   if S   is not None else np.diag([1.0,1.0])  # default no ΔU cost

        #define cvxpy variables for the optimization problem
        self.E = cp.Variable((N_horizon + 1, 3))
        self.U = cp.Variable((N_horizon, 2))

        #define cvxpy parameters for the optimization problem
        self.E0 = cp.Parameter(3)
        self.A = [cp.Parameter((3, 3)) for _ in range(N_horizon)]
        self.B = [cp.Parameter((3, 2)) for _ in range(N_horizon)]
        self.VR_ref = cp.Parameter(N_horizon, value=np.zeros(N_horizon))
        self.VL_ref = cp.Parameter(N_horizon, value=np.zeros(N_horizon))
        self.VR_ref_prev = cp.Parameter(value=0.0)
        self.VL_ref_prev = cp.Parameter(value=0.0)
        self._vr_ref_prev = None
        self._vl_ref_prev = None
        self.U_prev = cp.Parameter(2, value=np.zeros(2))

        self.problem = self._build_problem(vr_max, vl_max, du_max)

    def _build_problem(self, vr_max, vl_max, du_max):
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
        constraints = [self.E[0] == self.E0]

        for i in range(self.N):
            cost += cp.quad_form(self.E[i], self.Q)
            cost += cp.quad_form(self.U[i], self.R)
            constraints += [self.E[i + 1] == self.A[i] @ self.E[i] + self.B[i] @ self.U[i]]
            if self.S is not None:
                if i == 0:
                    dU = cp.hstack([
                        self.U[0, 0] + self.VR_ref[0] - self.U_prev[0] - self.VR_ref_prev,
                        self.U[0, 1] + self.VL_ref[0] - self.U_prev[1] - self.VL_ref_prev
                    ])
                else:
                    dU = cp.hstack([
                        self.U[i, 0] + self.VR_ref[i] - self.U[i-1, 0] - self.VR_ref[i-1],
                        self.U[i, 1] + self.VL_ref[i] - self.U[i-1, 1] - self.VL_ref[i-1]
                    ])
                cost += cp.quad_form(dU, self.S)

        cost += cp.quad_form(self.E[self.N], self.Q_N)

        constraints += [ self.U[:, 0] + self.VR_ref <= vr_max,
                         self.U[:, 0] + self.VR_ref >= -vr_max,
                         self.U[:, 1] + self.VL_ref <= vl_max,
                         self.U[:, 1] + self.VL_ref >= -vl_max ]

        if du_max is not None:
            constraints += [
                self.U[0, 0] + self.VR_ref[0] - self.U_prev[0] - self.VR_ref_prev <= du_max,
                self.U[0, 0] + self.VR_ref[0] - self.U_prev[0] - self.VR_ref_prev >= -du_max,
                self.U[0, 1] + self.VL_ref[0] - self.U_prev[1] - self.VL_ref_prev <= du_max,
                self.U[0, 1] + self.VL_ref[0] - self.U_prev[1] - self.VL_ref_prev >= -du_max,
            ]
            for k in range(1, self.N):
                constraints += [
                    self.U[k, 0] + self.VR_ref[k] - self.U[k-1, 0] - self.VR_ref[k-1] <= du_max,
                    self.U[k, 0] + self.VR_ref[k] - self.U[k-1, 0] - self.VR_ref[k-1] >= -du_max,
                    self.U[k, 1] + self.VL_ref[k] - self.U[k-1, 1] - self.VL_ref[k-1] <= du_max,
                    self.U[k, 1] + self.VL_ref[k] - self.U[k-1, 1] - self.VL_ref[k-1] >= -du_max,
                ]

        return cp.Problem(cp.Minimize(cost), constraints)



    def solve(self, error_state, A_matrices, B_matrices, vr_ref_horizon, vl_ref_horizon):
        '''Solve the MPC optimization problem  with the given error state and system matrices.
        Parameters:
        error_state : np.ndarray
            The current state error of the robot [x_error, y_error, theta_error].
        A_matrices : list of np.ndarray
            List of state transition matrices for each time step in the horizon.
        B_matrices : list of np.ndarray
            List of control input matrices for each time step in the horizon.
        vr_ref_horizon : list of float
            Reference right wheel velocities over the prediction horizon.
        vl_ref_horizon : list of float
            Reference left wheel velocities over the prediction horizon.
        Returns:
        delta_vr: float
            The computed velocity correction for the right wheel.
        delta_vl: float
            The computed velocity correction for the left wheel.
        '''
        self.E0.value = error_state
        self.VR_ref.value = np.array(vr_ref_horizon)
        self.VL_ref.value = np.array(vl_ref_horizon)
        if self._vr_ref_prev is None:
            self._vr_ref_prev = vr_ref_horizon[0]
            self._vl_ref_prev = vl_ref_horizon[0]
        self.VR_ref_prev.value = self._vr_ref_prev
        self.VL_ref_prev.value = self._vl_ref_prev

        for i in range(self.N):
            self.A[i].value = A_matrices[i]
            self.B[i].value = B_matrices[i]

        self.problem.solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-4, eps_rel=1e-4)

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            print(f'MPC optimization problem not solved to optimality. Status: {self.problem.status}')
            return 0.0, 0.0

        delta_vr = self.U.value[0, 0]
        delta_vl = self.U.value[0, 1]
        self.U_prev.value = np.array([delta_vr, delta_vl])
        self._vr_ref_prev = vr_ref_horizon[0]
        self._vl_ref_prev = vl_ref_horizon[0]
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
        B_c[2, 0] = (1 - s) / (l)
        B_c[2, 1] = -(1 - s) / (l)

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



