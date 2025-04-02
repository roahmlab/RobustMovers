import pinocchio as pin
import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import solve_ivp
import scipy.io as sio
# import pybullet as p

def compute_trajectories(t, coefficients, w, q0, q_d0, Nact = 7, degree = 3):
    """
    Compute trajectories based on coefficients and initial conditions
    
    Args:
        coefficients: numpy array of shape (2*degree+1, Nact)
        w: angular frequency
        q0: initial position
        q_d0: initial velocity
    """
    F = np.zeros(2 * degree + 1)
    dF = np.zeros(2 * degree + 1)
    ddF = np.zeros(2 * degree + 1)
    
    F0 = np.zeros(2 * degree + 1)
    dF0 = np.zeros(2 * degree + 1)
    
    q = np.zeros(Nact)
    q_d = np.zeros(Nact)
    q_dd = np.zeros(Nact)
    
    for i in range(Nact):
        # Get kernel (coefficient vector for joint i)
        kernel = coefficients[:, i]
        
        # Initialize base terms
        ddF[0] = 1
        dF[0] = t
        F[0] = t * t * 0.5
        
        dF0[0] = 0
        F0[0] = 0
        
        # Compute terms for each degree
        for j in range(degree):
            jt = (j + 1) * t
            sinjwt = np.sin(w * jt)
            cosjwt = np.cos(w * jt)
            
            # Second derivative terms
            ddF[2 * j + 1] = cosjwt
            ddF[2 * j + 2] = sinjwt
            
            # First derivative terms
            jw = (j + 1) * w
            dF[2 * j + 1] = sinjwt / jw
            dF[2 * j + 2] = -cosjwt / jw
            dF0[2 * j + 2] = -1 / jw
            
            # Position terms
            j2w2 = jw * jw
            F[2 * j + 1] = -cosjwt / j2w2
            F[2 * j + 2] = -sinjwt / j2w2
            F0[2 * j + 1] = -1 / j2w2
        
        # Compute final values for joint i
        q_dd[i] = np.dot(ddF, kernel)
        
        q_d_raw = np.dot(dF, kernel)
        q_d_raw0 = np.dot(dF0, kernel)
        q_d[i] = q_d_raw + (q_d0[i] - q_d_raw0)
        
        q_raw = np.dot(F, kernel) + (q_d0[i] - q_d_raw0) * t
        q_raw0 = np.dot(F0, kernel)
        q[i] = q_raw + (q0[i] - q_raw0)
    
    return q, q_d, q_dd

class grav_pid_controller:
    def __init__(self, model_path):
        self.model = pin.buildModelFromUrdf(model_path)
        self.data = self.model.createData()
        
        self.Kp = np.array([1000.0, 1000.0, 1000.0, 1000.0, 400.0, 400.0, 400.0])
        self.Kd = np.array([100.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0])
        # self.Kp = np.array([2000.0, 2000.0, 2000.0, 2000.0, 800.0, 800.0, 800.0])
        # self.Kd = np.array([200.0, 200.0, 200.0, 200.0, 80.0, 80.0, 80.0])
        self.Ki = np.zeros(self.model.nv)
        
        # self.accumulated_error = np.zeros(self.model.nv)
        
    def update(self, q, v, qd, qd_d, qd_dd):
        tau = pin.computeGeneralizedGravity(self.model, self.data, q)
        
        for i in range(self.model.nv):
            tau[i] += self.Kp[i] * (qd[i] - q[i]) + self.Kd[i] * (qd_d[i] - v[i])
            
        return tau

def integrate(model, ts_sim, x0, controller, method='BDF'):
    nq = model.nq
    nv = model.nv
    data = model.createData()

    def dynamics(t, x):
        q = x[:nq]
        v = x[nq:]
        
        qd, qd_d, qd_dd = desired_trajectory(t)
        
        tau = controller.update(q, v, qd, qd_d, qd_dd)
        
        a = pin.aba(
            model, data, q, v, tau
        )
        
        return np.concatenate([v, a])
    
    sol = solve_ivp(
        dynamics, 
        [ts_sim[0], ts_sim[-1]],
        x0, 
        method=method,
        t_eval=ts_sim
    )
    
    position = sol.y[:nq,:].T
    velocity = sol.y[nq:,:].T
    
    return position, velocity

# Kinova URDF model
model_path = "../../../Robots/kinova-gen3/gen3_2f85_dumbbell_7lb.urdf"
controller_model_path = "../../../Robots/kinova-gen3/gen3_2f85_fixed.urdf"
config_path = "../kinova_model_parameters.yaml"

mass_model_uncertainty = 0.05
com_model_uncertainty = 0.05
inertia_model_uncertainty = 0.05

if __name__ == "__main__":
    for traj_id in range(2, 21):
        print(traj_id)
    
        # Load trajectories
        try:
            data = np.loadtxt("/workspaces/RobustMovers-roahmlab/T7.5_d3_obs/exciting-solution-" + str(traj_id) + ".csv")
        except:
            continue
        
        degree = 3
        Nact = 7
        
        coefficients = data[:((2 * degree + 1) * Nact)]
        coefficients = coefficients.reshape((Nact, 2 * degree + 1)).T
        
        q0 = data[((2 * degree + 1) * Nact) : ((2 * degree + 1) * Nact + Nact)]
        q_d0 = data[((2 * degree + 1) * Nact + Nact) : ((2 * degree + 1) * Nact + 2 * Nact)]
        w = data[-1]
        
        desired_trajectory = lambda t: compute_trajectories(t, coefficients, w, q0, q_d0)
        
        # Load the URDF model
        model = pin.buildModelFromUrdf(model_path)
        
        # Add uncertainties to the model
        for i in range(model.nv):
            mass_uncertainty_range = (2 * mass_model_uncertainty * np.random.rand() - mass_model_uncertainty)
            model.inertias[i].mass *= (1 + mass_uncertainty_range)
            com_uncertainty_range = (2 * com_model_uncertainty * np.random.rand() - com_model_uncertainty)
            model.inertias[i].lever *= (1 + com_uncertainty_range)
            inertia_uncertainty_range = (2 * inertia_model_uncertainty * np.random.rand() - inertia_model_uncertainty)
            model.inertias[i].inertia *= (1 + inertia_uncertainty_range)
        
        # Controller parameters
        Kr = 10 * np.ones(model.nv)
        V_max = 1e-2
        alpha = 20
        r_norm_threshold = 1e-10
        controller = grav_pid_controller(
            "../../../Robots/kinova-gen3/gen3_2f85_fixed.urdf"
        )
        
        # Set the initial condition
        q0, v0, _ = desired_trajectory(0)

        # Set the simulation time
        ts_sim = np.linspace(0, 7.5, 75000)

        # Integrate the dynamics using pinocchio
        pos, vel = integrate(model, ts_sim, np.concatenate([q0, v0]), controller)
        
        # Recover the desired trajectory
        tau = np.zeros((len(ts_sim), 7))
        desired_pos = np.zeros((len(ts_sim), 7))
        desired_vel = np.zeros((len(ts_sim), 7))
        desired_acc = np.zeros((len(ts_sim), 7))
        for i, t in enumerate(ts_sim):
            desired_pos[i], desired_vel[i], desired_acc[i] = desired_trajectory(t)
            tau[i] = controller.update(pos[i], vel[i], desired_pos[i], desired_vel[i], desired_acc[i])
            
        tracking_error = pos - desired_pos
        tracking_error_vel = vel - desired_vel
        
        sio.savemat("simulation_data_gravpid_low_" + str(traj_id) + ".mat", {
            "ts_sim": ts_sim,
            "pos": pos,
            "vel": vel,
            "tau": tau,
            "desired_pos": desired_pos,
            "desired_vel": desired_vel,
            "desired_acc": desired_acc})
        
        traj_data = np.hstack((np.array([ts_sim]).T, pos, vel, tau))
        np.savetxt("simulation_data_gravpid_low_" + str(traj_id) + ".txt", traj_data, delimiter=" ")
        
        # raise Exception("Simulation done")
        
        # # Initialize the PyBullet simulation
        # p.connect(p.GUI)
        # p.setGravity(0, 0, -9.81)
        # robot = p.loadURDF(model_path, useFixedBase=True, basePosition=[0,0,0], baseOrientation=[0,0,0,1])
        # num_joints = p.getNumJoints(robot)
        
        # # Playback the simulation
        # for q in pos:
        #     id = 0
        #     for i in range(num_joints):
        #         joint_info = p.getJointInfo(robot, i)
        #         joint_type = joint_info[2]
        #         if joint_type == p.JOINT_FIXED:
        #             p.resetJointState(robot, i, targetValue=0)
        #         else:
        #             p.resetJointState(robot, i, targetValue=q[id])
        #             id += 1
        #     time.sleep(0.001)
            
        # input("Press Enter to continue...")
        # p.disconnect()
        
        # plt.figure()
        # for i in range(7):
        #     plt.subplot(2, 4, i+1)
        #     plt.plot(ts_sim, tracking_error[:,i], label=f"Joint {i}")
        #     plt.xlabel("Time [s]")
        #     plt.ylabel("Tracking error [rad]")
        #     plt.legend()
        # plt.savefig("tracking_error.png")
        
    

