import pybullet as p
import pybullet_data
import numpy as np

def solve_ik_urdf(
    urdf_path='devkit_base_descr.urdf',
    link_name=None,
    target_pos=None,
    target_quat=None,
    init_qpos=None,
    respect_joint_limits=True,
    max_samples=50,
    max_solver_iters=20,
    damping=0.01,
    pos_tol=5e-4,
    rot_tol=5e-3,
    pos_mask=(True, True, True),
    rot_mask=(True, True, True),
    max_step_size=0.5,
    verbose=True
):
    """
    Solve IK for a single link in a URDF using Damped Least Squares + random restarts.

    Steps:
      1. Load URDF in DIRECT PyBullet mode (collision shapes active).
      2. Reset all dofs to zero, print the link’s FK pose.
      3. For up to `max_samples` random restarts:
         – initialize q
         – iterate DLS updates (via Bullet’s Jacobian)
         – clamp steps, enforce joint limits
         – stop early if pos/rot errors < tol
      4. Warn if resulting config has self-collision.

    Returns a dict { joint_name: joint_angle } for the best found solution.
    """

    # 1) connect & load
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    body = p.loadURDF(urdf_path,
                      useFixedBase=True,
                      flags=p.URDF_USE_INERTIA_FROM_FILE,
                      physicsClientId=client)

    # 2) collect all actuated dofs
    n_joints = p.getNumJoints(body, physicsClientId=client)
    joint_indices, lower_limits, upper_limits, joint_names = [], [], [], []
    for i in range(n_joints):
        info = p.getJointInfo(body, i, physicsClientId=client)
        jtype = info[2]
        if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            joint_indices.append(i)
            joint_names.append(info[1].decode())
            ll, ul = info[8], info[9]
            lower_limits.append(ll)
            upper_limits.append(ul)
    joint_indices = np.array(joint_indices, dtype=int)
    lower_limits = np.array(lower_limits, dtype=float)
    upper_limits = np.array(upper_limits, dtype=float)
    n_dofs = len(joint_indices)

    # 3) find link index
    link_index = None
    base_name = p.getBodyInfo(body, physicsClientId=client)[0].decode()
    if link_name == base_name:
        link_index = -1
    else:
        for i in range(n_joints):
            child = p.getJointInfo(body, i, physicsClientId=client)[12].decode()
            if child == link_name:
                link_index = i
                break
    if link_index is None:
        p.disconnect(client)
        raise ValueError(f"Link '{link_name}' not in URDF.")

    # helper to apply q
    def set_q(q):
        for idx, qi in zip(joint_indices, q):
            p.resetJointState(body, idx, qi, physicsClientId=client)

    # -- Forward Kinematics at zero
    zero_q = np.zeros(n_dofs)
    set_q(zero_q)
    if link_index == -1:
        pos0, orn0 = p.getBasePositionAndOrientation(body, physicsClientId=client)
    else:
        st = p.getLinkState(body, link_index, computeForwardKinematics=True,
                            physicsClientId=client)
        pos0, orn0 = st[0], st[1]
    if verbose:
        print(f"[FK @ q=0]  pos = {pos0},  quat = {orn0}")

    # quaternion utilities
    def quat_inv(q):
        return np.array([-q[0], -q[1], -q[2], q[3]])
    def quat_mul(a, b):
        x1,y1,z1,w1 = a;  x2,y2,z2,w2 = b
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])

    target_pos = np.array(target_pos, dtype=float)
    target_quat = np.array(target_quat, dtype=float)
    best_q, best_err = None, np.inf

    # 4) IK w/ random restarts
    for sample in range(max_samples):
        if sample == 0 and init_qpos is not None:
            q = np.array(init_qpos, dtype=float)
        else:
            q = lower_limits + (upper_limits - lower_limits)*np.random.rand(n_dofs)
        set_q(q)

        for it in range(max_solver_iters):
            # current FK
            if link_index == -1:
                pos_c, orn_c = p.getBasePositionAndOrientation(body, physicsClientId=client)
            else:
                st = p.getLinkState(body, link_index, computeForwardKinematics=True,
                                    physicsClientId=client)
                pos_c, orn_c = st[0], st[1]
            pos_c = np.array(pos_c)

            # position error
            e_pos = target_pos - pos_c
            e_pos = np.where(pos_mask, e_pos, 0.0)

            # orientation error
            q_err = quat_mul(target_quat, quat_inv(np.array(orn_c)))
            ang = 2*np.arccos(np.clip(q_err[3], -1,1))
            if abs(ang) < 1e-6:
                axis = np.zeros(3)
            else:
                axis = q_err[:3]/np.sin(ang/2)
            e_rot = axis*ang
            e_rot = np.where(rot_mask, e_rot, 0.0)

            # stack
            e = np.concatenate([e_pos, e_rot])
            err_norm = np.linalg.norm(e)

            # check tol
            if (np.all(np.abs(e_pos[pos_mask]) < pos_tol) and
                np.all(np.abs(e_rot[rot_mask]) < rot_tol)):
                if verbose:
                    print(f"Solved @ sample {sample}, iter {it}, err {err_norm:.3e}")
                # collision check
                contacts = p.getClosestPoints(body, body, distance=0, physicsClientId=client)
                if contacts:
                    print("⚠ Warning: solution in self-collision!")
                p.disconnect(client)
                return {nm: angle for nm, angle in zip(joint_names, q)}

            # compute Jacobian
            zeros = [0.]*n_dofs
            jac_tup = p.calculateJacobian(
                body, link_index, [0,0,0],
                q.tolist(), zeros, zeros,
                physicsClientId=client
            )
            Jlin = np.array(jac_tup[0])
            Jrot = np.array(jac_tup[1])
            J6  = np.vstack((Jlin, Jrot))
            mask = np.array(list(pos_mask)+list(rot_mask))
            Jm  = J6[mask]

            # DLS update
            JJt = Jm.dot(Jm.T)
            W   = np.linalg.inv(JJt + (damping**2)*np.eye(JJt.shape[0]))
            dq  = Jm.T.dot(W).dot(e[mask])

            # clamp step
            nm = np.linalg.norm(dq)
            if nm > max_step_size:
                dq = dq / nm * max_step_size

            # update + limit
            q = q + dq
            if respect_joint_limits:
                q = np.clip(q, lower_limits, upper_limits)
            set_q(q)

            if err_norm < best_err:
                best_err, best_q = err_norm, q.copy()

        if verbose:
            print(f"Sample {sample} done, best_err = {best_err:.3e}")

    # no exact solution found
    if verbose:
        print(f"IK did not converge. Best err = {best_err:.3e}")
    p.disconnect(client)
    if best_q is None:
        raise RuntimeError("IK solver failed completely.")
    return {nm: angle for nm, angle in zip(joint_names, best_q)}

solution = solve_ik_urdf(
    urdf_path='devkit_base_descr.urdf',
    link_name='link6',
    target_pos=[-0.5, 0, 1.5],
    target_quat=[0, 1, 0, 0]
)
print(solution)