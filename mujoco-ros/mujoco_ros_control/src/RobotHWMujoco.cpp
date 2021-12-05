#include <mujoco_ros_control/RobotHWMujoco.h>

RobotHWMujoco::RobotHWMujoco(const mjModel &m) {
    assert(m.njnt >= 0);
    const auto n = (size_t) m.njnt;
    size_t k = (size_t) 0;
    // count relevant joints
    for (size_t i = 0; i < n; ++i) {
        const auto joint_type = m.jnt_type[i];
        std::cout << i << ": " << m.jnt_type[i] << std::endl;
        if (joint_type == 0  || joint_type == 2  || joint_type == mjJNT_BALL) {
            std::cout << i << ": deleted" << std::endl;
            continue;
        }else{
          k++;
        }}
    // const auto n = (size_t) k;

    cmd.resize(k, 0.0);
    pos.resize(k, 0.0);
    vel.resize(k, 0.0);
    eff.resize(k, 0.0);
    qadr.resize(k, 0);
    vadr.resize(k, 0);

    std::cout << "nu: " << m.nq << std::endl;

    for (size_t i = 0; i < n; ++i) {

        const auto joint_type = m.jnt_type[i];
	std::cout << i << ": type: " << joint_type << std::endl;
        if (joint_type == 0  || joint_type == 2 || joint_type == mjJNT_BALL) {
            const auto joint_name = mj_id2name(&m, mjOBJ_JOINT, i);
            std::cout << i << ": deleted: " << joint_name << std::endl;
            continue;
        }

        const auto joint_name = mj_id2name(&m, mjOBJ_JOINT, i);
        qadr[i] = (size_t) m.jnt_qposadr[i];
        vadr[i] = (size_t) m.jnt_dofadr[i];
	    std::cout << joint_name << std::endl;

        hardware_interface::JointStateHandle state_handle_a(joint_name, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointHandle eff_handle_a(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_eff_interface.registerHandle(eff_handle_a);

    }
    std::cout << "finished"  << std::endl;


    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_eff_interface);

    std::cout << "finished2"  << std::endl;
}

void RobotHWMujoco::read(const mjData &d) {
    for (size_t i = 0; i < qadr.size(); ++i) {
        pos[i] = d.qpos[qadr[i]];
        vel[i] = d.qvel[vadr[i]];
        eff[i] = d.qfrc_applied[vadr[i]];

        // std::cout << i << ": effort: " << eff[i] << std::endl;
        // std::cout << i << ": pos: " << d.qpos[qadr[i]] << std::endl;
        // std::cout << i << ": vel: " << d.qvel[vadr[i]] << std::endl;

        if (compensate_bias && !show_full_torques) {
            eff[i] -= d.qfrc_bias[vadr[i]];
        }
    }
}

void RobotHWMujoco::write(mjData &d) {
    for (size_t i = 0; i < vadr.size(); ++i) {
        d.qfrc_applied[vadr[i]] = cmd[i];
        // std::cout << i << ": Joint name: " << (std::string)d.mocap_pos << std::endl;

        // std::cout << i << ": cmd: " << cmd[i] << std::endl;
        // std::cout << i << ": cmd: " << d.qfrc_applied[vadr[i]] << std::endl;
        // d.qfrc_applied[vadr[i]] = (0.2 - pos[i]) * 100 - d.qvel[vadr[i]] *5;

        if (compensate_bias) {
            d.qfrc_applied[vadr[i]] += d.qfrc_bias[vadr[i]] * bias_error;
        }
    }
}
