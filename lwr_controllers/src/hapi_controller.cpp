#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <lwr_controllers/hapi_controller.h>
#include <kdl_conversions/kdl_msg.h>
#include <utils/pseudo_inversion.h>
#include <control_toolbox/filters.h>

using namespace H3D;
using namespace HAPI;

namespace hapi_controller
{
    HapiController::HapiController() {}
    HapiController::~HapiController() 
    {
        hd.releaseDevice();    
    }

    bool HapiController::init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, n);

        // get publishing period
        if (!nh_.getParam("publish_rate", publish_rate_)) {
            ROS_ERROR("Parameter 'publish_rate' not set");
            return false;
        }

        realtime_pub_.reset(new realtime_tools::RealtimePublisher<lwr_controllers::ArmState>(nh_,"arm_state",4));
        realtime_pub_->msg_.est_ext_torques.resize(kdl_chain_.getNrOfJoints());

        gravity_.reset(new KDL::Vector(0.0, 0.0, -9.81)); // TODO: compute from actual robot position (TF?)
        id_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_, *gravity_));
        jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_solver_pos_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        fk_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        jacobian_.reset(new KDL::Jacobian(kdl_chain_.getNrOfJoints()));
        joint_position_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_velocity_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_acceleration_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_wrenches_.reset(new KDL::Wrenches(kdl_chain_.getNrOfJoints()));
        joint_effort_est_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));

        if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
          //cerr << hd.getLastErrorMsg() << endl;
          return false;
        }

        return true;
    }

    void HapiController::starting(const ros::Time& time)
    {
        last_publish_time_ = time;
        for (unsigned i = 0; i < joint_handles_.size(); i++){
            (*joint_position_)(i) = joint_handles_[i].getPosition();
            (*joint_velocity_)(i) = joint_handles_[i].getVelocity();
            (*joint_acceleration_)(i) = 0;
        }
        
        hd.enableDevice();

        ROS_INFO("started");
    }

    void HapiController::update(const ros::Time& time, const ros::Duration& period)
    {
        // limit rate of publishing
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

            if (realtime_pub_->trylock()) {
                // we're actually publishing, so increment time
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

                for (unsigned i = 0; i < joint_handles_.size(); i++) {
                    float acceleration = filters::exponentialSmoothing((joint_handles_[i].getVelocity() - (*joint_velocity_)(i))/period.toSec(), (*joint_acceleration_)(i), 0.2);
                    (*joint_position_)(i) = joint_handles_[i].getPosition();
                    (*joint_velocity_)(i) = joint_handles_[i].getVelocity();
                    (*joint_acceleration_)(i) = acceleration;
                }

                //TODO: update hapi device and receive output

                //maybe acceleration?
                for(int i=0; i < joint_handles_.size(); i++)
                {
                  joint_msr_states_.q(i) = joint_handles_[i].getPosition();
                  joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
                }
                KDL::JntArrayVel joint_velocity(joint_msr_states_.qdot);
                fk_solver_pos_->JntToCart(joint_msr_states_.q, p_);
                fk_solver_vel_->JntToCart(joint_velocity, v_);

                Vec3f pos = Vec3f((float)p_.p(0), (float)p_.p(1), (float)p_.p(2));
                Vec3f vel = Vec3f((float)v_.p.v(0), (float)v_.p.v(1), (float)v_.p.v(2));  // TODO: maybe v_.p.p?

                //correct? fix type casting?
                SFRotation rot;
                rot.setValueFromVoidPtr((float *)p_.M.data, 9);

                //time? const ros::Time& time
                hd.updateDeviceValues(pos, rot, vel);

                Vec3 force = hd.getForce();
                Vec3 torque = hd.getTorque();

                ROS_INFO("HAPI Output force x: %f, y: %f, z: %f", force.x, force.y, force.z);
                ROS_INFO("HAPI Output torque x: %f, y: %f, z: %f", torque.x, torque.y, torque.z);

                // Compute Dynamics
                int ret = id_solver_->CartToJnt(*joint_position_,
                                                *joint_velocity_,
                                                *joint_acceleration_,
                                                *joint_wrenches_,
                                                *joint_effort_est_);
                if (ret < 0) {
                    ROS_ERROR("KDL: inverse dynamics ERROR");
                    realtime_pub_->unlock();
                    return;
                }

                realtime_pub_->msg_.header.stamp = time;
                for (unsigned i=0; i<joint_handles_.size(); i++) {
                    realtime_pub_->msg_.est_ext_torques[i] = joint_handles_[i].getEffort() - (*joint_effort_est_)(i);
                }

                // Compute cartesian wrench on end effector
                ret = jac_solver_->JntToJac(*joint_position_, *jacobian_);
                if (ret < 0) {
                    ROS_ERROR("KDL: jacobian computation ERROR");
                    realtime_pub_->unlock();
                    return;
                }

                Eigen::MatrixXd jinv;
                pseudo_inverse(jacobian_->data.transpose(),jinv,true);

                KDL::Wrench wrench;

                for (unsigned int i = 0; i < 6; i++) {
                    for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); j++) {
                        wrench[i] += jinv(i,j) * realtime_pub_->msg_.est_ext_torques[j];
                    }
                }

                tf::wrenchKDLToMsg(wrench, realtime_pub_->msg_.est_ee_wrench_base);

                // Transform cartesian wrench into tool reference frame
                KDL::Frame tool_frame;
                fk_solver_pos_->JntToCart(*joint_position_, tool_frame);
                KDL::Wrench tool_wrench = tool_frame * wrench;

                tf::wrenchKDLToMsg(tool_wrench, realtime_pub_->msg_.est_ee_wrench);

                realtime_pub_->unlockAndPublish();
            }
        }
    }

    void HapiController::stopping(const ros::Time& time)
    {
        hd.disableDevice();
    }

}

PLUGINLIB_EXPORT_CLASS(hapi_controller::HapiController, controller_interface::ControllerBase)