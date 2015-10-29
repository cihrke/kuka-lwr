#ifndef HAPI_CONTROLLER_H
#define HAPI_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <lwr_controllers/ArmState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <realtime_tools/realtime_publisher.h>

#include <boost/scoped_ptr.hpp>

#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/GodObjectRenderer.h>
#include "lwr_device.h"

namespace hapi_controller
{
    class HapiController: public controller_interface::KinematicChainControllerBase<hardware_interface::JointStateInterface>
    {
    public:

        HapiController();
        ~HapiController();

        bool init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);

    private:

        boost::shared_ptr< realtime_tools::RealtimePublisher< lwr_controllers::ArmState > > realtime_pub_;

        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
        boost::scoped_ptr<KDL::Jacobian> jacobian_;
        boost::scoped_ptr<KDL::Vector> gravity_;
        boost::scoped_ptr<KDL::JntArray> joint_position_;
        boost::scoped_ptr<KDL::JntArray> joint_velocity_;
        boost::scoped_ptr<KDL::JntArray> joint_acceleration_;
        boost::scoped_ptr<KDL::Wrenches> joint_wrenches_;
        boost::scoped_ptr<KDL::JntArray> joint_effort_est_;

        ros::Time last_publish_time_;
        double publish_rate_;

        KDL::Frame x_;

        //hapi stuff

        H3D::LwrDevice hd;
        //load surfaces and effects here?
    };
}

#endif
