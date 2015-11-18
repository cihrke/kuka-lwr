#ifndef HAPI_CONTROLLER_H
#define HAPI_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <lwr_controllers/ArmState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <realtime_tools/realtime_publisher.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/LwrHapticsDevice.h>
#include <HAPI/HapticPrimitive.h>
#include <H3DUtil/H3DUtil.h>
#include <HAPI/HAPIForceEffect.h>
#include <HAPI/HAPISurfaceObject.h>
#include <HAPI/HapticSpring.h>
#include <HAPI/FrictionSurface.h>

#include <lwr_controllers/Effect.h>
#include <lwr_controllers/Primitive.h>


using namespace HAPI;

namespace hapi_controller
{
    class HapiController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
    {
    public:

        HapiController();
        ~HapiController();

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
        Vec3 getPos();
        Vec3 getVel();
        Rotation getRot();

    private:

        boost::shared_ptr< realtime_tools::RealtimePublisher< lwr_controllers::ArmState > > realtime_pub_;

        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_pos_;
        boost::scoped_ptr<KDL::ChainFkSolverVel> fk_solver_vel_;
        boost::scoped_ptr<KDL::Jacobian> jacobian_;
        boost::scoped_ptr<KDL::Vector> gravity_;
        boost::scoped_ptr<KDL::JntArray> joint_position_;
        boost::scoped_ptr<KDL::JntArray> joint_velocity_;
        boost::scoped_ptr<KDL::JntArray> joint_acceleration_;
        boost::scoped_ptr<KDL::Wrenches> joint_wrenches_;
        boost::scoped_ptr<KDL::JntArray> joint_effort_est_;

        ros::Time last_publish_time_;
        double publish_rate_;

        KDL::Frame p_;
        KDL::FrameVel v_;

        //hapi stuff

        void effectsCallback(const lwr_controllers::Effect::ConstPtr &msg);
        ros::Subscriber effects_sub;

        void primitivesCallback(const lwr_controllers::Primitive::ConstPtr &msg);
        ros::Subscriber primitives_sub;

        HAPI::LwrHapticsDevice hd;

        Vec3 hapi_pos;
        Vec3 hapi_vel;
        Rotation hapi_rot;

        HapticPrimitive   *primitives;
        HAPISurfaceObject *surfaces;
        HAPIForceEffect   *force_effects;
    };
}

#endif
