#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <lwr_controllers/hapi_controller.h>
#include <kdl_conversions/kdl_msg.h>
#include <utils/pseudo_inversion.h>
#include <control_toolbox/filters.h>

//HAPI stuff
#include <HAPI/GodObjectRenderer.h>
#include <HAPI/HapticPrimitive.h>
#include <HAPI/HAPIForceEffect.h>
#include <HAPI/HAPISurfaceObject.h>
#include <HAPI/FrictionSurface.h>

//effects
#include <HAPI/HapticSpring.h>
#include <HAPI/HapticForceField.h>
#include <HAPI/HapticRotationalSpring.h>
#include <HAPI/HapticViscosity.h>
#include <HAPI/HapticPositionFunctionEffect.h>
#include <HAPI/HapticTimeFunctionEffect.h>
#include <HAPI/HapticShapeConstraint.h>


using namespace HAPI;

namespace hapi_controller
{
	HapiController::HapiController() {}
	HapiController::~HapiController()
	{
		hd.releaseDevice();
	}
	
	bool HapiController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
		
		// get publishing period
		if (!nh_.getParam("publish_rate", publish_rate_)) {
			ROS_ERROR("Parameter 'publish_rate' not set");
			return false;
		}
		
		realtime_pub_.reset(new realtime_tools::RealtimePublisher<lwr_controllers::ArmState>(nh_,"arm_state",4));
		realtime_pub_->msg_.est_ext_torques.resize(kdl_chain_.getNrOfJoints());

		effects_sub_ = nh_.subscribe("effects", 1, &HapiController::effectsCallback, this);
		primitives_sub_ = nh_.subscribe("primitives", 1, &HapiController::primitivesCallback, this);
		
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

        hd.setForceLimit(forceLimit);
        hd.setTorqueLimit(torqueLimit);
		
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
		
		hd.setHapticsRenderer(new GodObjectRenderer);
		//set stylus for visual representation
		
		hd.enableDevice();
		
		ROS_INFO("started");
	}
	
	void HapiController::update(const ros::Time& time, const ros::Duration& period)
	{
		// limit rate of publishing
		if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){			
			
			// we're actually publishing, so increment time
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
			
			for (unsigned i = 0; i < joint_handles_.size(); i++) {
				float acceleration = filters::exponentialSmoothing((joint_handles_[i].getVelocity() - (*joint_velocity_)(i))/period.toSec(), (*joint_acceleration_)(i), 0.2);
				(*joint_position_)(i) = joint_handles_[i].getPosition();
				(*joint_velocity_)(i) = joint_handles_[i].getVelocity();
				(*joint_acceleration_)(i) = acceleration;
			}
			
			//TODO: maybe acceleration?
			for(int i=0; i < joint_handles_.size(); i++)
			{
				joint_msr_states_.q(i) = joint_handles_[i].getPosition();
				joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			}
			
			KDL::JntArrayVel joint_velocity(joint_msr_states_.qdot);
			
			fk_solver_pos_->JntToCart(joint_msr_states_.q, p_);
			fk_solver_vel_->JntToCart(joint_velocity, v_);
			
			//update values for HAPI
			hapi_pos = Vec3((float)p_.p(0), (float)p_.p(1), (float)p_.p(2));
			hapi_vel = Vec3((float)v_.p.v(0), (float)v_.p.v(1), (float)v_.p.v(2));  // TODO: maybe v_.p.p?
			
			//TODO: correct?
			hapi_rot = Rotation(Vec3((float)p_.M.GetRot()[0], (float)p_.M.GetRot()[1],
                                     (float)p_.M.GetRot()[2]),(float)p_.M.GetRot().Norm());

			//send data to hapi
			hd.updateValues(hapi_pos, hapi_vel, hapi_rot);
			
			//get values from HAPI
			Vec3 force = hd.getForce();
			Vec3 torque = hd.getTorque();
			
			std::cout << "received force: " << force << " torque: " << torque << std::endl; 
			
			(*joint_wrenches_)[kdl_chain_.getNrOfJoints()-1].force = KDL::Vector(force.x, force.y, force.z);
			(*joint_wrenches_)[kdl_chain_.getNrOfJoints()-1].torque = KDL::Vector(torque.x, torque.y, torque.z);

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
			
			for(size_t i=0; i<joint_handles_.size(); i++) {
				joint_handles_[i].setCommand((*joint_effort_est_)(i));
			}	
			
			if (true && realtime_pub_->trylock()) {
				realtime_pub_->msg_.header.stamp = time;
				for (unsigned i=0; i<joint_handles_.size(); i++) {
					realtime_pub_->msg_.est_ext_torques[i] = (*joint_effort_est_)(i);
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
				
				// Transform cartesian wrench into tool reference frame
				KDL::Frame tool_frame;
				fk_solver_pos_->JntToCart(*joint_position_, tool_frame);
				KDL::Wrench tool_wrench = tool_frame * wrench;
				
				tf::wrenchKDLToMsg(wrench, realtime_pub_->msg_.est_ee_wrench_base);
				
				tf::wrenchKDLToMsg(tool_wrench, realtime_pub_->msg_.est_ee_wrench);
				
				realtime_pub_->unlockAndPublish();
			}		
		}
	}
	
	void HapiController::stopping(const ros::Time& time)
	{
		hd.disableDevice();
	}

    //TODO: change to accept arrays
    void HapiController::effectsCallback(const lwr_controllers::Effect::ConstPtr &msg){

        std::string type = msg->type;      

        if(type == "Spring") {
            Vec3 pos = Vec3(msg->position.x, msg->position.y, msg->position.z);
            HapticSpring *spring_effect = new HapticSpring(pos, msg->spring_constant);
            hd.addEffect(spring_effect);
	    std::cout << "received Spring pos: " << pos << " strength: " << msg->spring_constant << std::endl;
        } else if(type == "RotationalSpring") {
            Vec3 axis = Vec3(msg->position.x, msg->position.y, msg->position.z);
            HapticRotationalSpring *rotspring = new HapticRotationalSpring(axis, msg->spring_constant, msg->damping);
            hd.addEffect(rotspring);
        } else if(type == "ForceField") {
            Vec3 force = Vec3(msg->position.x, msg->position.y, msg->position.z);
            Vec3 torque = Vec3(msg->torque.x, msg->torque.y, msg->torque.z);
            HapticForceField *forcefield = new HapticForceField(force, torque);
            hd.addEffect(forcefield);
        } else if(type == "Viscosity") {
            HapticViscosity *viscosity = new HapticViscosity (msg->position.x, msg->position.y, msg->position.z);
            hd.addEffect(viscosity);
        } else {
            ROS_ERROR("Wrong or no effect specified!");
            return;
        }

        hd.transferObjects();

    }

    //TODO: change to accept arrays
    void HapiController::primitivesCallback(const lwr_controllers::Primitive::ConstPtr &msg){

        HAPISurfaceObject *surface;
        HapticPrimitive *primitive;

        std::string type_surface = msg->surface;
        std::string type = msg->type;

        if(type_surface == "FrictionSurface"){
            surface = new FrictionSurface(msg->surface_parameters[0], msg->surface_parameters[1],
                                          msg->surface_parameters[2], msg->surface_parameters[3]);
        } else {
            ROS_ERROR("Wrong or no surface specified!");
            return;
        }

        Vec3 pos = Vec3(msg->position.x, msg->position.y, msg->position.z);

        if(type == "Sphere"){
            primitive = new HapticPrimitive(new Collision::Sphere(pos, msg->radius), surface );
            hd.addShape(primitive);
            std::cout << "received Sphere pos: " << pos << " radius: " << msg->radius << std::endl;
        } else if(type == "Cylinder") {
            primitive = new HapticPrimitive(new Collision::Cylinder(msg->radius, msg->length), surface);
            hd.addShape(primitive);
        } else if(type == "LineSegment") {
            Vec3 end = Vec3(msg->end_position.x, msg->end_position.y, msg->end_position.z);
            primitive = new HapticPrimitive(new Collision::LineSegment (pos, end), surface );
            hd.addShape(primitive);
        } else if(type == "Plane") {
            Vec3 end = Vec3(msg->end_position.x, msg->end_position.y, msg->end_position.z);
            primitive = new HapticPrimitive(new Collision::Plane(pos, end), surface );
            hd.addShape(primitive);
        } else if(type == "Point"){
            primitive = new HapticPrimitive(new Collision::Point(pos), surface );
            hd.addShape(primitive);
        } else if(type == "Triangle"){
            Vec3 b = Vec3(msg->end_position.x, msg->end_position.y, msg->end_position.z);
            Vec3 c = Vec3(msg->third_vertex.x, msg->third_vertex.y, msg->third_vertex.z);
            primitive = new HapticPrimitive(new Collision::Triangle(pos, b, c), surface );
            hd.addShape(primitive);
        } else {
            ROS_ERROR("Wrong or no primitive specified!");
            return;
        }

        hd.transferObjects();

    }

    //remove get functions?
	
	Vec3 HapiController::getPos() {
		return hapi_pos;
	}
	
	Vec3 HapiController::getVel() {
		return hapi_vel;
	}
	
	Rotation HapiController::getRot() {
		return hapi_rot;
	}
	
}

PLUGINLIB_EXPORT_CLASS(hapi_controller::HapiController, controller_interface::ControllerBase)
