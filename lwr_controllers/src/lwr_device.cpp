#include <lwr_controllers/lwr_device.h>
#ifdef HAVE_DHDAPI
#include <HAPI/LwrHapticsDevice.h>
#endif

using namespace H3D;

H3DNodeDatabase LwrDevice::database( "LwrDevice",
                                    &(newInstance<LwrDevice>),
                                    typeid( LwrDevice ),
                                    &H3DHapticsDevice::database );

/// Constructor.
LwrDevice::LwrDevice(
               Inst< SFVec3f         > _devicePosition,
               Inst< SFRotation      > _deviceOrientation,
               Inst< TrackerPosition > _trackerPosition,
               Inst< TrackerOrientation > _trackerOrientation,
               Inst< SFMatrix4f      > _positionCalibration,
               Inst< SFRotation      > _orientationCalibration,
               Inst< SFVec3f         > _proxyPosition,
               Inst< WeightedProxy   > _weightedProxyPosition,
               Inst< SFFloat         > _proxyWeighting,
               Inst< SFBool          > _mainButton,
               Inst< SFBool          > _secondaryButton,
               Inst< SFInt32         > _buttons,
               Inst< SFVec3f         > _force,
               Inst< SFVec3f         > _torque,
               Inst< SFInt32         > _inputDOF,
               Inst< SFInt32         > _outputDOF,
               Inst< SFInt32         > _hapticsRate,
               Inst< SFInt32         > _desiredHapticsRate,
               Inst< SFNode          > _stylus,
               Inst< SFHapticsRendererNode > _hapticsRenderer,
               Inst< MFVec3f         > _proxyPositions,
               Inst< SFBool          > _followViewpoint,
               Inst< GravityComp     > _useGravityCompensation,
               Inst< Reset           > _reset,
               Inst< WaitReset       > _waitForReset,
               Inst< EffectorMass    > _endEffectorMass,
               Inst< Brakes          > _useBrakes,
               Inst< SFInt32         > _deviceType ) :
  H3DHapticsDevice( _devicePosition, _deviceOrientation, _trackerPosition,
              _trackerOrientation, _positionCalibration,
              _orientationCalibration, _proxyPosition,
              _weightedProxyPosition, _proxyWeighting, _mainButton,
                    _secondaryButton, _buttons,
              _force, _torque, _inputDOF, _outputDOF, _hapticsRate,
              _desiredHapticsRate, _stylus,_hapticsRenderer, _proxyPositions,
              _followViewpoint ) {

  type_name = "LwrDevice";
  database.initFields( this );

  deviceType->setValue( -1, id );
}


H3DHapticsDevice::ErrorCode LwrDevice::initDevice() {
  HAPI::HAPIHapticsDevice::ErrorCode e = H3DHapticsDevice::initDevice();
  H3DHapticsDevice::setHapticsRenderer( new GodObjectRenderer );
  //set stylus for visual representation

  //set surfaces and effects here!

  // Creating a default surface.
  HAPISurfaceObject * my_surface = new FrictionSurface();
  // Creating a sphere with radius 0.05 and center in (0, 0, 0). Units in m.
  HapticPrimitive *my_haptic_sphere =  new HapticPrimitive(
    new Collision::Sphere( Vec3( 0, 0, 0 ), 0.05 ),
    my_surface );
  // Add the shape to be rendered on the device.
  H3DHapticsDevice::addShape( my_haptic_sphere );
  // Transfer objects (shapes) to the haptics loop.
  H3DHapticsDevice::transferObjects();

  // The spring effect with a position and spring_constant input by the user.
  HapticSpring *spring_effect = new HapticSpring( Vec3( x, y, z ),
                                                  spring_constant );
  // Add the effect to the haptics device.
  H3DHapticsDevice::addEffect( spring_effect );
  // Send the effect to the haptics loop and from now on it will be used to
  // send forces to the device.
  H3DHapticsDevice::transferObjects();

  return e;
}

H3DHapticsDevice::ErrorCode updateDeviceValues(position_,
                                               rotation_,
                                               velocity_) {

  LwrDevice::DeviceValues->position = position_;
  LwrDevice::DeviceValues->rotation = rotation_;
  LwrDevice::DeviceValues->velocity = velocity_;

  //fix
  double dt = 0;

  HAPI::HAPIHapticsDevice::ErrorCode e = H3DHapticsDevice::updateDeviceValues(LwrDevice::DeviceValues, dt);

  return e;
}

Vec3f getForce() {
  return LwrDevice::DeviceValues->force;
}

Vec3f getTorque() {
  return LwrDevice::DeviceValues->torque;
}

H3DHapticsDevice::ErrorCode LwrDevice::releaseDevice() {
  HAPI::HAPIHapticsDevice::ErrorCode e = H3DHapticsDevice::releaseDevice();
  deviceType->setValue( -1, id );
  return e;
}
