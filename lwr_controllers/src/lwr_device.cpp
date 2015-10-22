#include <lwr_device.h>

using namespace H3D;

H3DNodeDatabase LwrDevice::database( "LwrDevice",
                                          &(newInstance<LwrDevice>),
                                          typeid( LwrDevice ),
                                          &H3DHapticsDevice::database );

LwrDevice::LwrDevice() {

}

HAPIHapticsDevice::HapticsDeviceRegistration
LwrDevice::device_registration("Lwr", &(newInstance< LwrDevice >),);

void LwrDevice::initDevice(int _thread_frequency) {
  com_thread = new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, _thread_frequency );
  com_thread->setThreadName( "Lwr com thread" );

  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

  return true;
}

void LwrDevice::updateDeviceValues(DeviceValues &dv, HAPITime dt) {

}

void LwrDevice::sendOutput(DeviceOutput &dv, HAPITime dt) {

}
