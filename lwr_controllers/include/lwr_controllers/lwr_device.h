
#ifndef __LWRDEVICE_H__
#define __LWRDEVICE_H__

#include <H3D/H3DHapticsDevice.h>
#include <H3D/MFString.h>
#include <H3D/SFString.h>
#include <H3D/SFDouble.h>
#include <H3D/MFVec3f.h>

namespace H3D {

  class H3DAPI_API LwrDevice: public H3DHapticsDevice {
    public:

    LwrDevice(
      Inst< SFVec3f            > _devicePosition         = 0,
      Inst< SFRotation         > _deviceOrientation      = 0,
      Inst< TrackerPosition    > _trackerPosition        = 0,
      Inst< TrackerOrientation > _trackerOrientation     = 0,
      Inst< PosCalibration     > _positionCalibration    = 0,
      Inst< OrnCalibration     > _orientationCalibration = 0,
      Inst< SFVec3f            > _proxyPosition          = 0,
      Inst< WeightedProxy      > _weightedProxyPosition  = 0,
      Inst< SFFloat            > _proxyWeighting         = 0,
      Inst< SFBool             > _mainButton             = 0,
      Inst< SFBool             > _secondaryButton        = 0,
      Inst< SFInt32            > _buttons                = 0,
      Inst< SFVec3f            > _force                  = 0,
      Inst< SFVec3f            > _torque                 = 0,
      Inst< SFInt32            > _inputDOF               = 0,
      Inst< SFInt32            > _outputDOF              = 0,
      Inst< SFInt32            > _hapticsRate            = 0,
      Inst< SFInt32            > _desiredHapticsRate     = 0,
      Inst< SFNode             > _stylus                 = 0 ) :
      com_thread( NULL ),
      com_func_cb_handle( -1 ) {};

    virtual ErrorCode initDevice( int _thread_frequency = 1000 );
    virtual void initialize();
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt )
    virtual void sendOutput( DeviceOutput &dv, HAPITime dt );

    /// Callback function for communication thread
    static H3DUtil::PeriodicThread::CallbackCode com_func( void *data );

    /// Callback handle to the com_func callback that is set up
    int com_func_cb_handle;

    /// Thread used to do communication with the haptics device
    H3DUtil::PeriodicThread *com_thread;

    /// Lock for exchanging data with the communication thread.
    H3DUtil::MutexLock com_lock;

    /// The current device values updated in the communicataion thread.
    /// Access to this structure must be contained within locking with
    /// com_lock.
    DeviceValues current_values;

    static H3DNodeDatabase database;
  };
}

#endif
