
#ifndef __LWRDEVICE_H__
#define __LWRDEVICE_H__

#include <H3D/H3DHapticsDevice.h>

namespace H3D {

  class H3DAPI_API AnyDevice: public H3DHapticsDevice {
    public:

    AnyDevice(
      Inst< SFVec3f > _devicePosition         = 0,
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
              Inst< SFNode             > _stylus                 = 0 );

      static H3DNodeDatabase database;
    };
}

#endif
