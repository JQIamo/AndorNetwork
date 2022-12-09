import sys
import traceback
import Ice
import time
import numpy as np
import AndorNetwork
 
status = 0
ic = None
try:
    ic = Ice.initialize(sys.argv)
    base = ic.stringToProxy("AndorNetwork:tcp -h 192.168.12.39 -p 5566")
    andor = AndorNetwork.AndorPrx.checkedCast(base)
    if not andor:
        raise RuntimeError("Invalid proxy")
 
    andor.Initialize("")

    andor.AbortAcquisition()

    def _check_ice_enum(ice_enum_type, mask):
        return [name for flag, name in ice_enum_type._enumerators.items() if flag & mask]

    andor_capabilities = andor.GetCapabilities()

    model = AndorNetwork.AndorCap_CameraType.valueOf(andor_capabilities.ulCameraType)

    acq_caps = _check_ice_enum(AndorNetwork.AndorCap_AcquisitionMode, andor_capabilities.ulAcqModes)
    read_caps = _check_ice_enum(AndorNetwork.AndorCap_ReadMode, andor_capabilities.ulReadModes)
    trig_caps = _check_ice_enum(AndorNetwork.AndorCap_TriggerMode, andor_capabilities.ulTriggerModes)
    pixmode = _check_ice_enum(AndorNetwork.AndorCap_PixelMode, andor_capabilities.ulPixelMode)
    setfuncs = _check_ice_enum(AndorNetwork.AndorCap_SetFunction, andor_capabilities.ulSetFunctions)
    getfuncs = _check_ice_enum(AndorNetwork.AndorCap_GetFunction, andor_capabilities.ulGetFunctions)
    features = _check_ice_enum(AndorNetwork.AndorCap_Feature, andor_capabilities.ulFeatures)
    emgain_caps = _check_ice_enum(AndorNetwork.AndorCap_EMGain, andor_capabilities.ulEMGainCapability)

    print(f"Camera Capabilities")
    print(f"   acq_caps: {acq_caps}")
    print(f"   read_caps: {read_caps}")
    print(f"   trig_caps: {trig_caps}")
    print(f"   pixmode: {pixmode}")
    print(f"   model: {model}")
    print(f"   set funcs: {setfuncs}")
    print(f"   get funcs: {getfuncs}")
    print(f"   features: {features}")
    print(f"   emgain_caps: {emgain_caps}")

    xpixels, ypixels = (lambda x: (x.xpixels, x.ypixels))(andor.GetDetector())
    print(f"GetDetector: xpixels = {xpixels} ypixels = {ypixels}")

    temperature, temperature_status = (lambda x: (x.temperature, x.status))(andor.GetTemperatureF())
    print(f"GetTemperatureF: temperature = {temperature}, status = {temperature_status}")

    andor.SetShutter(1, AndorNetwork.ShutterMode.PERMANENTLY_OPEN, 0, 0)

    andor.SetAcquisitionMode(AndorNetwork.AcquisitionMode.SINGLE_SCAN)

    andor.SetReadMode(AndorNetwork.ReadMode.IMAGE)

    andor.SetTriggerMode(AndorNetwork.TriggerMode.INTERNAL)

    andor.SetImage(1, 1, 1, xpixels, 1, ypixels)

    andor.SetExposureTime(0.01)

    exposure, accumulate, kinetic = (lambda x: (x.exposure, x.accumulate, x.kinetic))(andor.GetAcquisitionTimings())
    print(f"GetAcquisitionTimings: exposure = {exposure} accumulate = {accumulate} kinetic = {kinetic}")

    andor.PrepareAcquisition()

    # Perform Acquisition
    andor.StartAcquisition()

    andor.WaitForAcquisition()

    imageSize = xpixels * ypixels
    image_struct = andor.GetImages16(1, 1, imageSize)

    img = np.array(image_struct.arr).reshape((xpixels, ypixels))

    print(img)
except:
    traceback.print_exc()
    status = 1
 
if ic:
    # Clean up
    try:
        ic.destroy()
    except:
        traceback.print_exc()
        status = 1
 
sys.exit(status)
