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
    base = ic.stringToProxy("AndorNetwork:tcp -h 192.168.12.39 -p 10000")
    andor = AndorNetwork.AndorPrx.checkedCast(base)
    if not andor:
        raise RuntimeError("Invalid proxy")
 
    andor.Initialize("")

    detector = andor.GetDetector()
    print(detector)
    arr_size = detector.xpixels  * detector.ypixels;

    andor.SetAcquisitionMode(AndorNetwork.AcquisitionMode.SINGLE_SCAN)

    andor.SetReadMode(AndorNetwork.ReadMode.IMAGE)

    andor.SetTriggerMode(AndorNetwork.TriggerMode.INTERNAL)

    det = andor.GetDetector()
    print(f"GetDetector: xpixels = {det.xpixels} ypixels = {det.ypixels}")

    andor.SetImage(1, 1, 1, det.xpixels, 1, det.ypixels)

    andor.SetExposureTime(0.01)

    timing = andor.GetAcquisitionTimings()
    print(f"GetAcquisitionTimings: exposure = {timing.exposure} accumulate = {timing.accumulate} kinetic = {timing.kinetic}")

    andor.PrepareAcquisition()

    # Perform Acquisition
    andor.StartAcquisition()

    andor.WaitForAcquisition()

    imageSize = det.xpixels * det.ypixels
    image_struct = andor.GetImages16(1, 1, imageSize)

    img = np.array(image_struct.arr).reshape((det.xpixels, det.ypixels))

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
