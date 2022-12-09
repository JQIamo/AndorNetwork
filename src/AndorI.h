#include <cinttypes>
#include <vector>

#include "windows.h"
#include "Andor.h"
#include "atmcd32d.h" // Andor functions

namespace AndorNetwork {
    class AndorI : public Andor {
    private:
        void checkErr(int _errno) {
            if (_errno != DRV_SUCCESS) {
                AndorError e;
                e.errNo = static_cast<AndorErrNo>(_errno);
                throw e;
            }
        }
        void checkErr(AndorErrNo _errno) {
            if (_errno != AndorErrNo::SUCCESS) {
                AndorError e;
                e.errNo = _errno;
                throw e;
            }
        }
    public:
        //
        // ----- Manually added functions -----
        // 

        std::vector<int32_t> GetAcquiredData(int32_t size, const Ice::Current&) {
            std::vector<int32_t> ret (size);
            checkErr(::GetAcquiredData(
                        reinterpret_cast<long *>(ret.data()),  // zeroo-ice doesn't support unsigned types
                        size)
                    );

            return ret;
        }

        std::vector<int16_t> GetAcquiredData16(int32_t size, const Ice::Current&) {
            std::vector<int16_t> ret (size);
            checkErr(::GetAcquiredData16(
                        reinterpret_cast<uint16_t *>(ret.data()),  // zeroo-ice doesn't support unsigned types
                        size)
                    );

            return ret;
        }

        Ret_GetImages GetImages(int32_t first, int32_t last, int32_t size, const Ice::Current&) {
            Ret_GetImages ret;
            long validfirst, validlast;

            ret.arr.resize(size);
            checkErr(::GetImages(
                        first,
                        last,
                        reinterpret_cast<long *>(ret.arr.data()),
                        size,
                        &validfirst,
                        &validlast)
                    );

            ret.validfirst = validfirst; 
            ret.validlast = validlast; 

            return ret;
        }

        Ret_GetImages16 GetImages16(int32_t first, int32_t last, int32_t size, const Ice::Current&) {
            Ret_GetImages16 ret;
            long validfirst, validlast;

            ret.arr.resize(size);
            checkErr(::GetImages16(
                        first,
                        last,
                        reinterpret_cast<uint16_t *>(ret.arr.data()),
                        size,
                        &validfirst,
                        &validlast)
                    );

            ret.validfirst = validfirst; 
            ret.validlast = validlast; 

            return ret;
        }

        std::vector<int32_t> GetMostRecentImage(int32_t size, const Ice::Current&) {
            std::vector<int32_t> ret (size);

            checkErr(::GetMostRecentImage(
                        reinterpret_cast<long *>(ret.data()),
                        size
                        )
                    );

            return ret;
        }

        std::vector<int16_t> GetMostRecentImage16(int32_t size, const Ice::Current&) {
            std::vector<int16_t> ret (size);

            checkErr(::GetMostRecentImage16(
                        reinterpret_cast<WORD *>(ret.data()),
                        size
                        )
                    );

            return ret;
        }

        std::vector<int32_t> GetOldestImage(int32_t size, const Ice::Current&) {
            std::vector<int32_t> ret (size);

            checkErr(::GetOldestImage(
                        reinterpret_cast<long *>(ret.data()),
                        size
                        )
                    );

            return ret;
        }

        std::vector<int16_t> GetOldestImage16(int32_t size, const Ice::Current&) {
            std::vector<int16_t> ret (size);

            checkErr(::GetOldestImage16(
                        reinterpret_cast<WORD *>(ret.data()),
                        size
                        )
                    );

            return ret;
        }

        AndorCapabilities GetCapabilities(const Ice::Current&) {
            AndorCapabilities caps;
            checkErr(::GetCapabilities(reinterpret_cast<::AndorCapabilities *>(&caps)));

            return caps;
        }

        Ret_GetMostRecentColorImage16 GetMostRecentColorImage16(int32_t size, int32_t algorithm, const Ice::Current&) {
            Ret_GetMostRecentColorImage16 ret;
            ret.red.resize(size);
            ret.green.resize(size);
            ret.blue.resize(size);
            checkErr(::GetMostRecentColorImage16(
                        size,
                        algorithm,
                        reinterpret_cast<uint16_t *>(ret.red.data()),
                        reinterpret_cast<uint16_t *>(ret.green.data()),
                        reinterpret_cast<uint16_t *>(ret.blue.data())
                        )
                    );

            return ret;
        }

        std::vector<BYTE> I2CBurstRead(BYTE i2cAddress, int32_t nBytes, const Ice::Current&) {
            std::vector<BYTE> ret (nBytes);
            checkErr(::I2CBurstRead(
                        i2cAddress,
                        nBytes,
                        ret.data()
                        )
                    );

            return ret;
        }

        void I2CBurstWrite(BYTE i2cAddress, int32_t nBytes, std::vector<Ice::Byte> data, const Ice::Current&) {
            checkErr(::I2CBurstWrite(
                        i2cAddress,
                        nBytes,
                        const_cast<BYTE *>(data.data())
                        )
                    );
        }

        void SetRandomTracks(int32_t numTracks, std::vector<int32_t> areas, const Ice::Current&) {
            checkErr(::SetRandomTracks(
                        numTracks,
                        const_cast<int32_t *>(areas.data())
                        )
                    );
        }

        Ret_DemosaicImage DemosaicImage(std::vector<int16_t> grey, ColorDemosaicInfo info, const Ice::Current&) {
            Ret_DemosaicImage ret;
            ret.red.resize(info.iX * info.iY);
            ret.green.resize(info.iX * info.iY);
            ret.blue.resize(info.iX * info.iY); 

            checkErr(::DemosaicImage(
                        const_cast<uint16_t *>(reinterpret_cast<const uint16_t *>(grey.data())),
                        reinterpret_cast<uint16_t *>(ret.red.data()),
                        reinterpret_cast<uint16_t *>(ret.green.data()),
                        reinterpret_cast<uint16_t *>(ret.blue.data()),
                        const_cast<::ColorDemosaicInfo *>(reinterpret_cast<const ::ColorDemosaicInfo *>(&info))
                        )
                    );

            return ret;
        }

        std::string GetAmpDesc(int32_t index, const Ice::Current&) {
            char buf[1024];
            checkErr(::GetAmpDesc(index, buf, 1024));

            return buf;
        };

        std::string GetControllerCardModel(const Ice::Current&) {
            char buf[1024];
            checkErr(::GetControllerCardModel(buf));

            return buf;
        }

        std::string GetHeadModel(const Ice::Current&) {
            char buf[1024];
            checkErr(::GetHeadModel(buf));

            return buf;
        }

        Ret_GetMetaDataInfo GetMetaDataInfo(int32_t index, const Ice::Current&){
            Ret_GetMetaDataInfo ret;

            checkErr(::GetMetaDataInfo(
                        reinterpret_cast<SYSTEMTIME *>(&ret.timeOfStart),
                        &ret.pfTimeFromStart,
                        index
                        )
                    );

            return ret;
        }

        std::string GetPreAmpGainText(int32_t index, const Ice::Current&) {
            char buf[1024];
            checkErr(::GetPreAmpGainText(index, buf, 1024));

            return buf;
        }

        std::string GetVersionInfo(AT_VersionInfoId arr, const Ice::Current&) {
            char buf[1024];
            checkErr(::GetVersionInfo(static_cast<::AT_VersionInfoId>(arr), buf, 1024));

            return buf;
        }

        std::string GetVSAmplitudeString(int32_t index, const Ice::Current&) {
            char buf[1024];
            checkErr(::GetVSAmplitudeString(index, buf));

            return buf;
        }

        int32_t GetVSAmplitudeFromString(std::string text, const Ice::Current&) {
            int32_t index;
            checkErr(::GetVSAmplitudeFromString(&text[0], &index));

            return index;
        }

        std::string GPIBReceive(int32_t id, int16_t address, int size, const Ice::Current&) {
            std::string ret;
            ret.resize(size);
            checkErr(::GPIBReceive(id, address, &ret[0], size));

            return ret;
        }

        Ret_GetTemperature GetTemperature(const Ice::Current&) {
            int32_t temperature;
            AndorErrNo status = static_cast<AndorErrNo>(::GetTemperature(&temperature));
            Ret_GetTemperature ret;

            switch(status) {
                case AndorErrNo::TEMPERATURE_OFF:
                case AndorErrNo::TEMPERATURE_NOT_STABILIZED:
                case AndorErrNo::TEMPERATURE_STABILIZED:
                case AndorErrNo::TEMPERATURE_NOT_REACHED:
                case AndorErrNo::TEMPERATURE_DRIFT:
                    ret.status = status;
                    break;
                default:
                    checkErr(status);
            }

            ret.temperature = temperature;

            return ret;
        }

        Ret_GetTemperatureF GetTemperatureF(const Ice::Current&) {
            float temperature;
            AndorErrNo status = static_cast<AndorErrNo>(::GetTemperatureF(&temperature));
            Ret_GetTemperatureF ret;

            switch(status) {
                case AndorErrNo::TEMPERATURE_OFF:
                case AndorErrNo::TEMPERATURE_NOT_STABILIZED:
                case AndorErrNo::TEMPERATURE_STABILIZED:
                case AndorErrNo::TEMPERATURE_NOT_REACHED:
                case AndorErrNo::TEMPERATURE_DRIFT:
                    ret.status = status;
                    break;
                default:
                    checkErr(status);
            }

            ret.temperature = temperature;

            return ret;
        }


        //
        // ----- Mostly generated code, but heavily edited to please the complier -----
        //

        void AbortAcquisition(const Ice::Current&) {
            checkErr(::AbortAcquisition());
        }

        void CancelWait(const Ice::Current&) {
            checkErr(::CancelWait());
        }

        void CoolerOFF(const Ice::Current&) {
            checkErr(::CoolerOFF());
        }

        void CoolerON(const Ice::Current&) {
            checkErr(::CoolerON());
        }

        void EnableKeepCleans(int32_t mode, const Ice::Current&) {
            checkErr(::EnableKeepCleans(mode));
        }

        void EnableSensorCompensation(int32_t mode, const Ice::Current&) {
            checkErr(::EnableSensorCompensation(mode));
        }

        int32_t Filter_GetAveragingFactor(const Ice::Current&) {
            int32_t averagingFactor;
            checkErr(::Filter_GetAveragingFactor(&averagingFactor));

            return averagingFactor;
        }

        int32_t Filter_GetAveragingFrameCount(const Ice::Current&) {
            int32_t frames;
            checkErr(::Filter_GetAveragingFrameCount(&frames));

            return frames;
        }

        int32_t Filter_GetDataAveragingMode(const Ice::Current&) {
            int32_t mode;
            checkErr(::Filter_GetDataAveragingMode(&mode));

            return mode;
        }

        // fixed return type mismatch
        int32_t Filter_GetMode(const Ice::Current&) {
            uint32_t mode;
            checkErr(::Filter_GetMode(&mode));

            return mode;
        }

        float Filter_GetThreshold(const Ice::Current&) {
            float threshold;
            checkErr(::Filter_GetThreshold(&threshold));

            return threshold;
        }

        void Filter_SetAveragingFactor(int32_t averagingFactor, const Ice::Current&) {
            checkErr(::Filter_SetAveragingFactor(averagingFactor));
        }

        void Filter_SetAveragingFrameCount(int32_t frames, const Ice::Current&) {
            checkErr(::Filter_SetAveragingFrameCount(frames));
        }

        void Filter_SetDataAveragingMode(int32_t mode, const Ice::Current&) {
            checkErr(::Filter_SetDataAveragingMode(mode));
        }

        void Filter_SetMode(int32_t mode, const Ice::Current&) {
            checkErr(::Filter_SetMode(mode));
        }

        void Filter_SetThreshold(float threshold, const Ice::Current&) {
            checkErr(::Filter_SetThreshold(threshold));
        }

        void FreeInternalMemory(const Ice::Current&) {
            checkErr(::FreeInternalMemory());
        }

        // fixed type mismatch
        Ret_GetAcquisitionProgress GetAcquisitionProgress(const Ice::Current&) {
            Ret_GetAcquisitionProgress ret;
            long acc, series;
            checkErr(::GetAcquisitionProgress(&acc, &series));

            ret.acc = acc;
            ret.series = series;

            return ret;
        }

        Ret_GetAcquisitionTimings GetAcquisitionTimings(const Ice::Current&) {
            Ret_GetAcquisitionTimings ret;
            checkErr(::GetAcquisitionTimings(&ret.exposure, &ret.accumulate, &ret.kinetic));

            return ret;
        }

        float GetAdjustedRingExposureTimes(int32_t inumTimes, const Ice::Current&) {
            float fptimes;
            checkErr(::GetAdjustedRingExposureTimes(inumTimes, &fptimes));

            return fptimes;
        }

        float GetAmpMaxSpeed(int32_t index, const Ice::Current&) {
            float speed;
            checkErr(::GetAmpMaxSpeed(index, &speed));

            return speed;
        }

        // fixed call type mismatch
        int32_t GetAvailableCameras(const Ice::Current&) {
            long totalCameras;
            checkErr(::GetAvailableCameras(&totalCameras));

            return totalCameras;
        }

        int32_t GetBaselineClamp(const Ice::Current&) {
            int32_t state;
            checkErr(::GetBaselineClamp(&state));

            return state;
        }

        int32_t GetBitDepth(int32_t channel, const Ice::Current&) {
            int32_t depth;
            checkErr(::GetBitDepth(channel, &depth));

            return depth;
        }

        int32_t GetBitsPerPixel(int32_t readoutIndex, int32_t index, const Ice::Current&) {
            int32_t value;
            checkErr(::GetBitsPerPixel(readoutIndex, index, &value));

            return value;
        }

        // fixed call type mismatch
        int32_t GetCameraEventStatus(const Ice::Current&) {
            uint32_t camStatus;
            checkErr(::GetCameraEventStatus(reinterpret_cast<DWORD*>(&camStatus)));

            return camStatus;
        }

        // fixed call type mismatch
        int32_t GetCameraHandle(int32_t cameraIndex, const Ice::Current&) {
            long cameraHandle;
            checkErr(::GetCameraHandle(cameraIndex, &cameraHandle));

            return cameraHandle;
        }

        // fixed call type mismatch
        int32_t GetCameraInformation(int32_t index, const Ice::Current&) {
            long information;
            checkErr(::GetCameraInformation(index, &information));

            return information;
        }

        int32_t GetCameraSerialNumber(const Ice::Current&) {
            int32_t number;
            checkErr(::GetCameraSerialNumber(&number));

            return number;
        }

        Ret_GetCountConvertWavelengthRange GetCountConvertWavelengthRange(const Ice::Current&) {
            Ret_GetCountConvertWavelengthRange ret;
            checkErr(::GetCountConvertWavelengthRange(&ret.minval, &ret.maxval));

            return ret;
        }

        // fixed call type mismatch
        int32_t GetCurrentCamera(const Ice::Current&) {
            long cameraHandle;
            checkErr(::GetCurrentCamera(&cameraHandle));

            return cameraHandle;
        }

        // fixed string buffer
        Ret_GetCurrentPreAmpGain GetCurrentPreAmpGain(const Ice::Current&) {
            Ret_GetCurrentPreAmpGain ret;
            char buf[1024];
            checkErr(::GetCurrentPreAmpGain(&ret.index, buf, 1024));

            ret.name = buf;

            return ret;
        }

        // fixed return type mismatch
        int32_t GetDDGExternalOutputEnabled(int32_t uiIndex, const Ice::Current&) {
            unsigned long puiEnabled;
            checkErr(::GetDDGExternalOutputEnabled(uiIndex, &puiEnabled));

            return puiEnabled;
        }

        // fixed return type mismatch
        int32_t GetDDGExternalOutputPolarity(int32_t uiIndex, const Ice::Current&) {
            unsigned long puiPolarity;
            checkErr(::GetDDGExternalOutputPolarity(uiIndex, &puiPolarity));

            return puiPolarity;
        }

        // fixed return type mismatch
        int32_t GetDDGExternalOutputStepEnabled(int32_t uiIndex, const Ice::Current&) {
            unsigned long puiEnabled;
            checkErr(::GetDDGExternalOutputStepEnabled(uiIndex, &puiEnabled));

            return puiEnabled;
        }

        // fixed call type mismatch
        Ret_GetDDGExternalOutputTime GetDDGExternalOutputTime(int32_t uiIndex, const Ice::Current&) {
            Ret_GetDDGExternalOutputTime ret;
            uint64_t puiDelay, puiWidth;

            checkErr(::GetDDGExternalOutputTime(uiIndex, &puiDelay, &puiWidth));

            ret.puiDelay = puiDelay;
            ret.puiWidth = puiWidth;

            return ret;
        }

        // fixed call type mismatch
        Ret_GetDDGGateTime GetDDGGateTime(const Ice::Current&) {
            Ret_GetDDGGateTime ret;
            uint64_t puiDelay, puiWidth;
            checkErr(::GetDDGGateTime(&puiDelay, &puiWidth));

            ret.puiDelay = puiDelay;
            ret.puiWidth = puiWidth;

            return ret;
        }

        int32_t GetDDGInsertionDelay(const Ice::Current&) {
            int32_t piState;
            checkErr(::GetDDGInsertionDelay(&piState));

            return piState;
        }

        int32_t GetDDGIntelligate(const Ice::Current&) {
            int32_t piState;
            checkErr(::GetDDGIntelligate(&piState));

            return piState;
        }

        int32_t GetDDGIOC(const Ice::Current&) {
            int32_t state;
            checkErr(::GetDDGIOC(&state));

            return state;
        }

        double GetDDGIOCFrequency(const Ice::Current&) {
            double frequency;
            checkErr(::GetDDGIOCFrequency(&frequency));

            return frequency;
        }

        // fixed return type mismatch
        int32_t GetDDGIOCNumber(const Ice::Current&) {
            unsigned long numberPulses;
            checkErr(::GetDDGIOCNumber(&numberPulses));

            return numberPulses;
        }

        // fixed return type mismatch
        int32_t GetDDGIOCNumberRequested(const Ice::Current&) {
            unsigned long pulses;
            checkErr(::GetDDGIOCNumberRequested(&pulses));

            return pulses;
        }

        int64_t GetDDGIOCPeriod(const Ice::Current&) {
            uint64_t period;
            checkErr(::GetDDGIOCPeriod(&period));

            return period;
        }

        int32_t GetDDGIOCPulses(const Ice::Current&) {
            int32_t pulses;
            checkErr(::GetDDGIOCPulses(&pulses));

            return pulses;
        }

        // fixed return type mismatch
        int32_t GetDDGOpticalWidthEnabled(const Ice::Current&) {
            unsigned long puiEnabled;
            checkErr(::GetDDGOpticalWidthEnabled(&puiEnabled));

            return puiEnabled;
        }

        Ret_GetDDGPulse GetDDGPulse(double wid, double resolution, const Ice::Current&) {
            Ret_GetDDGPulse ret;
            checkErr(::GetDDGPulse(wid, resolution, &ret.delay, &ret.width));

            return ret;
        }

        // fixed call type mismatch
        Ret_GetDDGStepCoefficients GetDDGStepCoefficients(int32_t mode, const Ice::Current&) {
            Ret_GetDDGStepCoefficients ret;
            checkErr(::GetDDGStepCoefficients(mode, &ret.p1, &ret.p2));

            return ret;
        }

        // fixed call type mismatch
        int32_t GetDDGStepMode(const Ice::Current&) {
            unsigned long mode;
            checkErr(::GetDDGStepMode(&mode));

            return mode;
        }

        // fixed arg type mismatch
        int64_t GetDDGTTLGateWidth(int64_t opticalWidth, const Ice::Current&) {
            uint64_t ttlWidth;
            checkErr(::GetDDGTTLGateWidth(opticalWidth, &ttlWidth));

            return ttlWidth;
        }

        // fixed arg type mismatch
        Ret_GetDDGWidthStepCoefficients GetDDGWidthStepCoefficients(int32_t mode, const Ice::Current&) {
            Ret_GetDDGWidthStepCoefficients ret;
            checkErr(::GetDDGWidthStepCoefficients(mode, &ret.p1, &ret.p2));

            return ret;
        }

        // fixed return type mismatch
        int32_t GetDDGWidthStepMode(const Ice::Current&) {
            unsigned long mode;
            checkErr(::GetDDGWidthStepMode(&mode));

            return mode;
        }

        Ret_GetDetector GetDetector(const Ice::Current&) {
            Ret_GetDetector ret;
            checkErr(::GetDetector(&ret.xpixels, &ret.ypixels));

            return ret;
        }

        Ret_GetDualExposureTimes GetDualExposureTimes(const Ice::Current&) {
            Ret_GetDualExposureTimes ret;
            checkErr(::GetDualExposureTimes(&ret.exposure1, &ret.exposure2));

            return ret;
        }

        int32_t GetEMAdvanced(const Ice::Current&) {
            int32_t state;
            checkErr(::GetEMAdvanced(&state));

            return state;
        }

        int32_t GetEMCCDGain(const Ice::Current&) {
            int32_t gain;
            checkErr(::GetEMCCDGain(&gain));

            return gain;
        }

        Ret_GetEMGainRange GetEMGainRange(const Ice::Current&) {
            Ret_GetEMGainRange ret;
            checkErr(::GetEMGainRange(&ret.low, &ret.high));

            return ret;
        }

        // fixed arg type mismatch
        int32_t GetESDEventStatus(const Ice::Current&) {
            DWORD camStatus;
            checkErr(::GetESDEventStatus(&camStatus));

            return camStatus;
        }

        int32_t GetExternalTriggerTermination(const Ice::Current&) {
            unsigned long puiTermination;
            checkErr(::GetExternalTriggerTermination(&puiTermination));

            return puiTermination;
        }

        Ret_GetFastestRecommendedVSSpeed GetFastestRecommendedVSSpeed(const Ice::Current&) {
            Ret_GetFastestRecommendedVSSpeed ret;
            checkErr(::GetFastestRecommendedVSSpeed(&ret.index, &ret.speed));

            return ret;
        }

        int32_t GetFilterMode(const Ice::Current&) {
            int32_t mode;
            checkErr(::GetFilterMode(&mode));

            return mode;
        }

        float GetFKExposureTime(const Ice::Current&) {
            float time;
            checkErr(::GetFKExposureTime(&time));

            return time;
        }

        int32_t GetFKVShiftSpeed(int32_t index, const Ice::Current&) {
            int32_t speed;
            checkErr(::GetFKVShiftSpeed(index, &speed));

            return speed;
        }

        float GetFKVShiftSpeedF(int32_t index, const Ice::Current&) {
            float speed;
            checkErr(::GetFKVShiftSpeedF(index, &speed));

            return speed;
        }

        int32_t GetFrontEndStatus(const Ice::Current&) {
            int32_t piFlag;
            checkErr(::GetFrontEndStatus(&piFlag));

            return piFlag;
        }

        int32_t GetGateMode(const Ice::Current&) {
            int32_t piGatemode;
            checkErr(::GetGateMode(&piGatemode));

            return piGatemode;
        }

        // fix call type mismatch
        Ret_GetHardwareVersion GetHardwareVersion(const Ice::Current&) {
            Ret_GetHardwareVersion ret;
            uint32_t pCB, decode, dummy1, dummy2, cameraFirmwareVersion, cameraFirmwareBuild;
            checkErr(::GetHardwareVersion(&pCB, &decode, &dummy1, &dummy2, &cameraFirmwareVersion, &cameraFirmwareBuild));

            ret.pCB = pCB;
            ret.decode = decode;
            ret.dummy1 = dummy1; 
            ret.dummy2 = dummy2; 
            ret.cameraFirmwareVersion = cameraFirmwareVersion; 
            ret.cameraFirmwareBuild = cameraFirmwareBuild;

            return ret;
        }

        int32_t GetHorizontalSpeed(int32_t index, const Ice::Current&) {
            int32_t speed;
            checkErr(::GetHorizontalSpeed(index, &speed));

            return speed;
        }

        float GetHSSpeed(int32_t channel, int32_t typ, int32_t index, const Ice::Current&) {
            float speed;
            checkErr(::GetHSSpeed(channel, typ, index, &speed));

            return speed;
        }

        int32_t GetHVflag(const Ice::Current&) {
            int32_t bFlag;
            checkErr(::GetHVflag(&bFlag));

            return bFlag;
        }

        Ret_GetImageFlip GetImageFlip(const Ice::Current&) {
            Ret_GetImageFlip ret;
            checkErr(::GetImageFlip(&ret.iHFlip, &ret.iVFlip));

            return ret;
        }

        int32_t GetImageRotate(const Ice::Current&) {
            int iRotate;
            checkErr(::GetImageRotate(&iRotate));

            return iRotate;
        }

        // fixed call type mismatch
        int32_t GetImagesPerDMA(const Ice::Current&) {
            unsigned long images;
            checkErr(::GetImagesPerDMA(&images));

            return images;
        }

        int32_t GetIODirection(int32_t index, const Ice::Current&) {
            int32_t iDirection;
            checkErr(::GetIODirection(index, &iDirection));

            return iDirection;
        }

        int32_t GetIOLevel(int32_t index, const Ice::Current&) {
            int32_t iLevel;
            checkErr(::GetIOLevel(index, &iLevel));

            return iLevel;
        }

        // fix call type mismatch
        int32_t GetIRIGData(int32_t index, const Ice::Current&) {
            unsigned char irigData;
            checkErr(::GetIRIGData(&irigData, index));

            return irigData;
        }

        float GetKeepCleanTime(const Ice::Current&) {
            float keepCleanTime;
            checkErr(::GetKeepCleanTime(&keepCleanTime));

            return keepCleanTime;
        }

        int32_t GetMaximumBinning(ReadMode ReadMode, int32_t HorzVert, const Ice::Current&) {
            int32_t maxBinning;
            checkErr(::GetMaximumBinning(static_cast<int32_t>(ReadMode), HorzVert, &maxBinning));

            return maxBinning;
        }

        float GetMaximumExposure(const Ice::Current&) {
            float maxExp;
            checkErr(::GetMaximumExposure(&maxExp));

            return maxExp;
        }

        int32_t GetMaximumNumberRingExposureTimes(const Ice::Current&) {
            int32_t number;
            checkErr(::GetMaximumNumberRingExposureTimes(&number));

            return number;
        }

        int32_t GetMCPGain(const Ice::Current&) {
            int32_t gain;
            checkErr(::GetMCPGain(&gain));

            return gain;
        }

        Ret_GetMCPGainRange GetMCPGainRange(const Ice::Current&) {
            Ret_GetMCPGainRange ret;
            checkErr(::GetMCPGainRange(&ret.iLow, &ret.iHigh));

            return ret;
        }

        int32_t GetMCPVoltage(const Ice::Current&) {
            int32_t iVoltage;
            checkErr(::GetMCPVoltage(&iVoltage));

            return iVoltage;
        }

        int32_t GetMinimumImageLength(const Ice::Current&) {
            int32_t minImageLength;
            checkErr(::GetMinimumImageLength(&minImageLength));

            return minImageLength;
        }

        int32_t GetNumberADChannels(const Ice::Current&) {
            int32_t channels;
            checkErr(::GetNumberADChannels(&channels));

            return channels;
        }

        int32_t GetNumberAmp(const Ice::Current&) {
            int32_t amp;
            checkErr(::GetNumberAmp(&amp));

            return amp;
        }

        Ret_GetNumberAvailableImages GetNumberAvailableImages(const Ice::Current&) {
            Ret_GetNumberAvailableImages ret;
            long first, last;
            checkErr(::GetNumberAvailableImages(&first, &last));

            ret.first = first;
            ret.last = last;

            return ret;
        }

        int32_t GetNumberDDGExternalOutputs(const Ice::Current&) {
            unsigned long puiCount;
            checkErr(::GetNumberDDGExternalOutputs(&puiCount));

            return puiCount;
        }

        int32_t GetNumberFKVShiftSpeeds(const Ice::Current&) {
            int32_t number;
            checkErr(::GetNumberFKVShiftSpeeds(&number));

            return number;
        }

        int32_t GetNumberHorizontalSpeeds(const Ice::Current&) {
            int32_t number;
            checkErr(::GetNumberHorizontalSpeeds(&number));

            return number;
        }

        int32_t GetNumberHSSpeeds(int32_t channel, int32_t typ, const Ice::Current&) {
            int32_t speeds;
            checkErr(::GetNumberHSSpeeds(channel, typ, &speeds));

            return speeds;
        }

        int32_t GetNumberIO(const Ice::Current&) {
            int32_t iNumber;
            checkErr(::GetNumberIO(&iNumber));

            return iNumber;
        }

        Ret_GetNumberNewImages GetNumberNewImages(const Ice::Current&) {
            Ret_GetNumberNewImages ret;
            long first, last;
            checkErr(::GetNumberNewImages(&first, &last));

            ret.first = first;
            ret.last = last;

            return ret;
        }

        int32_t GetNumberPhotonCountingDivisions(const Ice::Current&) {
            unsigned long noOfDivisions;
            checkErr(::GetNumberPhotonCountingDivisions(&noOfDivisions));

            return noOfDivisions;
        }

        int32_t GetNumberPreAmpGains(const Ice::Current&) {
            int32_t noGains;
            checkErr(::GetNumberPreAmpGains(&noGains));

            return noGains;
        }

        int32_t GetNumberRingExposureTimes(const Ice::Current&) {
            int32_t ipnumTimes;
            checkErr(::GetNumberRingExposureTimes(&ipnumTimes));

            return ipnumTimes;
        }

        int32_t GetNumberVerticalSpeeds(const Ice::Current&) {
            int32_t number;
            checkErr(::GetNumberVerticalSpeeds(&number));

            return number;
        }

        int32_t GetNumberVSAmplitudes(const Ice::Current&) {
            int32_t number;
            checkErr(::GetNumberVSAmplitudes(&number));

            return number;
        }

        int32_t GetNumberVSSpeeds(const Ice::Current&) {
            int32_t speeds;
            checkErr(::GetNumberVSSpeeds(&speeds));

            return speeds;
        }

        int32_t GetPhosphorStatus(const Ice::Current&) {
            int32_t flag;
            checkErr(::GetPhosphorStatus(&flag));

            return flag;
        }

        Ret_GetPixelSize GetPixelSize(const Ice::Current&) {
            Ret_GetPixelSize ret;
            checkErr(::GetPixelSize(&ret.xSize, &ret.ySize));

            return ret;
        }

        float GetPreAmpGain(int32_t index, const Ice::Current&) {
            float gain;
            checkErr(::GetPreAmpGain(index, &gain));

            return gain;
        }

        float GetQE(std::string sensor, float wavelength, int32_t mode, const Ice::Current&) {
            float qE;
            checkErr(::GetQE(&sensor[0], wavelength, mode, &qE));

            return qE;
        }

        float GetReadOutTime(const Ice::Current&) {
            float readOutTime;
            checkErr(::GetReadOutTime(&readOutTime));

            return readOutTime;
        }

        int64_t GetRelativeImageTimes(int32_t first, int32_t last, int32_t size, const Ice::Current&) {
            uint64_t arr;
            checkErr(::GetRelativeImageTimes(first, last, &arr, size));

            return arr;
        }

        Ret_GetRingExposureRange GetRingExposureRange(const Ice::Current&) {
            Ret_GetRingExposureRange ret;
            checkErr(::GetRingExposureRange(&ret.fpMin, &ret.fpMax));

            return ret;
        }

        float GetSensitivity(int32_t channel, int32_t horzShift, int32_t amplifier, int32_t pa, const Ice::Current&) {
            float sensitivity;
            checkErr(::GetSensitivity(channel, horzShift, amplifier, pa, &sensitivity));

            return sensitivity;
        }

        Ret_GetShutterMinTimes GetShutterMinTimes(const Ice::Current&) {
            Ret_GetShutterMinTimes ret;
            checkErr(::GetShutterMinTimes(&ret.minclosingtime, &ret.minopeningtime));

            return ret;
        }

        int32_t GetSizeOfCircularBuffer(const Ice::Current&) {
            long index;
            checkErr(::GetSizeOfCircularBuffer(&index));

            return index;
        }

        Ret_GetSoftwareVersion GetSoftwareVersion(const Ice::Current&) {
            Ret_GetSoftwareVersion ret;
            unsigned int eprom, coffile, vxdrev, vxdver, dllrev, dllver;
            checkErr(::GetSoftwareVersion(&eprom, &coffile, &vxdrev, &vxdver, &dllrev, &dllver));
            ret.eprom = eprom; 
            ret.coffile = coffile;
            ret.vxdrev = vxdrev;
            ret.vxdver = vxdver;
            ret.dllrev = dllrev;
            ret.dllver = dllver;

            return ret;
        }

        int32_t GetSpoolProgress(const Ice::Current&) {
            long index;
            checkErr(::GetSpoolProgress(&index));

            return index;
        }

        AndorErrNo GetStatus(const Ice::Current&) {
            int32_t status;
            checkErr(::GetStatus(&status));

            return static_cast<AndorErrNo>(status);
        }

        int32_t GetTECStatus(const Ice::Current&) {
            int32_t piFlag;
            checkErr(::GetTECStatus(&piFlag));

            return piFlag;
        }

        int32_t GetTemperaturePrecision(const Ice::Current&) {
            int32_t precision;
            checkErr(::GetTemperaturePrecision(&precision));

            return precision;
        }

        Ret_GetTemperatureRange GetTemperatureRange(const Ice::Current&) {
            Ret_GetTemperatureRange ret;
            checkErr(::GetTemperatureRange(&ret.mintemp, &ret.maxtemp));

            return ret;
        }

        int32_t GetTotalNumberImagesAcquired(const Ice::Current&) {
            long index;
            checkErr(::GetTotalNumberImagesAcquired(&index));

            return index;
        }

        Ret_GetTriggerLevelRange GetTriggerLevelRange(const Ice::Current&) {
            Ret_GetTriggerLevelRange ret;
            checkErr(::GetTriggerLevelRange(&ret.minimum, &ret.maximum));

            return ret;
        }

        Ret_GetUSBDeviceDetails GetUSBDeviceDetails(const Ice::Current&) {
            Ret_GetUSBDeviceDetails ret;
            WORD vendorID, productID, firmwareVersion, specificationNumber;
            checkErr(::GetUSBDeviceDetails(&vendorID, &productID, &firmwareVersion, &specificationNumber));
            
            ret.vendorID = vendorID;
            ret.productID = productID;
            ret.firmwareVersion = firmwareVersion;
            ret.specificationNumber = specificationNumber;


            return ret;
        }

        int32_t GetVerticalSpeed(int32_t index, const Ice::Current&) {
            int32_t speed;
            checkErr(::GetVerticalSpeed(index, &speed));

            return speed;
        }

        int32_t GetVSAmplitudeValue(int32_t index, const Ice::Current&) {
            int32_t value;
            checkErr(::GetVSAmplitudeValue(index, &value));

            return value;
        }

        float GetVSSpeed(int32_t index, const Ice::Current&) {
            float speed;
            checkErr(::GetVSSpeed(index, &speed));

            return speed;
        }

        void GPIBSend(int32_t id, int16_t address, std::string text, const Ice::Current&) {
            checkErr(::GPIBSend(id, address, &text[0]));
        }

        BYTE I2CRead(BYTE deviceID, BYTE intAddress, const Ice::Current&) {
            BYTE pdata;
            checkErr(::I2CRead(deviceID, intAddress, &pdata));

            return pdata;
        }

        void I2CReset(const Ice::Current&) {
            checkErr(::I2CReset());
        }

        void I2CWrite(BYTE deviceID, BYTE intAddress, BYTE data, const Ice::Current&) {
            checkErr(::I2CWrite(deviceID, intAddress, data));
        }

        int32_t InAuxPort(int32_t port, const Ice::Current&) {
            int32_t state;
            checkErr(::InAuxPort(port, &state));

            return state;
        }

        void Initialize(std::string dir, const Ice::Current&) {
            checkErr(::Initialize(&dir[0]));
        }

        void IsAmplifierAvailable(int32_t iamp, const Ice::Current&) {
            checkErr(::IsAmplifierAvailable(iamp));
        }

        int32_t IsCoolerOn(const Ice::Current&) {
            int32_t iCoolerStatus;
            checkErr(::IsCoolerOn(&iCoolerStatus));

            return iCoolerStatus;
        }

        void IsCountConvertModeAvailable(int32_t mode, const Ice::Current&) {
            checkErr(::IsCountConvertModeAvailable(mode));
        }

        int32_t IsInternalMechanicalShutter(const Ice::Current&) {
            int32_t internalShutter;
            checkErr(::IsInternalMechanicalShutter(&internalShutter));

            return internalShutter;
        }

        int32_t IsPreAmpGainAvailable(int32_t channel, int32_t amplifier, int32_t index, int32_t pa, const Ice::Current&) {
            int32_t status;
            checkErr(::IsPreAmpGainAvailable(channel, amplifier, index, pa, &status));

            return status;
        }

        int32_t IsReadoutFlippedByAmplifier(int32_t amplifier, const Ice::Current&) {
            int32_t flipped;
            checkErr(::IsReadoutFlippedByAmplifier(amplifier, &flipped));

            return flipped;
        }

        void IsTriggerModeAvailable(TriggerMode iTriggerMode, const Ice::Current&) {
            checkErr(::IsTriggerModeAvailable(static_cast<int32_t>(iTriggerMode)));
        }

        void OA_DeleteMode(std::string pcModeName, int32_t uiModeNameLen, const Ice::Current&) {
            checkErr(::OA_DeleteMode(&pcModeName[0], uiModeNameLen));
        }

        void OA_EnableMode(std::string pcModeName, const Ice::Current&) {
            checkErr(::OA_EnableMode(&pcModeName[0]));
        }

        float OA_GetFloat(std::string pcModeName, std::string pcModeParam, const Ice::Current&) {
            float fFloatValue;
            checkErr(::OA_GetFloat(&pcModeName[0], &pcModeParam[0], &fFloatValue));

            return fFloatValue;
        }

        int32_t OA_GetInt(std::string pcModeName, std::string pcModeParam, const Ice::Current&) {
            int32_t iintValue;
            checkErr(::OA_GetInt(&pcModeName[0], &pcModeParam[0], &iintValue));

            return iintValue;
        }

        int32_t OA_GetNumberOfAcqParams(std::string pcModeName, const Ice::Current&) {
            unsigned int puiNumberOfParams;
            checkErr(::OA_GetNumberOfAcqParams(&pcModeName[0], &puiNumberOfParams));

            return puiNumberOfParams;
        }

        int32_t OA_GetNumberOfPreSetModes(const Ice::Current&) {
            unsigned int puiNumberOfModes;
            checkErr(::OA_GetNumberOfPreSetModes(&puiNumberOfModes));

            return puiNumberOfModes;
        }

        int32_t OA_GetNumberOfUserModes(const Ice::Current&) {
            unsigned int puiNumberOfModes;
            checkErr(::OA_GetNumberOfUserModes(&puiNumberOfModes));

            return puiNumberOfModes;
        }

        void OA_Initialize(std::string pcFilename, int32_t uiFileNameLen, const Ice::Current&) {
            checkErr(::OA_Initialize(&pcFilename[0], uiFileNameLen));
        }

        void OA_SetFloat(std::string pcModeName, std::string pcModeParam, float fFloatValue, const Ice::Current&) {
            checkErr(::OA_SetFloat(&pcModeName[0], &pcModeParam[0], fFloatValue));
        }

        void OA_SetInt(std::string pcModeName, std::string pcModeParam, int32_t iintValue, const Ice::Current&) {
            checkErr(::OA_SetInt(&pcModeName[0], &pcModeParam[0], iintValue));
        }

        void OA_SetString(std::string pcModeName, std::string pcModeParam, std::string pcStringValue, int32_t uiStringLen, const Ice::Current&) {
            checkErr(::OA_SetString(&pcModeName[0], &pcModeParam[0], &pcStringValue[0], uiStringLen));
        }

        void OA_WriteToFile(std::string pcFileName, int32_t uiFileNameLen, const Ice::Current&) {
            checkErr(::OA_WriteToFile(&pcFileName[0], uiFileNameLen));
        }

        void OutAuxPort(int32_t port, int32_t state, const Ice::Current&) {
            checkErr(::OutAuxPort(port, state));
        }

        void PrepareAcquisition(const Ice::Current&) {
            checkErr(::PrepareAcquisition());
        }

        void SaveAsBmp(std::string path, std::string palette, int32_t ymin, int32_t ymax, const Ice::Current&) {
            checkErr(::SaveAsBmp(&path[0], &palette[0], ymin, ymax));
        }

        void SaveAsCommentedSif(std::string path, std::string comment, const Ice::Current&) {
            checkErr(::SaveAsCommentedSif(&path[0], &comment[0]));
        }

        void SaveAsEDF(std::string szPath, int32_t iMode, const Ice::Current&) {
            checkErr(::SaveAsEDF(&szPath[0], iMode));
        }

        void SaveAsFITS(std::string szFileTitle, int32_t typ, const Ice::Current&) {
            checkErr(::SaveAsFITS(&szFileTitle[0], typ));
        }

        void SaveAsRaw(std::string szFileTitle, int32_t typ, const Ice::Current&) {
            checkErr(::SaveAsRaw(&szFileTitle[0], typ));
        }

        void SaveAsSif(std::string path, const Ice::Current&) {
            checkErr(::SaveAsSif(&path[0]));
        }

        void SaveAsSPC(std::string path, const Ice::Current&) {
            checkErr(::SaveAsSPC(&path[0]));
        }

        void SaveAsTiff(std::string path, std::string palette, int32_t position, int32_t typ, const Ice::Current&) {
            checkErr(::SaveAsTiff(&path[0], &palette[0], position, typ));
        }

        void SaveAsTiffEx(std::string path, std::string palette, int32_t position, int32_t typ, int32_t mode, const Ice::Current&) {
            checkErr(::SaveAsTiffEx(&path[0], &palette[0], position, typ, mode));
        }

        void SelectSensorPort(int32_t port, const Ice::Current&) {
            checkErr(::SelectSensorPort(port));
        }

        void SendSoftwareTrigger(const Ice::Current&) {
            checkErr(::SendSoftwareTrigger());
        }

        void SetAccumulationCycleTime(float time, const Ice::Current&) {
            checkErr(::SetAccumulationCycleTime(time));
        }

        void SetAcqStatusEvent(int64_t statusEvent, const Ice::Current&) {
            checkErr(::SetAcqStatusEvent(reinterpret_cast<HANDLE>(statusEvent)));
        }

        void SetAcquisitionMode(AcquisitionMode mode, const Ice::Current&) {
            checkErr(::SetAcquisitionMode(static_cast<int32_t>(mode)));
        }

        void SetADChannel(int32_t channel, const Ice::Current&) {
            checkErr(::SetADChannel(channel));
        }

        void SetAdvancedTriggerModeState(int32_t iState, const Ice::Current&) {
            checkErr(::SetAdvancedTriggerModeState(iState));
        }

        void SetBaselineClamp(int32_t state, const Ice::Current&) {
            checkErr(::SetBaselineClamp(state));
        }

        void SetBaselineOffset(int32_t offset, const Ice::Current&) {
            checkErr(::SetBaselineOffset(offset));
        }

        void SetBitsPerPixel(int32_t value, const Ice::Current&) {
            checkErr(::SetBitsPerPixel(value));
        }

        void SetCameraLinkMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetCameraLinkMode(mode));
        }

        void SetCameraStatusEnable(int32_t Enable, const Ice::Current&) {
            checkErr(::SetCameraStatusEnable(Enable));
        }

        void SetChargeShifting(int32_t NumberRows, int32_t NumberRepeats, const Ice::Current&) {
            checkErr(::SetChargeShifting(NumberRows, NumberRepeats));
        }

        int32_t SetComplexImage(int32_t numAreas, const Ice::Current&) {
            int32_t areas;
            checkErr(::SetComplexImage(numAreas, &areas));

            return areas;
        }

        void SetCoolerMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetCoolerMode(mode));
        }

        void SetCountConvertMode(int32_t Mode, const Ice::Current&) {
            checkErr(::SetCountConvertMode(Mode));
        }

        void SetCountConvertWavelength(float wavelength, const Ice::Current&) {
            checkErr(::SetCountConvertWavelength(wavelength));
        }

        void SetCropMode(int32_t active, int32_t cropHeight, int32_t reserved, const Ice::Current&) {
            checkErr(::SetCropMode(active, cropHeight, reserved));
        }

        void SetCurrentCamera(int32_t cameraHandle, const Ice::Current&) {
            checkErr(::SetCurrentCamera(cameraHandle));
        }

        void SetCustomTrackHBin(int32_t bin, const Ice::Current&) {
            checkErr(::SetCustomTrackHBin(bin));
        }

        void SetDACOutput(int32_t iOption, int32_t iResolution, int32_t iValue, const Ice::Current&) {
            checkErr(::SetDACOutput(iOption, iResolution, iValue));
        }

        void SetDACOutputScale(int32_t iScale, const Ice::Current&) {
            checkErr(::SetDACOutputScale(iScale));
        }

        void SetDDGExternalOutputEnabled(int32_t uiIndex, int32_t uiEnabled, const Ice::Current&) {
            checkErr(::SetDDGExternalOutputEnabled(uiIndex, uiEnabled));
        }

        void SetDDGExternalOutputPolarity(int32_t uiIndex, int32_t uiPolarity, const Ice::Current&) {
            checkErr(::SetDDGExternalOutputPolarity(uiIndex, uiPolarity));
        }

        void SetDDGExternalOutputStepEnabled(int32_t uiIndex, int32_t uiEnabled, const Ice::Current&) {
            checkErr(::SetDDGExternalOutputStepEnabled(uiIndex, uiEnabled));
        }

        void SetDDGExternalOutputTime(int32_t uiIndex, int64_t uiDelay, int64_t uiWidth, const Ice::Current&) {
            checkErr(::SetDDGExternalOutputTime(uiIndex, uiDelay, uiWidth));
        }

        void SetDDGGain(int32_t gain, const Ice::Current&) {
            checkErr(::SetDDGGain(gain));
        }

        void SetDDGGateStep(double step, const Ice::Current&) {
            checkErr(::SetDDGGateStep(step));
        }

        void SetDDGGateTime(int64_t uiDelay, int64_t uiWidth, const Ice::Current&) {
            checkErr(::SetDDGGateTime(uiDelay, uiWidth));
        }

        void SetDDGInsertionDelay(int32_t state, const Ice::Current&) {
            checkErr(::SetDDGInsertionDelay(state));
        }

        void SetDDGIntelligate(int32_t state, const Ice::Current&) {
            checkErr(::SetDDGIntelligate(state));
        }

        void SetDDGIOC(int32_t state, const Ice::Current&) {
            checkErr(::SetDDGIOC(state));
        }

        void SetDDGIOCFrequency(double frequency, const Ice::Current&) {
            checkErr(::SetDDGIOCFrequency(frequency));
        }

        void SetDDGIOCNumber(int32_t numberPulses, const Ice::Current&) {
            checkErr(::SetDDGIOCNumber(numberPulses));
        }

        void SetDDGIOCPeriod(int64_t period, const Ice::Current&) {
            checkErr(::SetDDGIOCPeriod(period));
        }

        void SetDDGIOCTrigger(int32_t trigger, const Ice::Current&) {
            checkErr(::SetDDGIOCTrigger(trigger));
        }

        void SetDDGOpticalWidthEnabled(int32_t uiEnabled, const Ice::Current&) {
            checkErr(::SetDDGOpticalWidthEnabled(uiEnabled));
        }

        void SetDDGStepCoefficients(int32_t mode, double p1, double p2, const Ice::Current&) {
            checkErr(::SetDDGStepCoefficients(mode, p1, p2));
        }

        void SetDDGStepMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetDDGStepMode(mode));
        }

        void SetDDGTimes(double t0, double t1, double t2, const Ice::Current&) {
            checkErr(::SetDDGTimes(t0, t1, t2));
        }

        void SetDDGTriggerMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetDDGTriggerMode(mode));
        }

        void SetDDGVariableGateStep(int32_t mode, double p1, double p2, const Ice::Current&) {
            checkErr(::SetDDGVariableGateStep(mode, p1, p2));
        }

        void SetDDGWidthStepCoefficients(int32_t mode, double p1, double p2, const Ice::Current&) {
            checkErr(::SetDDGWidthStepCoefficients(mode, p1, p2));
        }

        void SetDDGWidthStepMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetDDGWidthStepMode(mode));
        }

        void SetDelayGenerator(int32_t board, int16_t address, int32_t typ, const Ice::Current&) {
            checkErr(::SetDelayGenerator(board, address, typ));
        }

        void SetDMAParameters(int32_t MaxImagesPerDMA, float SecondsPerDMA, const Ice::Current&) {
            checkErr(::SetDMAParameters(MaxImagesPerDMA, SecondsPerDMA));
        }

        void SetDriverEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetDriverEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetDualExposureMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetDualExposureMode(mode));
        }

        void SetDualExposureTimes(float expTime1, float expTime2, const Ice::Current&) {
            checkErr(::SetDualExposureTimes(expTime1, expTime2));
        }

        void SetEMAdvanced(int32_t state, const Ice::Current&) {
            checkErr(::SetEMAdvanced(state));
        }

        void SetEMCCDGain(int32_t gain, const Ice::Current&) {
            checkErr(::SetEMCCDGain(gain));
        }

        void SetEMGainMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetEMGainMode(mode));
        }

        void SetESDEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetESDEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetExposureTime(float time, const Ice::Current&) {
            checkErr(::SetExposureTime(time));
        }

        void SetExternalTriggerTermination(int32_t uiTermination, const Ice::Current&) {
            checkErr(::SetExternalTriggerTermination(uiTermination));
        }

        void SetFanMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetFanMode(mode));
        }

        void SetFastExtTrigger(int32_t mode, const Ice::Current&) {
            checkErr(::SetFastExtTrigger(mode));
        }

        void SetFastKinetics(int32_t exposedRows, int32_t seriesLength, float time, int32_t mode, int32_t hbin, int32_t vbin, const Ice::Current&) {
            checkErr(::SetFastKinetics(exposedRows, seriesLength, time, mode, hbin, vbin));
        }

        void SetFastKineticsEx(int32_t exposedRows, int32_t seriesLength, float time, int32_t mode, int32_t hbin, int32_t vbin, int32_t offset, const Ice::Current&) {
            checkErr(::SetFastKineticsEx(exposedRows, seriesLength, time, mode, hbin, vbin, offset));
        }

        void SetFastKineticsStorageMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetFastKineticsStorageMode(mode));
        }

        void SetFastKineticsTimeScanMode(int32_t exposedRows, int32_t seriesLength, int32_t mode, const Ice::Current&) {
            checkErr(::SetFastKineticsTimeScanMode(exposedRows, seriesLength, mode));
        }

        void SetFilterMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetFilterMode(mode));
        }

        void SetFKVShiftSpeed(int32_t index, const Ice::Current&) {
            checkErr(::SetFKVShiftSpeed(index));
        }

        void SetFrameTransferMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetFrameTransferMode(mode));
        }

        void SetFrontEndEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetFrontEndEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetFullImage(int32_t hbin, int32_t vbin, const Ice::Current&) {
            checkErr(::SetFullImage(hbin, vbin));
        }

        void SetFVBHBin(int32_t bin, const Ice::Current&) {
            checkErr(::SetFVBHBin(bin));
        }

        void SetGain(int32_t gain, const Ice::Current&) {
            checkErr(::SetGain(gain));
        }

        void SetGate(float delay, float width, float stepRenamed, const Ice::Current&) {
            checkErr(::SetGate(delay, width, stepRenamed));
        }

        void SetGateMode(int32_t gatemode, const Ice::Current&) {
            checkErr(::SetGateMode(gatemode));
        }

        void SetHighCapacity(int32_t state, const Ice::Current&) {
            checkErr(::SetHighCapacity(state));
        }

        void SetHorizontalSpeed(int32_t index, const Ice::Current&) {
            checkErr(::SetHorizontalSpeed(index));
        }

        void SetHSSpeed(int32_t typ, int32_t index, const Ice::Current&) {
            checkErr(::SetHSSpeed(typ, index));
        }

        void SetImage(int32_t hbin, int32_t vbin, int32_t hstart, int32_t hend, int32_t vstart, int32_t vend, const Ice::Current&) {
            checkErr(::SetImage(hbin, vbin, hstart, hend, vstart, vend));
        }

        void SetImageFlip(int32_t iHFlip, int32_t iVFlip, const Ice::Current&) {
            checkErr(::SetImageFlip(iHFlip, iVFlip));
        }

        void SetImageRotate(int32_t iRotate, const Ice::Current&) {
            checkErr(::SetImageRotate(iRotate));
        }

        void SetIODirection(int32_t index, int32_t iDirection, const Ice::Current&) {
            checkErr(::SetIODirection(index, iDirection));
        }

        void SetIOLevel(int32_t index, int32_t iLevel, const Ice::Current&) {
            checkErr(::SetIOLevel(index, iLevel));
        }

        void SetIRIGModulation(int16_t mode, const Ice::Current&) {
            checkErr(::SetIRIGModulation(mode));
        }

        void SetIsolatedCropMode(int32_t active, int32_t cropheight, int32_t cropwidth, int32_t vbin, int32_t hbin, const Ice::Current&) {
            checkErr(::SetIsolatedCropMode(active, cropheight, cropwidth, vbin, hbin));
        }

        void SetIsolatedCropModeEx(int32_t active, int32_t cropheight, int32_t cropwidth, int32_t vbin, int32_t hbin, int32_t cropleft, int32_t cropbottom, const Ice::Current&) {
            checkErr(::SetIsolatedCropModeEx(active, cropheight, cropwidth, vbin, hbin, cropleft, cropbottom));
        }

        void SetIsolatedCropModeType(int32_t mode, const Ice::Current&) {
            checkErr(::SetIsolatedCropModeType(mode));
        }

        void SetKineticCycleTime(float time, const Ice::Current&) {
            checkErr(::SetKineticCycleTime(time));
        }

        void SetMCPGain(int32_t gain, const Ice::Current&) {
            checkErr(::SetMCPGain(gain));
        }

        void SetMCPGating(int32_t gating, const Ice::Current&) {
            checkErr(::SetMCPGating(gating));
        }

        void SetMessageWindow(int64_t wnd, const Ice::Current&) {
            checkErr(::SetMessageWindow(reinterpret_cast<HWND>(wnd)));
        }

        void SetMetaData(int32_t state, const Ice::Current&) {
            checkErr(::SetMetaData(state));
        }

        Ret_SetMultiTrack SetMultiTrack(int32_t number, int32_t height, int32_t offset, const Ice::Current&) {
            Ret_SetMultiTrack ret;
            checkErr(::SetMultiTrack(number, height, offset, &ret.bottom, &ret.gap));

            return ret;
        }

        void SetMultiTrackHBin(int32_t bin, const Ice::Current&) {
            checkErr(::SetMultiTrackHBin(bin));
        }

        void SetMultiTrackHRange(int32_t iStart, int32_t iEnd, const Ice::Current&) {
            checkErr(::SetMultiTrackHRange(iStart, iEnd));
        }

        void SetNumberAccumulations(int32_t number, const Ice::Current&) {
            checkErr(::SetNumberAccumulations(number));
        }

        void SetNumberKinetics(int32_t number, const Ice::Current&) {
            checkErr(::SetNumberKinetics(number));
        }

        void SetNumberPrescans(int32_t iNumber, const Ice::Current&) {
            checkErr(::SetNumberPrescans(iNumber));
        }

        void SetOutputAmplifier(int32_t typ, const Ice::Current&) {
            checkErr(::SetOutputAmplifier(typ));
        }

        void SetOverlapMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetOverlapMode(mode));
        }

        void SetOverTempEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetOverTempEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetPCIMode(int32_t mode, int32_t value, const Ice::Current&) {
            checkErr(::SetPCIMode(mode, value));
        }

        void SetPhosphorEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetPhosphorEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetPhotonCounting(int32_t state, const Ice::Current&) {
            checkErr(::SetPhotonCounting(state));
        }

        int32_t SetPhotonCountingDivisions(int32_t noOfDivisions, const Ice::Current&) {
            long divisions;
            checkErr(::SetPhotonCountingDivisions(noOfDivisions, &divisions));

            return divisions;
        }

        void SetPhotonCountingThreshold(int32_t min, int32_t max, const Ice::Current&) {
            checkErr(::SetPhotonCountingThreshold(min, max));
        }

        void SetPreAmpGain(int32_t index, const Ice::Current&) {
            checkErr(::SetPreAmpGain(index));
        }

        void SetReadMode(ReadMode mode, const Ice::Current&) {
            checkErr(::SetReadMode(static_cast<int32_t>(mode)));
        }

        void SetReadoutRegisterPacking(int32_t mode, const Ice::Current&) {
            checkErr(::SetReadoutRegisterPacking(mode));
        }

        float SetRingExposureTimes(int32_t numTimes, const Ice::Current&) {
            float times;
            checkErr(::SetRingExposureTimes(numTimes, &times));

            return times;
        }

        void SetSaturationEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetSaturationEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetSensorPortMode(int32_t mode, const Ice::Current&) {
            checkErr(::SetSensorPortMode(mode));
        }

        void SetShutter(int32_t typ, ShutterMode mode, int32_t closingtime, int32_t openingtime, const Ice::Current&) {
            checkErr(::SetShutter(typ, static_cast<int32_t>(mode), closingtime, openingtime));
        }

        void SetShutterEx(int32_t typ, ShutterMode mode, int32_t closingtime, int32_t openingtime, int32_t extmode, const Ice::Current&) {
            checkErr(::SetShutterEx(typ, static_cast<int32_t>(mode), closingtime, openingtime, extmode));
        }

        void SetSifComment(std::string comment, const Ice::Current&) {
            checkErr(::SetSifComment(&comment[0]));
        }

        void SetSingleTrack(int32_t centre, int32_t height, const Ice::Current&) {
            checkErr(::SetSingleTrack(centre, height));
        }

        void SetSingleTrackHBin(int32_t bin, const Ice::Current&) {
            checkErr(::SetSingleTrackHBin(bin));
        }

        void SetSpool(int32_t active, int32_t method, std::string path, int32_t framebuffersize, const Ice::Current&) {
            checkErr(::SetSpool(active, method, &path[0], framebuffersize));
        }

        void SetSpoolThreadCount(int32_t count, const Ice::Current&) {
            checkErr(::SetSpoolThreadCount(count));
        }

        void SetTECEvent(int64_t event, const Ice::Current&) {
            checkErr(::SetTECEvent(reinterpret_cast<HANDLE>(event)));
        }

        void SetTemperature(int32_t temperature, const Ice::Current&) {
            checkErr(::SetTemperature(temperature));
        }

        void SetTriggerInvert(int32_t mode, const Ice::Current&) {
            checkErr(::SetTriggerInvert(mode));
        }

        void SetTriggerLevel(float f_level, const Ice::Current&) {
            checkErr(::SetTriggerLevel(f_level));
        }

        void SetTriggerMode(TriggerMode mode, const Ice::Current&) {
            checkErr(::SetTriggerMode(static_cast<int32_t>(mode)));
        }

        void SetVerticalSpeed(int32_t index, const Ice::Current&) {
            checkErr(::SetVerticalSpeed(index));
        }

        void SetVSAmplitude(int32_t index, const Ice::Current&) {
            checkErr(::SetVSAmplitude(index));
        }

        void SetVSSpeed(int32_t index, const Ice::Current&) {
            checkErr(::SetVSSpeed(index));
        }

        void ShutDown(const Ice::Current&) {
            checkErr(::ShutDown());
        }

        void StartAcquisition(const Ice::Current&) {
            checkErr(::StartAcquisition());
        }

        void UpdateDDGTimings(const Ice::Current&) {
            checkErr(::UpdateDDGTimings());
        }

        void WaitForAcquisition(const Ice::Current&) {
            checkErr(::WaitForAcquisition());
        }

        void WaitForAcquisitionByHandle(int32_t cameraHandle, const Ice::Current&) {
            checkErr(::WaitForAcquisitionByHandle(cameraHandle));
        }

        void WaitForAcquisitionByHandleTimeOut(int32_t cameraHandle, int32_t iTimeOutMs, const Ice::Current&) {
            checkErr(::WaitForAcquisitionByHandleTimeOut(cameraHandle, iTimeOutMs));
        }

        void WaitForAcquisitionTimeOut(int32_t iTimeOutMs, const Ice::Current&) {
            checkErr(::WaitForAcquisitionTimeOut(iTimeOutMs));
        }
    };
}

