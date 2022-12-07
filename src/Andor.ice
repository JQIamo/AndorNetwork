module AndorNetwork {
    enum AndorErrNo {
        ERROR_CODES = 20001,
        SUCCESS = 20002,
        VXDNOTINSTALLED = 20003,
        ERROR_SCAN = 20004,
        ERROR_CHECK_SUM = 20005,
        ERROR_FILELOAD = 20006,
        UNKNOWN_FUNCTION = 20007,
        ERROR_VXD_INIT = 20008,
        ERROR_ADDRESS = 20009,
        ERROR_PAGELOCK = 20010,
        ERROR_PAGEUNLOCK = 20011,
        ERROR_BOARDTEST = 20012,
        ERROR_ACK = 20013,
        ERROR_UP_FIFO = 20014,
        ERROR_PATTERN = 20015,
        ACQUISITION_ERRORS = 20017,
        ACQ_BUFFER = 20018,
        ACQ_DOWNFIFO_FULL = 20019,
        PROC_UNKONWN_INSTRUCTION = 20020,
        ILLEGAL_OP_CODE = 20021,
        KINETIC_TIME_NOT_MET = 20022,
        ACCUM_TIME_NOT_MET = 20023,
        NO_NEW_DATA = 20024,
        PCI_DMA_FAIL = 20025,
        SPOOLERROR = 20026,
        SPOOLSETUPERROR = 20027,
        FILESIZELIMITERROR = 20028,
        ERROR_FILESAVE = 20029,
        TEMPERATURE_CODES = 20033,
        TEMPERATURE_OFF = 20034,
        TEMPERATURE_NOT_STABILIZED = 20035,
        TEMPERATURE_STABILIZED = 20036,
        TEMPERATURE_NOT_REACHED = 20037,
        TEMPERATURE_OUT_RANGE = 20038,
        TEMPERATURE_NOT_SUPPORTED = 20039,
        TEMPERATURE_DRIFT = 20040,
        GENERAL_ERRORS = 20049,
        INVALID_AUX = 20050,
        COF_NOTLOADED = 20051,
        FPGAPROG = 20052,
        FLEXERROR = 20053,
        GPIBERROR = 20054,
        EEPROMVERSIONERROR = 20055,
        DATATYPE = 20064,
        DRIVER_ERRORS = 20065,
        P1INVALID = 20066,
        P2INVALID = 20067,
        P3INVALID = 20068,
        P4INVALID = 20069,
        INIERROR = 20070,
        COFERROR = 20071,
        ACQUIRING = 20072,
        IDLE = 20073,
        TEMPCYCLE = 20074,
        NOT_INITIALIZED = 20075,
        P5INVALID = 20076,
        P6INVALID = 20077,
        INVALID_MODE = 20078,
        INVALID_FILTER = 20079,
        I2CERRORS = 20080,
        I2CDEVNOTFOUND = 20081,
        I2CTIMEOUT = 20082,
        P7INVALID = 20083,
        P8INVALID = 20084,
        P9INVALID = 20085,
        P10INVALID = 20086,
        P11INVALID = 20087,
        USBERROR = 20089,
        IOCERROR = 20090,
        VRMVERSIONERROR = 20091,
        GATESTEPERROR = 20092,
        USB_INTERRUPT_ENDPOINT_ERROR = 20093,
        RANDOM_TRACK_ERROR = 20094,
        INVALID_TRIGGER_MODE = 20095,
        LOAD_FIRMWARE_ERROR = 20096,
        DIVIDE_BY_ZERO_ERROR = 20097,
        INVALID_RINGEXPOSURES = 20098,
        BINNING_ERROR = 20099,
        INVALID_AMPLIFIER = 20100,
        INVALID_COUNTCONVERT_MODE = 20101,
        USB_INTERRUPT_ENDPOINT_TIMEOUT = 20102,
        ERROR_NOCAMERA = 20990,
        NOT_SUPPORTED = 20991,
        NOT_AVAILABLE = 20992,
        ERROR_MAP = 20115,
        ERROR_UNMAP = 20116,
        ERROR_MDL = 20117,
        ERROR_UNMDL = 20118,
        ERROR_BUFFSIZE = 20119,
        ERROR_NOHANDLE = 20121,
        GATING_NOT_AVAILABLE = 20130,
        FPGA_VOLTAGE_ERROR = 20131,
        OW_CMD_FAIL = 20150,
        OWMEMORY_BAD_ADDR = 20151,
        OWCMD_NOT_AVAILABLE = 20152,
        OW_NO_SLAVES = 20153,
        OW_NOT_INITIALIZED = 20154,
        OW_ERROR_SLAVE_NUM = 20155,
        MSTIMINGS_ERROR = 20156,
        OA_NULL_ERROR = 20173,
        OA_PARSE_DTD_ERROR = 20174,
        OA_DTD_VALIDATE_ERROR = 20175,
        OA_FILE_ACCESS_ERROR = 20176,
        OA_FILE_DOES_NOT_EXIST = 20177,
        OA_XML_INVALID_OR_NOT_FOUND_ERROR = 20178,
        OA_PRESET_FILE_NOT_LOADED = 20179,
        OA_USER_FILE_NOT_LOADED = 20180,
        OA_PRESET_AND_USER_FILE_NOT_LOADED = 20181,
        OA_INVALID_FILE = 20182,
        OA_FILE_HAS_BEEN_MODIFIED = 20183,
        OA_BUFFER_FULL = 20184,
        OA_INVALID_STRING_LENGTH = 20185,
        OA_INVALID_CHARS_IN_NAME = 20186,
        OA_INVALID_NAMING = 20187,
        OA_GET_CAMERA_ERROR = 20188,
        OA_MODE_ALREADY_EXISTS = 20189,
        OA_STRINGS_NOT_EQUAL = 20190,
        OA_NO_USER_DATA = 20191,
        OA_VALUE_NOT_SUPPORTED = 20192,
        OA_MODE_DOES_NOT_EXIST = 20193,
        OA_CAMERA_NOT_SUPPORTED = 20194,
        OA_FAILED_TO_GET_MODE = 20195,
        OA_CAMERA_NOT_AVAILABLE = 20196,
        PROCESSING_FAILED = 20211,
    };

    enum ReadMode {
        FULL_VERTICAL_BINNING = 0,
        MULTI_TRACK = 1,
        RANDOM_TRACK = 2,
        SINGLE_TRACK = 3,
        IMAGE = 4
    };


    enum TriggerMode {
        INTERNAL = 0,
        EXTERNAL = 1,
        EXTERNAL_START = 6,
        EXTERNAL_EXPOSURE_BULB = 7,
        EXTERNAL_FVB_EM = 9,
        SOFTWARE_TRIGGER = 10,
        EXTERNAL_CHARGE_SHIFTING = 12
    };


    enum AcquisitionMode {
        SINGLE_SCAN = 1,
        ACCUMULATE = 2,
        KINETICS = 3,
        FAST_KINETICS = 4,
        RUN_TILL_ABORT = 5
    };


    enum SpoolMode {
        FILE_32_BIT_SEQUENCE = 0,
        DATA_DEPENDENT_FORMAT = 1,
        FILE_16_BIT_SEQUENCE = 2,
        MULTIPLE_DIRECTORY_STRUCTURE = 3,
        SPOOL_TO_RAM = 4,
        SPOOL_TO_16_BIT_FITS = 5,
        SPOOL_TO_SIF = 6,
        SPOOL_TO_16_BIT_TIFF = 7,
        COMPRESSED_MULTIPLE_DIRECTORY_STRUCTURE = 8
    };


    enum GateMode {
        FIRE_ANDED_WITH_THE_GATE_INPUT = 0,
        GATING_CONTROLLED_FROM_FIRE_PULSE_ONLY = 1,
        GATING_CONTROLLED_FROM_SMB_GATE_INPUT_ONLY = 2,
        GATING_ON_CONTINUOUSLY = 3,
        GATING_OFF_CONTINUOUSLY = 4,
        GATE_USING_DDG = 5
    };


    enum ShutterMode {
        FULLY_AUTO = 0,
        PERMANENTLY_OPEN = 1,
        PERMANENTLY_CLOSED = 2,
        OPEN_FOR_FVB_SERIES = 4,
        OPEN_FOR_ANY_SERIES = 5
    };

    enum AT_VersionInfoId {
        AT_SDKVersion = 0x40000000, 
        AT_DeviceDriverVersion = 0x40000001
    };

    exception AndorError {
        AndorErrNo errNo;
    };

    sequence<int> ArrayInt;
    sequence<short> ArrayShort;
    sequence<byte> ArrayByte;

    struct AndorCapabilities
    {
        int ulSize;
        int ulAcqModes;
        int ulReadModes;
        int ulTriggerModes;
        int ulCameraType;
        int ulPixelMode;
        int ulSetFunctions;
        int ulGetFunctions;
        int ulFeatures;
        int ulPCICard;
        int ulEMGainCapability;
        int ulFTReadModes;
        int ulFeatures2;
    };
    
    struct ColorDemosaicInfo
    {
        int iX;
        int iY;
        int iAlgorithm;
        int iXPhase;
        int iYPhase;
        int iBackground;
    };
	
    struct WhiteBalanceInfo
    {
        int iSize;
        int iX;
        int iY;
        int iAlgorithm;
        int iROI_left;
        int iROI_right;
        int iROI_top;
        int iROI_bottom;
        int iOperation;
    };

    struct SystemTime {
        short wYear;
        short wMonth;
        short wDayOfWeek;
        short wDay;
        short wHour;
        short wMinute;
        short wSecond;
        short wMilliseconds;
    };

    // ----- Return Value Structure -----

    struct Ret_GetImages {
        ArrayInt arr;
        int validfirst;
        int validlast;
    };

    struct Ret_GetImages16 {
        ArrayShort arr;
        int validfirst;
        int validlast;
    };

    struct Ret_GetAcquisitionProgress {
        int acc;
        int series;
    };

    struct Ret_GetAcquisitionTimings {
        float exposure;
        float accumulate;
        float kinetic;
    };

    struct Ret_GetCountConvertWavelengthRange {
        float minval;
        float maxval;
    };

    struct Ret_GetCurrentPreAmpGain {
        int index;
        string name;
    };

    struct Ret_GetDDGExternalOutputTime {
        long puiDelay;
        long puiWidth;
    };

    struct Ret_GetDDGGateTime {
        long puiDelay;
        long puiWidth;
    };

    struct Ret_GetDDGPulse {
        double delay;
        double width;
    };

    struct Ret_GetDDGStepCoefficients {
        double p1;
        double p2;
    };

    struct Ret_GetDDGWidthStepCoefficients {
        double p1;
        double p2;
    };

    struct Ret_GetDetector {
        int xpixels;
        int ypixels;
    };

    struct Ret_GetDualExposureTimes {
        float exposure1;
        float exposure2;
    };

    struct Ret_GetEMGainRange {
        int low;
        int high;
    };

    struct Ret_GetFastestRecommendedVSSpeed {
        int index;
        float speed;
    };

    struct Ret_GetHardwareVersion {
        int pCB;
        int decode;
        int dummy1;
        int dummy2;
        int cameraFirmwareVersion;
        int cameraFirmwareBuild;
    };

    struct Ret_GetImageFlip {
        int iHFlip;
        int iVFlip;
    };

    struct Ret_GetMCPGainRange {
        int iLow;
        int iHigh;
    };

    struct Ret_GetMostRecentColorImage16 {
        ArrayShort red;
        ArrayShort green;
        ArrayShort blue;
    };

    struct Ret_DemosaicImage {
        ArrayShort red;
        ArrayShort green;
        ArrayShort blue;
    };

    struct Ret_GetNumberAvailableImages {
        int first;
        int last;
    };

    struct Ret_GetNumberNewImages {
        int first;
        int last;
    };

    struct Ret_GetPixelSize {
        float xSize;
        float ySize;
    };

    struct Ret_GetRingExposureRange {
        float fpMin;
        float fpMax;
    };

    struct Ret_GetShutterMinTimes {
        int minclosingtime;
        int minopeningtime;
    };

    struct Ret_GetSoftwareVersion {
        int eprom;
        int coffile;
        int vxdrev;
        int vxdver;
        int dllrev;
        int dllver;
    };

    struct Ret_GetTemperatureRange {
        int mintemp;
        int maxtemp;
    };

    struct Ret_GetTriggerLevelRange {
        float minimum;
        float maximum;
    };

    struct Ret_GetUSBDeviceDetails {
        short vendorID;
        short productID;
        short firmwareVersion;
        short specificationNumber;
    };

    struct Ret_SetMultiTrack {
        int bottom;
        int gap;
    };

    struct Ret_GetMetaDataInfo {
        SystemTime timeOfStart;
        float pfTimeFromStart;
    };

    interface Andor {
        void AbortAcquisition() throws AndorError;
        void CancelWait() throws AndorError;
        void CoolerOFF() throws AndorError;
        void CoolerON() throws AndorError;
        void EnableKeepCleans(int mode) throws AndorError;
        void EnableSensorCompensation(int mode) throws AndorError;
        int Filter_GetAveragingFactor() throws AndorError;
        int Filter_GetAveragingFrameCount() throws AndorError;
        int Filter_GetDataAveragingMode() throws AndorError;
        int Filter_GetMode() throws AndorError;
        float Filter_GetThreshold() throws AndorError;
        void Filter_SetAveragingFactor(int averagingFactor) throws AndorError;
        void Filter_SetAveragingFrameCount(int frames) throws AndorError;
        void Filter_SetDataAveragingMode(int mode) throws AndorError;
        void Filter_SetMode(int mode) throws AndorError;
        void Filter_SetThreshold(float threshold) throws AndorError;
        void FreeInternalMemory() throws AndorError;
        Ret_GetAcquisitionProgress GetAcquisitionProgress() throws AndorError;
        Ret_GetAcquisitionTimings GetAcquisitionTimings() throws AndorError;
        float GetAdjustedRingExposureTimes(int inumTimes) throws AndorError;
        float GetAmpMaxSpeed(int index) throws AndorError;
        int GetAvailableCameras() throws AndorError;
        int GetBaselineClamp() throws AndorError;
        int GetBitDepth(int channel) throws AndorError;
        int GetBitsPerPixel(int readoutIndex, int index) throws AndorError;
        int GetCameraEventStatus() throws AndorError;
        int GetCameraHandle(int cameraIndex) throws AndorError;
        int GetCameraInformation(int index) throws AndorError;
        int GetCameraSerialNumber() throws AndorError;
        Ret_GetCountConvertWavelengthRange GetCountConvertWavelengthRange() throws AndorError;
        int GetCurrentCamera() throws AndorError;
        Ret_GetCurrentPreAmpGain GetCurrentPreAmpGain() throws AndorError;
        int GetDDGExternalOutputEnabled(int uiIndex) throws AndorError;
        int GetDDGExternalOutputPolarity(int uiIndex) throws AndorError;
        int GetDDGExternalOutputStepEnabled(int uiIndex) throws AndorError;
        Ret_GetDDGExternalOutputTime GetDDGExternalOutputTime(int uiIndex) throws AndorError;
        Ret_GetDDGGateTime GetDDGGateTime() throws AndorError;
        int GetDDGInsertionDelay() throws AndorError;
        int GetDDGIntelligate() throws AndorError;
        int GetDDGIOC() throws AndorError;
        double GetDDGIOCFrequency() throws AndorError;
        int GetDDGIOCNumber() throws AndorError;
        int GetDDGIOCNumberRequested() throws AndorError;
        long GetDDGIOCPeriod() throws AndorError;
        int GetDDGIOCPulses() throws AndorError;
        int GetDDGOpticalWidthEnabled() throws AndorError;
        Ret_GetDDGPulse GetDDGPulse(double wid, double resolution) throws AndorError;
        Ret_GetDDGStepCoefficients GetDDGStepCoefficients(int mode) throws AndorError;
        int GetDDGStepMode() throws AndorError;
        long GetDDGTTLGateWidth(long opticalWidth) throws AndorError;
        Ret_GetDDGWidthStepCoefficients GetDDGWidthStepCoefficients(int mode) throws AndorError;
        int GetDDGWidthStepMode() throws AndorError;
        Ret_GetDetector GetDetector() throws AndorError;
        Ret_GetDualExposureTimes GetDualExposureTimes() throws AndorError;
        int GetEMAdvanced() throws AndorError;
        int GetEMCCDGain() throws AndorError;
        Ret_GetEMGainRange GetEMGainRange() throws AndorError;
        int GetESDEventStatus() throws AndorError;
        int GetExternalTriggerTermination() throws AndorError;
        Ret_GetFastestRecommendedVSSpeed GetFastestRecommendedVSSpeed() throws AndorError;
        int GetFilterMode() throws AndorError;
        float GetFKExposureTime() throws AndorError;
        int GetFKVShiftSpeed(int index) throws AndorError;
        float GetFKVShiftSpeedF(int index) throws AndorError;
        int GetFrontEndStatus() throws AndorError;
        int GetGateMode() throws AndorError;
        Ret_GetHardwareVersion GetHardwareVersion() throws AndorError;
        int GetHorizontalSpeed(int index) throws AndorError;
        float GetHSSpeed(int channel, int typ, int index) throws AndorError;
        int GetHVflag() throws AndorError;
        Ret_GetImageFlip GetImageFlip() throws AndorError;
        int GetImageRotate() throws AndorError;
        int GetImagesPerDMA() throws AndorError;
        int GetIODirection(int index) throws AndorError;
        int GetIOLevel(int index) throws AndorError;
        int GetIRIGData(int index) throws AndorError;
        float GetKeepCleanTime() throws AndorError;
        int GetMaximumBinning(ReadMode ReadMode, int HorzVert) throws AndorError;
        float GetMaximumExposure() throws AndorError;
        int GetMaximumNumberRingExposureTimes() throws AndorError;
        int GetMCPGain() throws AndorError;
        Ret_GetMCPGainRange GetMCPGainRange() throws AndorError;
        int GetMCPVoltage() throws AndorError;
        int GetMinimumImageLength() throws AndorError;
        int GetNumberADChannels() throws AndorError;
        int GetNumberAmp() throws AndorError;
        Ret_GetNumberAvailableImages GetNumberAvailableImages() throws AndorError;
        int GetNumberDDGExternalOutputs() throws AndorError;
        int GetNumberFKVShiftSpeeds() throws AndorError;
        int GetNumberHorizontalSpeeds() throws AndorError;
        int GetNumberHSSpeeds(int channel, int typ) throws AndorError;
        int GetNumberIO() throws AndorError;
        Ret_GetNumberNewImages GetNumberNewImages() throws AndorError;
        int GetNumberPhotonCountingDivisions() throws AndorError;
        int GetNumberPreAmpGains() throws AndorError;
        int GetNumberRingExposureTimes() throws AndorError;
        int GetNumberVerticalSpeeds() throws AndorError;
        int GetNumberVSAmplitudes() throws AndorError;
        int GetNumberVSSpeeds() throws AndorError;
        int GetPhosphorStatus() throws AndorError;
        Ret_GetPixelSize GetPixelSize() throws AndorError;
        float GetPreAmpGain(int index) throws AndorError;
        float GetQE(string sensor, float wavelength, int mode) throws AndorError;
        float GetReadOutTime() throws AndorError;
        long GetRelativeImageTimes(int first, int last, int size) throws AndorError;
        Ret_GetRingExposureRange GetRingExposureRange() throws AndorError;
        float GetSensitivity(int channel, int horzShift, int amplifier, int pa) throws AndorError;
        Ret_GetShutterMinTimes GetShutterMinTimes() throws AndorError;
        int GetSizeOfCircularBuffer() throws AndorError;
        Ret_GetSoftwareVersion GetSoftwareVersion() throws AndorError;
        int GetSpoolProgress() throws AndorError;
        int GetStatus() throws AndorError;
        int GetTECStatus() throws AndorError;
        int GetTemperature() throws AndorError;
        float GetTemperatureF() throws AndorError;
        int GetTemperaturePrecision() throws AndorError;
        Ret_GetTemperatureRange GetTemperatureRange() throws AndorError;
        int GetTotalNumberImagesAcquired() throws AndorError;
        Ret_GetTriggerLevelRange GetTriggerLevelRange() throws AndorError;
        Ret_GetUSBDeviceDetails GetUSBDeviceDetails() throws AndorError;
        int GetVerticalSpeed(int index) throws AndorError;
        int GetVSAmplitudeValue(int index) throws AndorError;
        float GetVSSpeed(int index) throws AndorError;
        void GPIBSend(int id, short address, string text) throws AndorError;
        ArrayByte I2CBurstRead(byte i2cAddress, int nBytes) throws AndorError;
        void I2CBurstWrite(byte i2cAddress, int nBytes, ArrayByte data) throws AndorError;
        byte I2CRead(byte deviceID, byte intAddress) throws AndorError;
        void I2CReset() throws AndorError;
        void I2CWrite(byte deviceID, byte intAddress, byte data) throws AndorError;
        int InAuxPort(int port) throws AndorError;
        void Initialize(string dir) throws AndorError;
        void IsAmplifierAvailable(int iamp) throws AndorError;
        int IsCoolerOn() throws AndorError;
        void IsCountConvertModeAvailable(int mode) throws AndorError;
        int IsInternalMechanicalShutter() throws AndorError;
        int IsPreAmpGainAvailable(int channel, int amplifier, int index, int pa) throws AndorError;
        int IsReadoutFlippedByAmplifier(int amplifier) throws AndorError;
        void IsTriggerModeAvailable(TriggerMode iTriggerMode) throws AndorError;
        void OA_DeleteMode(string pcModeName, int uiModeNameLen) throws AndorError;
        void OA_EnableMode(string pcModeName) throws AndorError;
        float OA_GetFloat(string pcModeName, string pcModeParam) throws AndorError;
        int OA_GetInt(string pcModeName, string pcModeParam) throws AndorError;
        int OA_GetNumberOfAcqParams(string pcModeName) throws AndorError;
        int OA_GetNumberOfPreSetModes() throws AndorError;
        int OA_GetNumberOfUserModes() throws AndorError;
        void OA_Initialize(string pcFilename, int uiFileNameLen) throws AndorError;
        void OA_SetFloat(string pcModeName, string pcModeParam, float fFloatValue) throws AndorError;
        void OA_SetInt(string pcModeName, string pcModeParam, int iintValue) throws AndorError;
        void OA_SetString(string pcModeName, string pcModeParam, string pcStringValue, int uiStringLen) throws AndorError;
        void OA_WriteToFile(string pcFileName, int uiFileNameLen) throws AndorError;
        void OutAuxPort(int port, int state) throws AndorError;
        void PrepareAcquisition() throws AndorError;
        void SaveAsBmp(string path, string palette, int ymin, int ymax) throws AndorError;
        void SaveAsCommentedSif(string path, string comment) throws AndorError;
        void SaveAsEDF(string szPath, int iMode) throws AndorError;
        void SaveAsFITS(string szFileTitle, int typ) throws AndorError;
        void SaveAsRaw(string szFileTitle, int typ) throws AndorError;
        void SaveAsSif(string path) throws AndorError;
        void SaveAsSPC(string path) throws AndorError;
        void SaveAsTiff(string path, string palette, int position, int typ) throws AndorError;
        void SaveAsTiffEx(string path, string palette, int position, int typ, int mode) throws AndorError;
        void SelectSensorPort(int port) throws AndorError;
        void SendSoftwareTrigger() throws AndorError;
        void SetAccumulationCycleTime(float time) throws AndorError;
        void SetAcqStatusEvent(long statusEvent) throws AndorError;
        void SetAcquisitionMode(AcquisitionMode mode) throws AndorError;
        void SetADChannel(int channel) throws AndorError;
        void SetAdvancedTriggerModeState(int iState) throws AndorError;
        void SetBaselineClamp(int state) throws AndorError;
        void SetBaselineOffset(int offset) throws AndorError;
        void SetBitsPerPixel(int value) throws AndorError;
        void SetCameraLinkMode(int mode) throws AndorError;
        void SetCameraStatusEnable(int Enable) throws AndorError;
        void SetChargeShifting(int NumberRows, int NumberRepeats) throws AndorError;
        int SetComplexImage(int numAreas) throws AndorError;
        void SetCoolerMode(int mode) throws AndorError;
        void SetCountConvertMode(int Mode) throws AndorError;
        void SetCountConvertWavelength(float wavelength) throws AndorError;
        void SetCropMode(int active, int cropHeight, int reserved) throws AndorError;
        void SetCurrentCamera(int cameraHandle) throws AndorError;
        void SetCustomTrackHBin(int bin) throws AndorError;
        void SetDACOutput(int iOption, int iResolution, int iValue) throws AndorError;
        void SetDACOutputScale(int iScale) throws AndorError;
        void SetDDGExternalOutputEnabled(int uiIndex, int uiEnabled) throws AndorError;
        void SetDDGExternalOutputPolarity(int uiIndex, int uiPolarity) throws AndorError;
        void SetDDGExternalOutputStepEnabled(int uiIndex, int uiEnabled) throws AndorError;
        void SetDDGExternalOutputTime(int uiIndex, long uiDelay, long uiWidth) throws AndorError;
        void SetDDGGain(int gain) throws AndorError;
        void SetDDGGateStep(double step) throws AndorError;
        void SetDDGGateTime(long uiDelay, long uiWidth) throws AndorError;
        void SetDDGInsertionDelay(int state) throws AndorError;
        void SetDDGIntelligate(int state) throws AndorError;
        void SetDDGIOC(int state) throws AndorError;
        void SetDDGIOCFrequency(double frequency) throws AndorError;
        void SetDDGIOCNumber(int numberPulses) throws AndorError;
        void SetDDGIOCPeriod(long period) throws AndorError;
        void SetDDGIOCTrigger(int trigger) throws AndorError;
        void SetDDGOpticalWidthEnabled(int uiEnabled) throws AndorError;
        void SetDDGStepCoefficients(int mode, double p1, double p2) throws AndorError;
        void SetDDGStepMode(int mode) throws AndorError;
        void SetDDGTimes(double t0, double t1, double t2) throws AndorError;
        void SetDDGTriggerMode(int mode) throws AndorError;
        void SetDDGVariableGateStep(int mode, double p1, double p2) throws AndorError;
        void SetDDGWidthStepCoefficients(int mode, double p1, double p2) throws AndorError;
        void SetDDGWidthStepMode(int mode) throws AndorError;
        void SetDelayGenerator(int board, short address, int typ) throws AndorError;
        void SetDMAParameters(int MaxImagesPerDMA, float SecondsPerDMA) throws AndorError;
        void SetDriverEvent(long event) throws AndorError;
        void SetDualExposureMode(int mode) throws AndorError;
        void SetDualExposureTimes(float expTime1, float expTime2) throws AndorError;
        void SetEMAdvanced(int state) throws AndorError;
        void SetEMCCDGain(int gain) throws AndorError;
        void SetEMGainMode(int mode) throws AndorError;
        void SetESDEvent(long event) throws AndorError;
        void SetExposureTime(float time) throws AndorError;
        void SetExternalTriggerTermination(int uiTermination) throws AndorError;
        void SetFanMode(int mode) throws AndorError;
        void SetFastExtTrigger(int mode) throws AndorError;
        void SetFastKinetics(int exposedRows, int seriesLength, float time, int mode, int hbin, int vbin) throws AndorError;
        void SetFastKineticsEx(int exposedRows, int seriesLength, float time, int mode, int hbin, int vbin, int offset) throws AndorError;
        void SetFastKineticsStorageMode(int mode) throws AndorError;
        void SetFastKineticsTimeScanMode(int exposedRows, int seriesLength, int mode) throws AndorError;
        void SetFilterMode(int mode) throws AndorError;
        void SetFKVShiftSpeed(int index) throws AndorError;
        void SetFrameTransferMode(int mode) throws AndorError;
        void SetFrontEndEvent(long event) throws AndorError;
        void SetFullImage(int hbin, int vbin) throws AndorError;
        void SetFVBHBin(int bin) throws AndorError;
        void SetGain(int gain) throws AndorError;
        void SetGate(float delay, float width, float stepRenamed) throws AndorError;
        void SetGateMode(int gatemode) throws AndorError;
        void SetHighCapacity(int state) throws AndorError;
        void SetHorizontalSpeed(int index) throws AndorError;
        void SetHSSpeed(int typ, int index) throws AndorError;
        void SetImage(int hbin, int vbin, int hstart, int hend, int vstart, int vend) throws AndorError;
        void SetImageFlip(int iHFlip, int iVFlip) throws AndorError;
        void SetImageRotate(int iRotate) throws AndorError;
        void SetIODirection(int index, int iDirection) throws AndorError;
        void SetIOLevel(int index, int iLevel) throws AndorError;
        void SetIRIGModulation(short mode) throws AndorError;
        void SetIsolatedCropMode(int active, int cropheight, int cropwidth, int vbin, int hbin) throws AndorError;
        void SetIsolatedCropModeEx(int active, int cropheight, int cropwidth, int vbin, int hbin, int cropleft, int cropbottom) throws AndorError;
        void SetIsolatedCropModeType(int mode) throws AndorError;
        void SetKineticCycleTime(float time) throws AndorError;
        void SetMCPGain(int gain) throws AndorError;
        void SetMCPGating(int gating) throws AndorError;
        void SetMessageWindow(long wnd) throws AndorError;
        void SetMetaData(int state) throws AndorError;
        Ret_SetMultiTrack SetMultiTrack(int number, int height, int offset) throws AndorError;
        void SetMultiTrackHBin(int bin) throws AndorError;
        void SetMultiTrackHRange(int iStart, int iEnd) throws AndorError;
        void SetNumberAccumulations(int number) throws AndorError;
        void SetNumberKinetics(int number) throws AndorError;
        void SetNumberPrescans(int iNumber) throws AndorError;
        void SetOutputAmplifier(int typ) throws AndorError;
        void SetOverlapMode(int mode) throws AndorError;
        void SetOverTempEvent(long event) throws AndorError;
        void SetPCIMode(int mode, int value) throws AndorError;
        void SetPhosphorEvent(long event) throws AndorError;
        void SetPhotonCounting(int state) throws AndorError;
        int SetPhotonCountingDivisions(int noOfDivisions) throws AndorError;
        void SetPhotonCountingThreshold(int min, int max) throws AndorError;
        void SetPreAmpGain(int index) throws AndorError;
        void SetRandomTracks(int numTracks, ArrayInt areas) throws AndorError;
        void SetReadMode(ReadMode mode) throws AndorError;
        void SetReadoutRegisterPacking(int mode) throws AndorError;
        float SetRingExposureTimes(int numTimes) throws AndorError;
        void SetSaturationEvent(long event) throws AndorError;
        void SetSensorPortMode(int mode) throws AndorError;
        void SetShutter(int typ, int mode, int closingtime, int openingtime) throws AndorError;
        void SetShutterEx(int typ, int mode, int closingtime, int openingtime, int extmode) throws AndorError;
        void SetSifComment(string comment) throws AndorError;
        void SetSingleTrack(int centre, int height) throws AndorError;
        void SetSingleTrackHBin(int bin) throws AndorError;
        void SetSpool(int active, int method, string path, int framebuffersize) throws AndorError;
        void SetSpoolThreadCount(int count) throws AndorError;
        void SetTECEvent(long event) throws AndorError;
        void SetTemperature(int temperature) throws AndorError;
        void SetTriggerInvert(int mode) throws AndorError;
        void SetTriggerLevel(float f_level) throws AndorError;
        void SetTriggerMode(TriggerMode mode) throws AndorError;
        void SetVerticalSpeed(int index) throws AndorError;
        void SetVSAmplitude(int index) throws AndorError;
        void SetVSSpeed(int index) throws AndorError;
        void ShutDown() throws AndorError;
        void StartAcquisition() throws AndorError;
        void UpdateDDGTimings() throws AndorError;
        void WaitForAcquisition() throws AndorError;
        void WaitForAcquisitionByHandle(int cameraHandle) throws AndorError;
        void WaitForAcquisitionByHandleTimeOut(int cameraHandle, int iTimeOutMs) throws AndorError;
        void WaitForAcquisitionTimeOut(int iTimeOutMs) throws AndorError;

        // Manually added functions
        AndorCapabilities GetCapabilities() throws AndorError;
        ArrayInt GetAcquiredData(int size) throws AndorError;
        ArrayShort GetAcquiredData16(int size) throws AndorError;
        Ret_GetImages GetImages(int first, int last, int size) throws AndorError;
        Ret_GetImages16 GetImages16(int first, int last, int size) throws AndorError;
        Ret_GetMostRecentColorImage16 GetMostRecentColorImage16(int size, int algorithm) throws AndorError;
        ArrayInt GetMostRecentImage(int size) throws AndorError;
        ArrayShort GetMostRecentImage16(int size) throws AndorError;
        ArrayInt GetOldestImage(int size) throws AndorError;
        ArrayShort GetOldestImage16(int size) throws AndorError;
        Ret_DemosaicImage DemosaicImage(ArrayShort grey, ColorDemosaicInfo info) throws AndorError;
        string GetAmpDesc(int index) throws AndorError;
        string GetControllerCardModel() throws AndorError;
        string GetHeadModel() throws AndorError;
        Ret_GetMetaDataInfo GetMetaDataInfo(int index) throws AndorError;
        string GetPreAmpGainText(int index) throws AndorError;
        string GetVersionInfo(AT_VersionInfoId arr) throws AndorError;
        string GetVSAmplitudeString(int index) throws AndorError;
        string GPIBReceive(int id, short address, int size) throws AndorError;
    };
};
