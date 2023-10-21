#pragma once
#include <cstdint>
#include <cstring>
#include <arpa/inet.h>
#include <eigen3/Eigen/Dense>

namespace m3 {
namespace imb {

// Available data types for the IMB Beamformed Data Format
enum class DataType : uint16_t {
    FloatComplex = 0x1002,
    IntegerMagnitude = 0x2003
};

#pragma pack(push, 1) // Disable padding for the following structures

// Structure for MUM_TVG_PARAMS
struct MUM_TVG_PARAMS {
    int16_t factorA;   // Factor A, the spreading coefficient
    int16_t factorB;   // Factor B, the absorption coefficient in dB/km
    float factorC;     // Factor C, the TVG curve offset in dB
    float factorL;     // Factor L, the maximum gain limit in dB};
};

// Structure for OFFSETS
struct OFFSETS 
{
    float xOffset;    // Translational offset in X axis
    float yOffset;    // Translational offset in Y axis
    float zOffset;    // Translational offset in Z axis
    float xRotOffset; // Rotational offset about X axis in degrees (Pitch offset)
    float yRotOffset; // Rotational offset about Y axis in degrees (Roll offset)
    float zRotOffset; // Rotational offset about Z axis in degrees (Yaw offset)
};

// Structure for ROTATOR_OFFSETS
struct ROTATOR_OFFSETS 
{
    float fOffsetA;   // Rotator offset A in meters
    float fOffsetB;   // Rotator offset B in meters
    float fOffsetR;   // Rotator offset R in meters
    float fAngle;     // Rotator angle in degrees
};

// Structure for REAL_TIME_SOUND_SPEED
struct REAL_TIME_SOUND_SPEED 
{
    float fRaw;       // Raw sound speed in m/s directly from sensor readings
    float fFiltered;  // Filtered sound speed in m/s after a median filter on the raw values
    float fApplied;   // Applied sound speed in m/s after filtering and thresholding
    float fThreshold; // User predefined threshold in m/s used to get the applied value
};

// Structure for PROFILING_DEPTH_TRACKING
struct PROFILING_DEPTH_TRACKING 
{
    int8_t nRangeDeepLimitPercentage;       // Percentage of the maximum range for auto depth algorithm to switch to the next higher range
    int8_t nRangeShallowLimitPercentage;    // Percentage of the maximum range for auto depth algorithm to switch to the next lower range
    int8_t nMinPingRateInHz;                // Minimum ping rate in Hz required by the system
    int8_t nReserved[5];                    // Reserved field to keep the structure aligned
};

// Structure for GPS_QUALITY_PARAS
struct GPS_QUALITY_PARAS 
{
    int16_t wQualIndicator;         // GPS quality indicator
    int16_t wNSat;                  // Number of Satellites in use
    float fHorizDilution;           // Horizontal dilution of precision
    int16_t wPosFixQualityCm;       // Position fixed quality in cm
    int16_t wReserved;              // Reserved field to keep the structure aligned
};











// Structure for Packet Header
struct PacketHeader 
{
    uint16_t synchronizationWord[4];    // Synchronization words, always 0x8000
    DataType dataType;                  // Data type
    uint16_t reservedField;             // Reserved field
    uint32_t reservedBytes[10];         // 40 reserved bytes
    uint32_t packetBodySize;            // Packet body size

    PacketHeader(const uint8_t* byteArray);
};

// Structure for Data Header
struct DataHeader 
{
    uint32_t dwBeamformed_Data_Version;                 // Version of this header
    uint32_t dwSonarID;                                 // Sonar identification
    uint32_t dwSonarInfo[8];                            // Sonar information such as serial number, power up configurations
    uint32_t dwTimeSec;                                 // Time stamp of current ping in seconds elapsed since 1970
    uint32_t dwTimeMillisec;                            // Milliseconds part of current ping
    float fSoundSpeed;                                  // Speed of sound in m/s for image processing
    uint32_t nNumImageSample;                           // Total number of image samples
    float fNearRangeMeter;                              // Minimum mode range in meters
    float fFarRangeMeter;                               // Maximum mode range in meters
    float fSWST;                                        // Sampling Window Start Time in seconds
    float fSWL;                                         // Sampling Window Length in seconds
    uint16_t nNumBeams;                                 // Total number of beams
    uint16_t wProcessingType;                           // Processing type (e.g., standard, EIQ, Profiling, etc.)
    float fBeamList[1024];                              // List of angles for all beams
    float fImageSampleInterval;                         // Image sample interval in seconds
    uint16_t nImageDestination;                         // Image designation
    uint8_t bTXPulseType;                               // Tx pulse type (e.g., CW, FM up sweep, FM down sweep)
    uint8_t bAzimuthProcessing;                         // Azimuth processing (0: off, 1: on)
    uint32_t dwModeID;                                  // Unique mode ID, application ID
    int32_t nNumHybridPRIs;                             // Number of PRIs in a hybrid mode
    int32_t nHybridIndex;                               // Hybrid mode index
    uint16_t nPhaseSeqLength;                           // Spatial phase sequence length
    uint16_t nPhaseSeqIndex;                            // Spatial phase sequence length index
    uint16_t nNumImages;                                // Number of sub-images for one PRI
    uint16_t nSubImageIndex;                            // Sub-image index
    uint32_t dwFrequency;                               // Sonar frequency in Hz
    uint32_t dwPulseLength;                             // Transmit pulse length in microseconds
    uint32_t dwPingNumber;                              // Ping counter
    float fRxFilterBW;                                  // RX filter bandwidth in Hz
    float fRxNominalResolution;                         // RX nominal resolution in meters
    float fPulseRepFreq;                                // Pulse Repetition Frequency in Hz
    char strAppName[128];                               // Application name
    char strTXPulseName[64];                            // TX pulse name
    MUM_TVG_PARAMS sTVGParameters;                      // TVG Parameters
    double dbLatitude;                                  // Latitude of current ping in decimal degrees
    double dbLongitude;                                 // Longitude of current ping in decimal degrees
    float fTXWST;                                       // TX Window Start Time in seconds
    uint8_t bHeadSensorVersion;                         // Head Sensor Version
    uint8_t HeadHWStatus;                               // Head hardware status
    uint8_t bSoundSpeedSource;                          // Source of the sound speed
    uint8_t bTimeSyncMode;                              // Timer synchronization mode
    float fInternalSensorHeading;                       // Heading from internal sensor in decimal degrees
    float fInternalSensorPitch;                         // Pitch from internal sensor in decimal degrees
    float fInternalSensorRoll;                          // Roll from internal sensor in decimal degrees
    ROTATOR_OFFSETS sAxesRotatorOffsets[3];             // Rotator offsets
    uint16_t nStartElement;                             // Start element of the sub array
    uint16_t nEndElement;                               // End element of the sub array
    char strCustomText[32];                             // Custom text field
    char strROVText[32];                                // Custom text field 2
    float fLocalTimeOffset;                             // Local time zone offset in decimal hours
    float fVesselSOG;                                   // Vessel Speed Over Ground in knots
    float fHeave;                                       // Heave in meters
    float fPRIMinRange;                                 // Minimum PRI range in meters
    float fPRIMaxRange;                                 // Maximum PRI range in meters
    float fAltitude;                                    // Altitude in decimal meters
    float fHeightGeoidAbvEllipsoid;                     // Ellipsoid height in meters
    REAL_TIME_SOUND_SPEED sRealTimeSoundSpeed;          // Real-time sound speed
    PROFILING_DEPTH_TRACKING sProfilingDepthTracking;   // Profiling depth tracking
    GPS_QUALITY_PARAS sGPSQualityParas;                 // GPS quality parameters
    uint32_t b3DScan;                                   // 3D Scan with rotator
    uint32_t b3DScanAxis;                               // 3D Scan rotation axis
    uint8_t bSonarClassID;                              // Sonar type used to collect the data
    uint8_t byTX_Enabled;                               // For Forward Looking Sonar multiple transducers
    uint8_t bReserved[2];                               // Reserved field
    float fROV_Altitude;                                // ROV Altitude
    float fConductivity;                                // Conductivity
    uint8_t Reserved[3796];                             // Reserved field

    DataHeader(const uint8_t* byteArray);
};


// Structure for Data Body
struct DataBody 
{
    DataType dataType;
    size_t size;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> magnitudeData; // For 8-bit integer magnitude data
    Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic> complexData; // For complex data

    /**
     * @brief Construct a new Data Body object from the raw bytes of the TCP packet
     * 
     * @param byteArray Raw bytes of the TCP packet
     * @param nNumBeams Number of beams
     * @param nNumSamples Number of samples in each beam
     * @param type Data type (FloatComplex or IntegerMagnitude)
     * @throw std::runtime_error if the data type is not supported
     */
    DataBody(const uint8_t* byteArray, uint16_t nNumBeams, uint16_t nNumSamples, DataType type);
};

// Structure for Packet Footer
struct PacketFooter 
{
    uint32_t packetBodySize;      // Packet body size (4 bytes)
    uint32_t reservedBytes[10];   // 40 reserved bytes (10 x 4 bytes)

    PacketFooter(const uint8_t* byteArray);
};

#pragma pack(pop) // Stop disabling padding



// Define the combined struct of the IMB Beamformed Data Format
struct ImbPacketStructure 
{
    PacketHeader packetHeader;
    DataHeader dataHeader;
    DataBody dataBody;
    PacketFooter packetFooter;

    ImbPacketStructure(const uint8_t* byteArray);
    void validate() const;
};




} // namespace imb
} // namespace m3
