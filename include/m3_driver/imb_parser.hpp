#pragma once
#include <cstdint>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <complex>
#include <cmath>
#include <algorithm>

namespace m3 {
namespace imb {

// Available data types for the IMB Beamformed Data Format
enum class DataType : uint16_t {
    FloatComplex = 0x1002,
    IntegerMagnitude = 0x2003
};

#pragma pack(push, 1) // Disable padding for the following structures

struct MUM_TVG_PARAMS {
    uint16_t factorA;   // Factor A, the spreading coefficient
    uint16_t factorB;   // Factor B, the absorption coefficient in dB/km
    float factorC;      // Factor C, the TVG curve offset in dB
    float factorL;      // Factor L, the maximum gain limit in dB
};

struct OFFSETS 
{
    float xOffset;    // Translational offset in X axis
    float yOffset;    // Translational offset in Y axis
    float zOffset;    // Translational offset in Z axis
    float xRotOffset; // Rotational offset about X axis in degrees (Pitch offset)
    float yRotOffset; // Rotational offset about Y axis in degrees (Roll offset)
    float zRotOffset; // Rotational offset about Z axis in degrees (Yaw offset)
    uint32_t dwMounting; // Mounting orientation
};

struct ROTATOR_OFFSETS 
{
    float fOffsetA;   // Rotator offset A in meters
    float fOffsetB;   // Rotator offset B in meters
    float fOffsetR;   // Rotator offset R in meters
    float fAngle;     // Rotator angle in degrees
};

struct REAL_TIME_SOUND_SPEED 
{
    float fRaw;       // Raw sound speed in m/s directly from sensor readings
    float fFiltered;  // Filtered sound speed in m/s after a median filter on the raw values
    float fApplied;   // Applied sound speed in m/s after filtering and thresholding
    float fThreshold; // User predefined threshold in m/s used to get the applied value
};

struct PROFILING_DEPTH_TRACKING 
{
    int8_t nRangeDeepLimitPercentage;       // Percentage of the maximum range for auto depth algorithm to switch to the next higher range
    int8_t nRangeShallowLimitPercentage;      // Percentage of the maximum range for auto depth algorithm to switch to the next lower range
    int8_t nMinPingRateInHz;                  // Minimum ping rate in Hz required by the system
    int8_t nReserved[5];                      // Reserved field to keep the structure aligned
};

struct GPS_QUALITY_PARAS 
{
    int16_t wQualIndicator;       // GPS quality indicator
    int16_t wNSat;                // Number of Satellites in use
    float fHorizDilution;         // Horizontal dilution of precision
    int16_t wPosFixQualityCm;     // Position fixed quality in cm
    int16_t wReserved;            // Reserved field to keep the structure aligned
};

struct PacketHeader 
{
    uint16_t synchronizationWord[4];    // Synchronization word, always 0x8000
    DataType dataType;                  // Data type
    uint16_t reservedField;             // Reserved field
    uint32_t reservedBytes[10];         // 40 reserved bytes
    uint32_t packet_body_size;          // Packet body size

    PacketHeader(const uint8_t* byte_array);
};

struct DataHeader 
{
    uint32_t dwBeamformed_Data_Version;                 
    uint32_t dwSonarID;                                 
    uint32_t dwSonarInfo[8];                            
    uint32_t dwTimeSec;                                 
    uint32_t dwTimeMillisec;                            
    float fSoundSpeed;                                  
    uint32_t nNumImageSample;                           
    float fNearRangeMeter;                              
    float fFarRangeMeter;                               
    float fSWST;                                        
    float fSWL;                                         
    uint16_t nNumBeams;                                 
    uint16_t wProcessingType;                           
    float fBeamList[1024];                              
    float fImageSampleInterval;                         
    uint16_t nImageDestination;                         
    uint8_t bTXPulseType;                               
    uint8_t bAzimuthProcessing;                         
    uint32_t dwModeID;                                  
    int32_t nNumHybridPRIs;                             
    int32_t nHybridIndex;                               
    uint16_t nPhaseSeqLength;                           
    uint16_t nPhaseSeqIndex;                            
    uint16_t nNumImages;                                
    uint16_t nSubImageIndex;                            
    uint32_t dwFrequency;                               
    uint32_t dwPulseLength;                             
    uint32_t dwPingNumber;                              
    float fRxFilterBW;                                  
    float fRxNominalResolution;                         
    float fPulseRepFreq;                                
    char strAppName[128];                               
    char strTXPulseName[64];                            
    MUM_TVG_PARAMS sTVGParameters;                      
    float fCompassHeading;                              
    float fMagneticVariation;                           
    float fPitch;                                       
    float fRoll;                                        
    float fDepth;                                       
    float fTemperature;                                 
    OFFSETS sOffsets;                                   
    double dbLatitude;                                  
    double dbLongitude;                                 
    float fTXWST;                                       
    uint8_t bHeadSensorVersion;                         
    uint8_t HeadHWStatus;                               
    uint8_t bSoundSpeedSource;                          
    uint8_t bTimeSyncMode;                              
    float fInternalSensorHeading;                       
    float fInternalSensorPitch;                         
    float fInternalSensorRoll;                          
    ROTATOR_OFFSETS sAxesRotatorOffsets[3];             
    uint16_t nStartElement;                             
    uint16_t nEndElement;                               
    char strCustomText[32];                             
    char strROVText[32];                                
    float fLocalTimeOffset;                             
    float fVesselSOG;                                   
    float fHeave;                                       
    float fPRIMinRange;                                 
    float fPRIMaxRange;                                 
    float fAltitude;                                    
    float fHeightGeoidAbvEllipsoid;                     
    REAL_TIME_SOUND_SPEED sRealTimeSoundSpeed;          
    PROFILING_DEPTH_TRACKING sProfilingDepthTracking;   
    GPS_QUALITY_PARAS sGPSQualityParas;                 
    uint32_t b3DScan;                                   
    uint32_t b3DScanAxis;                               
    uint8_t bSonarClassID;                              
    uint8_t byTX_Enabled;                               
    uint8_t bReserved[2];                               
    float fROV_Altitude;                                
    float fConductivity;                                
    uint8_t reserved[3796];                             

    DataHeader(const uint8_t* byte_array);
};

struct DataBody 
{
    DataType dataType;
    size_t size;
    // For 8-bit integer magnitude data
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> magnitudeData;
    // For complex data
    Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic> complexData;

    DataBody(const uint8_t* byte_array, uint16_t nNumBeams, uint16_t nNumSamples, DataType type);
};

struct PacketFooter 
{
    uint32_t packet_body_size;      
    uint32_t reserved_bytes[10];   

    PacketFooter(const uint8_t* byte_array);
};

#pragma pack(pop) // Restore default packing

// Combined structure for the IMB Beamformed Data Format
struct ImbPacketStructure {
    PacketHeader packet_header;
    DataHeader data_header;
    DataBody dataBody;
    PacketFooter packet_footer;
    bool validated = true;
    // Generated point cloud (ROS independent)
    pcl::PointCloud<pcl::PointXYZI> cloud;

    ImbPacketStructure(const uint8_t* byteArray);
    void Validate();
    void GeneratePointCloud();
};

} // namespace imb
} // namespace m3
