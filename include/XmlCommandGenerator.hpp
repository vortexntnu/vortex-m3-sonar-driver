#pragma once

#include <string>

/**
 * @brief The XmlCommandGenerator class provides functions to generate XML commands.
 */
class XmlCommandGenerator {
public:
    /**
     * @brief Generates an XML command for connecting. 
     * The Connect function establishes communication between the M3 software 
     * and Sonar Head, then starts “pinging”.
     * @return A string containing the XML command for connecting.
     */
    static std::string Connect();

    /**
     * @brief Generates an XML command for disconnecting.
     * The Disconnect function stops operation of the Sonar Head.
     * @return A string containing the XML command for disconnecting.
     */
    static std::string Disconnect();

    /**
     * @brief Generates an XML command for getting status.
     * The Get Status function displays information about the 
     * Sonar Head and M3 software in the API response text.
     * @return A string containing the XML command for getting status.
     */
    static std::string GetStatus();

    /**
     * @brief Generates an XML command for setting mode. 
     * The Set Mode function allows you to set the optimal operating mode for 
     * your application. Each mode has its own pre-defined characteristics, 
     * such as differing ranges, angular resolutions, and pulse types.
     * @param mode The mode to set.
     * @return A string containing the XML command for setting mode.
     */
    static std::string SetMode(int mode);

    /**
     * @brief Generates an XML command for setting TVG values.
     * When an acoustic pulse is sent through the water, it will gradually lose its energy. The
     * greater the distance between the Sonar Head and the target(s), the greater the loss of energy.
     * The TVG (Time Variable Gain) function is used to compensate the received echo data for the
     * loss of acoustic energy due to geometric spread and absorption.
     * 
     * @param tvgA A Factor
     * @param tvgB B Factor
     * @param tvgC C Factor
     * @param tvgL L Factor
     * @return A string containing the XML command for setting TVG values.
     */
    static std::string SetTVG(int tvgA, int tvgB, int tvgC, int tvgL);

    /**
     * @brief Generates an XML command for stopping ping. 
     * If the Sonar Head is running, the Stop Ping function pauses 
     * operation of the head.
     * @return A string containing the XML command for stopping ping.
     */
    static std::string StopPing();

    /**
     * @brief Generates an XML command for starting ping.
     * If the Sonar Head is paused, the Start Ping function resumes 
     * operation and starts “pinging”.
     * @param mode The mode (optional).
     * @return A string containing the XML command for starting ping.
     */
    static std::string StartPing(int mode = -1);

    /**
     * @brief Generates an XML command for starting recording.
     * The Record function allows you to record echo data.
     * @return A string containing the XML command for starting recording.
     */
    static std::string StartRecord();

    /**
     * @brief Generates an XML command for stopping recording.
     * If you are recording, the Stop Record API command will stop 
     * the recording activity.
     * @return A string containing the XML command for stopping recording.
     */
    static std::string StopRecord();

    /**
     * @brief Generates an XML command for starting export.
     * The Export function allows you to send echo data to third-party 
     * software through the UDP port.
     * @return A string containing the XML command for starting export.
     */
    static std::string StartExport();

    /**
     * @brief Generates an XML command for stopping export.
     * If you are exporting data, the Stop Export API command will stop 
     * the export.
     * @return A string containing the XML command for stopping export.
     */
    static std::string StopExport();
};

