#pragma once

#include <string>

namespace m3 {
namespace api {
namespace command {

/**
 * @brief Generates an XML command for connecting.
 * The Connect function establishes communication between the M3 software
 * and Sonar Head, then starts “pinging”.
 * @return A string containing the XML command for connecting.
 */
std::string connect();

/**
 * @brief Generates an XML command for disconnecting.
 * The Disconnect function stops operation of the Sonar Head.
 * @return A string containing the XML command for disconnecting.
 */
std::string disconnect();

/**
 * @brief Generates an XML command for getting status.
 * The Get Status function displays information about the
 * Sonar Head and M3 software in the API response text.
 * @return A string containing the XML command for getting status.
 */
std::string getStatus();

/**
 * @brief Generates an XML command for setting mode.
 * The Set Mode function allows you to set the optimal operating mode for
 * your application. Each mode has its own pre-defined characteristics,
 * such as differing ranges, angular resolutions, and pulse types.
 * @param mode The mode to set.
 * @return A string containing the XML command for setting mode.
 */
std::string setMode(int mode);

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
std::string setTVG(int tvgA, int tvgB, int tvgC, int tvgL);

/**
 * @brief Generates an XML command for stopping ping.
 * If the Sonar Head is running, the Stop Ping function pauses
 * operation of the head.
 * @return A string containing the XML command for stopping ping.
 */
std::string stopPing();

/**
 * @brief Generates an XML command for starting ping.
 * If the Sonar Head is paused, the Start Ping function resumes
 * operation and starts “pinging”.
 * @param mode The mode (optional).
 * @return A string containing the XML command for starting ping.
 */
std::string startPing(int mode = -1);

/**
 * @brief Generates an XML command for starting recording.
 * The Record function allows you to record echo data.
 * @return A string containing the XML command for starting recording.
 */
std::string startRecord();

/**
 * @brief Generates an XML command for stopping recording.
 * If you are recording, the Stop Record API command will stop
 * the recording activity.
 * @return A string containing the XML command for stopping recording.
 */
std::string stopRecord();

/**
 * @brief Generates an XML command for starting export.
 * The Export function allows you to send echo data to third-party
 * software through the UDP port.
 * @return A string containing the XML command for starting export.
 */
std::string startExport();

/**
 * @brief Generates an XML command for stopping export.
 * If you are exporting data, the Stop Export API command will stop
 * the export.
 * @return A string containing the XML command for stopping export.
 */
std::string stopExport();

} // namespace command
} // namespace api
} // namespace m3
