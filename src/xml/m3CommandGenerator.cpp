#include <xml/m3CommandGenerator.hpp>

namespace xml {
namespace command {

std::string connect() { return "<Command>\n<Operation>Connect</Operation>\n</Command>\n"; }

std::string disconnect() { return "<Command>\n<Operation>Disconnect</Operation>\n</Command>\n"; }

std::string getStatus() { return "<Command>\n<Operation>Get_Status</Operation>\n</Command>\n"; }

std::string setMode(int mode) { return "<Command>\n<Operation>Set_Mode</Operation>\n<Mode>" + std::to_string(mode) + "</Mode>\n</Command>\n"; }

std::string setTVG(int tvgA, int tvgB, int tvgC, int tvgL)
{
	return "<Command>\n<Operation>Set_TVG</Operation>\n<TVG_A>" + std::to_string(tvgA) + "</TVG_A>\n<TVG_B>" + std::to_string(tvgB) + "</TVG_B>\n<TVG_C>" +
	       std::to_string(tvgC) + "</TVG_C>\n<TVG_L>" + std::to_string(tvgL) + "</TVG_L>\n</Command>\n";
}

std::string stopPing() { return "<Command>\n<Operation>Stop_Ping</Operation>\n</Command>\n"; }

std::string startPing(int mode)
{
	if (mode != -1) {
		return "<Command>\n<Operation>Start_Ping</Operation>\n<Mode>" + std::to_string(mode) + "</Mode>\n</Command>\n";
	}
	else {
		return "<Command>\n<Operation>Start_Ping</Operation>\n</Command>\n";
	}
}

std::string startRecord() { return "<Command>\n<Operation>Start_Record</Operation>\n</Command>\n"; }

std::string stopRecord() { return "<Command>\n<Operation>Stop_Record</Operation>\n</Command>\n"; }

std::string startExport() { return "<Command>\n<Operation>Start_Export</Operation>\n</Command>\n"; }

std::string stopExport() { return "<Command>\n<Operation>Stop_Export</Operation>\n</Command>\n"; }

} // namespace command
} // namespace xml