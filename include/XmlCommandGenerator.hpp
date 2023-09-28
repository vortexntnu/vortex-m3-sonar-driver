#ifndef XMLCOMMANDGENERATOR_HPP
#define XMLCOMMANDGENERATOR_HPP

#include <string>

class XmlCommandGenerator {
public:
    static std::string Connect();
    static std::string Disconnect();
    static std::string GetStatus();
    static std::string SetMode(int mode);
    static std::string SetTVG(int tvgA, int tvgB, int tvgC, int tvgL);
    static std::string StopPing();
    static std::string StartPing(int mode = -1);
    static std::string StartRecord();
    static std::string StopRecord();
    static std::string StartExport();
    static std::string StopExport();
};

#endif // XMLCOMMANDGENERATOR_HPP
