#include <gtest/gtest.h>
#include <xml/m3CommandGenerator.hpp>


TEST(XmlCommandGenerator, Connect) {
    EXPECT_EQ(xml::command::connect(), std::string("<Command>\n<Operation>Connect</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, Disconnect) {
    EXPECT_EQ(xml::command::disconnect(), std::string("<Command>\n<Operation>Disconnect</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, GetStatus) {
    EXPECT_EQ(xml::command::getStatus(), std::string("<Command>\n<Operation>Get_Status</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, SetMode) {
    EXPECT_EQ(xml::command::setMode(0), std::string("<Command>\n<Operation>Set_Mode</Operation>\n<Mode>0</Mode>\n</Command>\n"));
    EXPECT_EQ(xml::command::setMode(1), std::string("<Command>\n<Operation>Set_Mode</Operation>\n<Mode>1</Mode>\n</Command>\n"));
    EXPECT_EQ(xml::command::setMode(2), std::string("<Command>\n<Operation>Set_Mode</Operation>\n<Mode>2</Mode>\n</Command>\n"));
}

TEST(XmlCommandGenerator, SetTVG) {
    EXPECT_EQ(xml::command::setTVG(0, 0, 0, 0), std::string("<Command>\n<Operation>Set_TVG</Operation>\n<TVG_A>0</TVG_A>\n<TVG_B>0</TVG_B>\n<TVG_C>0</TVG_C>\n<TVG_L>0</TVG_L>\n</Command>\n"));
    EXPECT_EQ(xml::command::setTVG(1, 2, 3, 4), std::string("<Command>\n<Operation>Set_TVG</Operation>\n<TVG_A>1</TVG_A>\n<TVG_B>2</TVG_B>\n<TVG_C>3</TVG_C>\n<TVG_L>4</TVG_L>\n</Command>\n"));

}

TEST(XmlCommandGenerator, StopPing) {
    EXPECT_EQ(xml::command::stopPing(), std::string("<Command>\n<Operation>Stop_Ping</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, StartPing) {
    EXPECT_EQ(xml::command::startPing(), std::string("<Command>\n<Operation>Start_Ping</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, StartRecord) {
    EXPECT_EQ(xml::command::startRecord(), std::string("<Command>\n<Operation>Start_Record</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, StopRecord) {
    EXPECT_EQ(xml::command::stopRecord(), std::string("<Command>\n<Operation>Stop_Record</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, StartExport) {
    EXPECT_EQ(xml::command::startExport(), std::string("<Command>\n<Operation>Start_Export</Operation>\n</Command>\n"));
}

TEST(XmlCommandGenerator, StopExport) {
    EXPECT_EQ(xml::command::stopExport(), std::string("<Command>\n<Operation>Stop_Export</Operation>\n</Command>\n"));
}