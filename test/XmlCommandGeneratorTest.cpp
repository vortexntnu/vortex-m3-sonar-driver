#include <gtest/gtest.h>
#include <api/m3CommandGenerator.hpp>

using namespace m3::api::command;

TEST(XmlCommandGenerator, Connect) { EXPECT_EQ(connect(), std::string("<Command>\n<Operation>Connect</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, Disconnect) { EXPECT_EQ(disconnect(), std::string("<Command>\n<Operation>Disconnect</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, GetStatus) { EXPECT_EQ(getStatus(), std::string("<Command>\n<Operation>Get_Status</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, SetMode)
{
	EXPECT_EQ(setMode(0), std::string("<Command>\n<Operation>Set_Mode</Operation>\n<Mode>0</Mode>\n</Command>\n"));
	EXPECT_EQ(setMode(1), std::string("<Command>\n<Operation>Set_Mode</Operation>\n<Mode>1</Mode>\n</Command>\n"));
	EXPECT_EQ(setMode(2), std::string("<Command>\n<Operation>Set_Mode</Operation>\n<Mode>2</Mode>\n</Command>\n"));
}

TEST(XmlCommandGenerator, SetTVG)
{
	EXPECT_EQ(setTVG(0, 0, 0, 0),
	          std::string("<Command>\n<Operation>Set_TVG</Operation>\n<TVG_A>0</TVG_A>\n<TVG_B>0</TVG_B>\n<TVG_C>0</TVG_C>\n<TVG_L>0</TVG_L>\n</Command>\n"));
	EXPECT_EQ(setTVG(1, 2, 3, 4),
	          std::string("<Command>\n<Operation>Set_TVG</Operation>\n<TVG_A>1</TVG_A>\n<TVG_B>2</TVG_B>\n<TVG_C>3</TVG_C>\n<TVG_L>4</TVG_L>\n</Command>\n"));
}

TEST(XmlCommandGenerator, StopPing) { EXPECT_EQ(stopPing(), std::string("<Command>\n<Operation>Stop_Ping</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, StartPing) { EXPECT_EQ(startPing(), std::string("<Command>\n<Operation>Start_Ping</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, StartRecord) { EXPECT_EQ(startRecord(), std::string("<Command>\n<Operation>Start_Record</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, StopRecord) { EXPECT_EQ(stopRecord(), std::string("<Command>\n<Operation>Stop_Record</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, StartExport) { EXPECT_EQ(startExport(), std::string("<Command>\n<Operation>Start_Export</Operation>\n</Command>\n")); }

TEST(XmlCommandGenerator, StopExport) { EXPECT_EQ(stopExport(), std::string("<Command>\n<Operation>Stop_Export</Operation>\n</Command>\n")); }