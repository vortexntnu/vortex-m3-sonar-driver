#include <gtest/gtest.h>
#include <imb/TcpServer.hpp>

TEST(tcp_run_test, temp)
{
    m3::tcp::TcpServer server ("10.0.0.211", 20001);
    server.run();
}
