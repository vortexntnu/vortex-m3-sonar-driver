#include <gtest/gtest.h>
#include <imb/TcpServer.hpp>

TEST(tcp_run_test, temp)
{
    m3::tcp::TcpServer server ("100.0.0.205", 20001);
    server.run();
}
