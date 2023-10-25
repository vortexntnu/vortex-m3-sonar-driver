// TcpServer.h
#include <atomic>
#include <thread>
#include <vector>
#include "../src/utils/RingBuffer.hpp" // Include the RingBuffer class header
namespace m3{
namespace tcp{
class TcpServer {
public:
    TcpServer(const std::string& ipAddress, int port);
    ~TcpServer() = default;
    void run();
    void stop();

private:
    int port;
    int server_fd;
    std::atomic<bool> running;
    // m3::utils::RingBuffer<std::vector<uint8_t>>& ringBuffer;

    void setup(const std::string& ipAddress, int port);
    void handleClient(int client_socket);
};
}
}



