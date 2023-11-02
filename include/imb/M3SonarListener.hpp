#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
namespace m3{
class M3SonarListener {
public:
    const char *addr_;
    const u_int16_t port_;
    int client_socket_;
    char buffer_[1024];
    sockaddr_in server_addr_;
    M3SonarListener(const char *addr, u_int16_t port);
    void create_socket();
    void connect_to_sonar();
    void run_listener();
    void stop_listener();
};
}
