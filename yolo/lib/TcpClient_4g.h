#ifndef tcp_cli_obj
#include "uartwireless.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
static string tcp_client_ip = "192.168.43.29"; //ip
static int tcp_client_port = 2333;			   //port

class tcp_cli_obj
{
public:
	tcp_cli_obj(char *ip = (char *)tcp_client_ip.c_str(), int port = tcp_client_port)
	{
		bzero(&svr_addr, sizeof(struct sockaddr_in));
		svr_addr.sin_family = AF_INET;
		svr_addr.sin_port = htons(tcp_client_port);
		svr_addr.sin_addr.s_addr = inet_addr(tcp_client_ip.c_str());
	};
	~tcp_cli_obj()
	{
		close_socket_fd();
	}
	void sfd_init();
	void client_send_message();
	void client_recv_message();
	void close_socket_fd();
	string recv_str;
	string send_str;

private:
	int sfd;
	sockaddr_in svr_addr;
};
#endif
