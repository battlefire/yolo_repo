#include "TcpClient_4g.h"
#include "TransferJson.h"

extern string globle_str;
extern bool switch_seq;

void tcp_cli_obj::sfd_init()
{
    signal(SIGPIPE,SIG_IGN);
    sfd = socket(AF_INET,SOCK_STREAM,0);
    if(sfd == -1)
    {
        perror("socket()");
        return;
    }
    printf("create sfd successful\n");
}

void tcp_cli_obj::client_send_message()
{
    int connect_stat;
    while(switch_seq == true)
    {
        connect_stat = connect(sfd,(struct sockaddr *)&svr_addr,sizeof(struct sockaddr));
        if(errno != EISCONN && connect_stat < 0)
        {
            perror("connect()");
            close_socket_fd();
            sfd_init();
            continue;
        }
        this->send_str = globle_str;
        cout<<send_str<<endl;
        send(sfd, this->send_str.c_str(), send_str.length(), 0);
        globle_str.clear();
        close_socket_fd();
        sfd_init();
        switch_seq = false;
    }
    return;
}

void tcp_cli_obj::client_recv_message()
{
    return;
}

void tcp_cli_obj::close_socket_fd()
{
    close(sfd);
    return;
}
