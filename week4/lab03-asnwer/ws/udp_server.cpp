#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
 
#define PORT 9999
 
int main(){
    int sock;
    struct sockaddr_in addr, client_addr;
    char recv_buffer[1024];
    int recv_len;
    socklen_t addr_len;
 
    if((sock = socket(AF_INET, SOCK_DGRAM, 0)) <0){
        perror("socket ");
        return 1;
    }
 
    memset(&addr, 0x00, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(PORT);
 
    if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        perror("bind ");
        return 1;
    }
 
    printf("waiting for messages\n");
 
    addr_len = sizeof(client_addr);
    if((recv_len = recvfrom(sock, recv_buffer, 1024, 0, (struct sockaddr *)&client_addr, &addr_len)) < 0){
        perror("recvfrom ");
        return 1;
    }
    
    recv_buffer[recv_len] = '\0';
 
    printf("ip : %s\n", inet_ntoa(client_addr.sin_addr));
    printf("received data ; %s\n", recv_buffer);
 
    sendto(sock, recv_buffer, strlen(recv_buffer), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
    
    close(sock);
    return 0;
}
