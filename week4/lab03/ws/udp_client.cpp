#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
 
#define PORT 9999
 
int main(void){
    int sock;
    struct sockaddr_in target_addr;
    char recv_buffer[1024];
    int recv_len;
    socklen_t addr_len;
    const char *msg = "hello network!";
    
    if((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
        perror("socket ");
        return 1;
    }
 
    memset(&target_addr, 0x00, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    target_addr.sin_port = htons(PORT);
 
    addr_len = sizeof(target_addr);
 
    sendto(sock, msg, strlen(msg), 0, (struct sockaddr*)&target_addr, addr_len);
 
    if((recv_len = recvfrom(sock, recv_buffer, 1024, 0, (struct sockaddr *)&target_addr, &addr_len)) < 0){
        perror("recvfrom ");
        return 1;
    }
    
    recv_buffer[recv_len] = '\0';
 
    printf("ip : %s\n", inet_ntoa(target_addr.sin_addr));
    printf("received data : %s\n", recv_buffer);
 
    close(sock);
 
    return 0;
}

