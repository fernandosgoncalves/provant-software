/*
 * socketTCPIP.h
 *
 *  Created on: 15/08/2014
 *      Author: fernando
 */

#ifndef SOCKETTCPIP_H_
#define SOCKETTCPIP_H_

#include "proVantTypes.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdio>

class socketTCPIP {
	int sd;
	int sdHost;
	int serverLen;
	unsigned int serverPort;
	char connectionAddress[32];

	struct sockaddr_in serverAddress;
public:
	socketTCPIP();
	void init(unsigned int port, char connectionAddress[32]);
	virtual ~socketTCPIP();
	int connect();
	proVant::states receiveData();
	int sendData(proVant::controlOutput controlOutput);
};

#endif /* SOCKETTCPIP_H_ */
