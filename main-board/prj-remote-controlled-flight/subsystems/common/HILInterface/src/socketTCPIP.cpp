/*
 * socketTCPIP.cpp
 *
 *  Created on: 15/08/2014
 *      Author: fernando
 */

#include "socketTCPIP.h"

socketTCPIP::socketTCPIP() {
}

void socketTCPIP::init(unsigned int port, char connectionAddress[32]) {
	this->serverPort = port;
	strcpy(this->connectionAddress, connectionAddress);
}

socketTCPIP::~socketTCPIP() {
	// TODO Auto-generated destructor stub
}

int socketTCPIP::connect() {
	// Procedimento de conexão com o PC

	sd = socket(AF_INET, SOCK_STREAM, 0);
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(this->serverPort);
	serverAddress.sin_addr.s_addr = inet_addr(connectionAddress);
	serverLen = sizeof(serverAddress);

	bind(sd, (struct sockaddr *) &serverAddress, serverLen);

	listen(sd, 1); // Aceita apenas 1 conexão

	printf("Esperando pela conexao do PC...\n");

	sdHost = accept(sd, NULL, NULL);
	if (sdHost < 0) {
		printf("ClientRegister: accept() failed");
		return -1;
	}

//	connected = true;

	printf("Conexao aceita\n");

	return 1;
}

proVant::states socketTCPIP::receiveData() {
	// Aguarda o recebimento de dados do PC
	int ret;
	proVant::states pcPackage;

	ret = recv(sdHost, &pcPackage, sizeof(proVant::states), 0);
	if (ret <= 0) {
		printf("Erro em recv < %d\n", ret);
		//pcPackage.flag = -1;
		return pcPackage;
	}
	//pcPackage.flag = 1;

	return pcPackage;
}

int socketTCPIP::sendData(proVant::controlOutput controlOutput) {
	// Envia dados ao PC
	int ret;

	ret = send(sdHost, &controlOutput, sizeof(controlOutput), 0);
	if (ret == -1) {
		printf("error\n");
		return -1;
	}

	return 1;
}
