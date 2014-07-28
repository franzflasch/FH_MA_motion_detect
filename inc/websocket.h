/*
 * websocket.h
 *
 *  Created on: Jul 28, 2014
 *      Author: franz
 */

#ifndef WEBSOCKET_H_
#define WEBSOCKET_H_

#include <pthread.h>

#define WS_END_THREAD 0x01

typedef struct webSocketState_s{
	unsigned int status;
	unsigned int writeable;
	struct libwebsocket *wsi;
}webSocketState_t;


int initWebsocketContext(webSocketState_t *pWebSocketStatus);
void writeWebsocket(char *data, int size, void *pWebSocketStatus);

#endif /* WEBSOCKET_H_ */
