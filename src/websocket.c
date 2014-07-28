/*
 * websocket.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: franz
 */
#include <stdio.h>
#include <string.h>
#include <libwebsockets.h>
#include <websocket.h>
#include <pthread.h>

enum demo_protocols
{
	PROTOCOL_NULL,
	PROTOCOL_HEXA_CTRL,
	PROTOCOL_LAST
};

static int nullHttpCallback(
    struct libwebsocket_context *this,
    struct libwebsocket *wsi,
    enum libwebsocket_callback_reasons reason,
    void *user,
    void *in,
    size_t len)
{
    return 0;
}


// struct for the thread data
struct analyze_data_info {
    unsigned char *data;
    struct libwebsocket *wsi;
};

static unsigned int opts;
static int was_closed;
static int deny_deflate;
static int deny_mux;

static int
hexaCtrl(struct libwebsocket_context *this,
			struct libwebsocket *wsi,
			enum libwebsocket_callback_reasons reason,
			void *user, void *in, size_t len)
{
	webSocketState_t *pWebSocketStatus = libwebsocket_context_user(this);

	switch (reason)
	{
		case LWS_CALLBACK_CLIENT_ESTABLISHED:
			fprintf(stdout, "callback_dumb_increment: LWS_CALLBACK_CLIENT_ESTABLISHED\n");
			libwebsocket_callback_on_writable(this, wsi);
			break;

		case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
			fprintf(stderr, "LWS_CALLBACK_CLIENT_CONNECTION_ERROR\n");
			was_closed = 1;
			break;

		case LWS_CALLBACK_CLIENT_WRITEABLE:
			/* get notified as soon as we can write again */
			libwebsocket_callback_on_writable(this, wsi);
			pWebSocketStatus->writeable = 1;
			break;

		case LWS_CALLBACK_CLOSED:
			fprintf(stderr, "LWS_CALLBACK_CLOSED\n");
			was_closed = 1;
			break;

		case LWS_CALLBACK_CLIENT_RECEIVE:
			//((char *)in)[len] = '\0';
			//fprintf(stdout, "rx %d '%s'\n", (int)len, (char *)in);
			break;

		/* because we are protocols[0] ... */
		case LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED:
			if ((strcmp(in, "deflate-stream") == 0) && deny_deflate) {
				fprintf(stderr, "denied deflate-stream extension\n");
				return 1;
			}
			if ((strcmp(in, "deflate-frame") == 0) && deny_deflate) {
				fprintf(stderr, "denied deflate-frame extension\n");
				return 1;
			}
			if ((strcmp(in, "x-google-mux") == 0) && deny_mux) {
				fprintf(stderr, "denied x-google-mux extension\n");
				return 1;
			}

			break;

		default:
			break;
	}

	return 0;
}

// protocol types for websockets
static struct libwebsocket_protocols protocols[] = {
    {
        "http-only",
        nullHttpCallback,
        0
    },
    {
        "hexa_ctrl",
        hexaCtrl,
        0,
        20,
    },
    {
        NULL, NULL, 0
    }
};

void *websocketThread(void *threadInfo)
{
	int n = 0;
	int port = 9000;
	int use_ssl = 0;
	struct libwebsocket_context *context;
	const char address[] = "192.168.1.1";
	int ietf_version = -1; /* latest */
	struct lws_context_creation_info info;

	webSocketState_t *pWebSocketStatus = (webSocketState_t *)threadInfo;

	memset(&info, 0, sizeof info);

	info.port = CONTEXT_PORT_NO_LISTEN;
	info.protocols = protocols;
	#ifndef LWS_NO_EXTENSIONS
	info.extensions = libwebsocket_get_internal_extensions();
	#endif
	info.gid = -1;
	info.uid = -1;
	info.user = threadInfo;

	context = libwebsocket_create_context(&info);
	if (context == NULL) {
		fprintf(stderr, "Creating libwebsocket context failed\n");
		goto exit_thread;
	}

	pWebSocketStatus->wsi = libwebsocket_client_connect(context, address, port, use_ssl, "/", address, address, protocols[1].name, ietf_version);
	if (pWebSocketStatus->wsi == NULL)
	{
		fprintf(stderr, "libwebsocket connect failed\n");
		goto bail;
	}
	fprintf(stdout, "Waiting for connect...\n");

	n = 0;
	while ((n >= 0))
	{
		n = libwebsocket_service(context, 300);
		//writeWebsocket("DSADSA");
		if (n < 0)
			continue;
	}

	bail:
		fprintf(stderr, "Exiting\n");
		libwebsocket_context_destroy(context);

	exit_thread:
    	pthread_exit((void*) threadInfo);
}

void writeWebsocket(char *data, int size, void *pWsStatus)
{
	unsigned char buf[LWS_SEND_BUFFER_PRE_PADDING + 4096 + LWS_SEND_BUFFER_POST_PADDING];
	int n = 0;
	int i = 0;

	webSocketState_t *pWebSocketStatus = (webSocketState_t *)pWsStatus;

	if(pWebSocketStatus->writeable)
	{
		pWebSocketStatus->writeable = 0;
		//l = sprintf((char *)&buf[LWS_SEND_BUFFER_PRE_PADDING], data);
		for(i = 0;i<size;i++)
		{
			buf[LWS_SEND_BUFFER_PRE_PADDING + i] = data[i];
		}
		n = libwebsocket_write(pWebSocketStatus->wsi, &buf[LWS_SEND_BUFFER_PRE_PADDING], size, opts | LWS_WRITE_TEXT);

		if(n<0)
			fprintf(stderr, "Error with libwebsocket_write\n");
	}
	else
	{
		printf("NOT WRITEABLE\n");
	}
	return;
}

int initWebsocketContext(webSocketState_t *pWebSocketStatus)
{
	pthread_t ws_thread;
	int rc;

    memset(pWebSocketStatus, 0, sizeof(webSocketState_t));

	rc = pthread_create(&ws_thread, NULL, websocketThread, (void *)pWebSocketStatus);

	if (rc)
	{
		printf("ERROR; return code from pthread_create() is %d\n", rc);
		return -1;
	}

    return 0;
}
