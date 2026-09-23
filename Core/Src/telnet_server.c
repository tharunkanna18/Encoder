#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "telnet_server.h"
#include <string.h>

static struct tcp_pcb *telnet_client = NULL;

/* Line buffer for incoming Hercules commands */
static char    cmd_buffer[64];
static uint16_t cmd_index = 0;

/* Implemented in main.c — parses the command, updates PWM, replies */
extern void Telnet_Process_Command(char *cmd);

static err_t telnet_accept(void *arg,
                           struct tcp_pcb *newpcb,
                           err_t err);

static err_t telnet_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        /* Hercules closed the connection */
        tcp_close(tpcb);
        telnet_client = NULL;
        return ERR_OK;
    }

    if (err != ERR_OK)
    {
        pbuf_free(p);
        return err;
    }

    /* Copy the whole receive into cmd_buffer, stripping any CR/LF
     * if present. We don't require a terminator -- each Hercules
     * "Send" click is treated as one complete command, since Hercules
     * doesn't reliably append CR/LF on its own. */
    cmd_index = 0;
    struct pbuf *q;
    for (q = p; q != NULL; q = q->next)
    {
        char *data = (char *)q->payload;
        for (uint16_t i = 0; i < q->len; i++)
        {
            char c = data[i];

            if (c == '\r' || c == '\n')
            {
                continue; /* strip terminator if it IS present */
            }

            if (cmd_index < (sizeof(cmd_buffer) - 1))
            {
                cmd_buffer[cmd_index++] = c;
            }
        }
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    if (cmd_index > 0)
    {
        cmd_buffer[cmd_index] = '\0';
        Telnet_Process_Command(cmd_buffer);
        cmd_index = 0;
    }

    return ERR_OK;
}

static err_t telnet_accept(void *arg,
                           struct tcp_pcb *newpcb,
                           err_t err)
{
    telnet_client = newpcb;
    cmd_index = 0;

    tcp_recv(newpcb, telnet_recv);

    const char *msg = "STM32 Telnet Server Connected\r\n"
                       "Commands: FREQ=<Hz>  DUTY=<0-100>  DIR=L|R  AUTO\r\n";

    tcp_write(newpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
    tcp_output(newpcb);

    return ERR_OK;
}

void Telnet_Server_Init(void)
{
    struct tcp_pcb *pcb;

    pcb = tcp_new();

    tcp_bind(pcb, IP_ADDR_ANY, 23);

    pcb = tcp_listen(pcb);

    tcp_accept(pcb, telnet_accept);
}

void Telnet_Send(char *msg)
{
    if (telnet_client != NULL)
    {
        tcp_write(telnet_client,
                  msg,
                  strlen(msg),
                  TCP_WRITE_FLAG_COPY);

        tcp_output(telnet_client);
    }
}
