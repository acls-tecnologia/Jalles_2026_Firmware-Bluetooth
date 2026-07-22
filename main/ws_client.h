#pragma once

#include "esp_err.h"
#include <stdbool.h>

/*
 * Inicia o cliente WebSocket.
 * O backend reconhece o dispositivo como ONLINE
 * automaticamente quando a conexão é estabelecida.
 *
 * token: token de autenticação enviado na URI (?token=...)
 */

esp_err_t ws_client_start(const char *token);

/*
 * Para e destrói o cliente WebSocket.
 * O backend marcará o dispositivo como OFFLINE
 * ao detectar o DISCONNECT ou timeout.
 */
void ws_client_stop(void);

/*
 * Retorna true se o WebSocket estiver conectado
 */
bool ws_client_is_connected(void);
