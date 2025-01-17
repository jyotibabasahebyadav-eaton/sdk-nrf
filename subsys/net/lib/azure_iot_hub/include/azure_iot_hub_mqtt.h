/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef AZURE_IOT_HUB_MQTT__
#define AZURE_IOT_HUB_MQTT__

#include <stdio.h>
#include <net/azure_iot_hub.h>
#include <zephyr/net/mqtt.h>

#ifdef __cplusplus
extern "C" {
#endif

enum mqtt_state {
	MQTT_STATE_UNINIT,
	MQTT_STATE_DISCONNECTED,
	MQTT_STATE_TRANSPORT_CONNECTING,
	MQTT_STATE_CONNECTING,
	MQTT_STATE_TRANSPORT_CONNECTED,
	MQTT_STATE_CONNECTED,
	MQTT_STATE_DISCONNECTING,

	MQTT_STATE_COUNT,
};

enum mqtt_helper_error {
	/* The received payload is larger than the payload buffer. */
	MQTT_HELPER_ERROR_MSG_SIZE,
};

typedef void (*mqtt_helper_handler_t)(struct mqtt_evt *evt);
typedef void (*mqtt_helper_on_connack_t)(enum mqtt_conn_return_code return_code);
typedef void (*mqtt_helper_on_disconnect_t)(int result);
typedef void (*mqtt_helper_on_publish_t)(struct azure_iot_hub_buf topic_buf,
					 struct azure_iot_hub_buf payload_buf);
typedef void (*mqtt_helper_on_puback_t)(uint16_t message_id, int result);
typedef void (*mqtt_helper_on_suback_t)(uint16_t message_id, int result);
typedef void (*mqtt_helper_on_pingresp_t)(void);
typedef void (*mqtt_helper_on_error_t)(enum mqtt_helper_error error);

struct mqtt_helper_cfg {
	struct {
		mqtt_helper_on_connack_t on_connack;
		mqtt_helper_on_disconnect_t on_disconnect;
		mqtt_helper_on_publish_t on_publish;
		mqtt_helper_on_puback_t on_puback;
		mqtt_helper_on_suback_t on_suback;
		mqtt_helper_on_pingresp_t on_pingresp;
		mqtt_helper_on_error_t on_error;
	} cb;
};

struct mqtt_helper_conn_params {
	uint16_t port;
	/* The hostname must be null-terminated. */
	struct azure_iot_hub_buf hostname;
	struct azure_iot_hub_buf device_id;
	struct azure_iot_hub_buf user_name;
};

/* Initialize the MQTT helper.
 *
 * @retval 0 if successful.
 * @retval -EOPNOTSUPP if operation is not supported in the current state.
 * @return Otherwise a negative error code.
 */
int mqtt_helper_init(struct mqtt_helper_cfg *cfg);


/* Connect to an MQTT broker.
 *
 * @retval 0 if successful.
 * @retval -EOPNOTSUPP if operation is not supported in the current state.
 * @return Otherwise a negative error code.
 */
int mqtt_helper_connect(struct mqtt_helper_conn_params *conn_params);

/* Disconnect from the MQTT broker.
 *
 * @retval 0 if successful.
 * @retval -EOPNOTSUPP if operation is not supported in the current state.
 * @return Otherwise a negative error code.
 */
int mqtt_helper_disconnect(void);

/* Subscribe to MQTT topics.
 *
 * @retval 0 if successful.
 * @retval -EOPNOTSUPP if operation is not supported in the current state.
 * @return Otherwise a negative error code.
 */
int mqtt_helper_subscribe(struct mqtt_subscription_list *sub_list);

/* Publish an MQTT message.
 *
 * @retval 0 if successful.
 * @retval -EOPNOTSUPP if operation is not supported in the current state.
 * @return Otherwise a negative error code.
 */
int mqtt_helper_publish(const struct mqtt_publish_param *param);

/* Must be called when all MQTT operations are done to release resources and
 * allow for a new client. The client must be in a disconnected state.
 *
 * @retval 0 if successful.
 * @retval -EOPNOTSUPP if operation is not supported in the current state.
 * @return Otherwise a negative error code.
 */
int mqtt_helper_deinit(void);


#ifdef __cplusplus
}
#endif

#endif /* AZURE_IOT_HUB_MQTT__ */
