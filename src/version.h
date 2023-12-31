/*
 * Versioning stuff and history.
 *
 * 0.5.0: Retrieval of data from the sensors works.
 * 
 * 0.6.0: Connecting to the WiFi network.
 * 
 * 0.7.0: Home Assistant integration.
 * 
 * 0.7.1: Average temperature.
 * 
 * 0.8.0: Do not loop endlessly on failure to connect to the WiFi network.
 * 
 * 0.8.1: - Added short function comments
 *        - Reconnect to the WiFi network, if disconnected.
 *
 * 0.8.2: Set hostname
 * 
 * 0.8.3: - Force station mode
 *        - Run MQTT loop only if connected to the network
 */
 
#ifndef _THUMPRESS_VERSION_H_
#define _THUMPRESS_VERSION_H_

#define VERSION_STRING "0.8.3"

#endif