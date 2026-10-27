#ifndef ROBO_NET_H
#define ROBO_NET_H

#include <stdbool.h>

#include "logic.h"

// Starts the access point, the DHCP server and the HTTP / WebSocket server.
// Prints the SSID, password and session token on the serial console.
// Returns false if the radio or the server could not start. Motors stay the
// caller's problem: this function never touches them.
bool net_start(const rr_config_t *cfg);

#endif
