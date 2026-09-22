#pragma once

// SynScan Wi-Fi dongle compatible bridge: UDP port 11880, one motor command per
// datagram (":j1\r" -> "=xxxxxx\r"). pysynscan's commUDP talks to this unchanged.
void synscanUdpBegin();
