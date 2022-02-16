// Wifi_names.h

#define TITLE "your location"
#define WIFI_SSID "your SSID"
#define WIFI_PASSWORD "your password"

#ifndef TITLE
   #define TITLE "pool/spa controller"
#endif
#ifndef WIFI_PORT
   #define WIFI_PORT 80 // default TCP port number
#endif

#if 0  // use static IP address?
   #define WIFI_IPADDR      192,168,86,123
   #define WIFI_GATEWAYADDR 192,168,86,1
   #define WIFI_SUBNET      255,255,255,0
#endif

