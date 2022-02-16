//file: webserver_esp.cpp
/* ----------------------------------------------------------------------------------------
   web routines for the pool/spa controller

   This runs on the alternate core 0 CPU of the ESP32 processor, while the main
   pool/spa controller code is running on core 1.

   As a web server we provide the current status as the home page, with active
   hotspots for the 8 real buttons and the rotary temperaturec control that can be
   activated by clicking on them in the browser. The home page will auto-refresh
   every 5 seconds.

   The home page also has navigation buttons to these subpages:
     /log         show the whole event log
     /visitors    show the list of IP addresses who visited
     /temps       show the temperature history when the pool or spa was being heated

   See the main module for other details and the change log.
   ----------------------------------------------------------------------------------------
   Copyright (c) 2022 Len Shustek
   The MIT License (MIT)
   Permission is hereby granted, free of charge, to any person obtaining a copy of this software
   and associated documentation files (the "Software"), to deal in the Software without
   restriction, including without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or
   substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
   BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ------------------------------------------------------------------------------------------------*/


/* notes about using ESP32 wifi
   https://github.com/rdalla/arq_webserver
   https://github.com/espressif/esp-idf/blob/733fbd9ecc8ac0780de51b3761a16d1faec63644/examples/wifi/getting_started/station/main/station_example_main.c
   https://iotsimplified.com/2020/06/14/make-an-esp32-web-server-with-http-esp-idf/
   https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_server/simple/main/main.c
   https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/file_serving

   IPaddress symbols: C:\Users\len\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.1\tools\sdk\esp32\include\esp_netif\include\esp_netif_ip_addr.h

   Arduino-compatible wifi library: C:\Users\len\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.1\libraries\WiFi\src

   This requires that CONFIG_HTTPD_MAX_REQ_HDR_LEN be changed from 512 to 1024.
   See https://github.com/espressif/arduino-esp32/issues/5969
   
   The config file and the libraries are here in GitHub:
   https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/esp32/sdkconfig
   https://github.com/espressif/arduino-esp32/tree/master/tools/sdk/esp32/lib
   On my computer the library source code is here:
   C:\Users\len\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.1\libraries
   the binaries are here:
   C:\Users\len\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.1\tools\sdk\esp32\lib
   the include files are here:
   C:\Users\len\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.1\tools\sdk\esp32\include
*/

#include "Wifi_names.h"
#include "controller_03.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include <esp_http_server.h>
#include "esp_event.h"
//#include <http_parser.h>
#include "Arduino.h"
#include "lwip/sockets.h"
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define MAX_IP_ADDRESSES 50     // maximum visitors we keep track of
#define ESP_MAX_RETRY   3       // maximum retries to connect to WiFi
typedef uint32_t IPV4address;

#define HTML_DOWNARROW   "&#8595;"   // HTML arrow symbols
#define HTML_UPARROW     "&#8593;"
#define HTML_RIGHTARROW  "&#8594;"
#define HTML_LEFTARROW   "&#8592;"

#define ESP_CHECKERR(stmt) { \
      esp_err_t error = (stmt);   \
      assert_that(error == ESP_OK, "ESP err %d line %d", error, __LINE__); }

//******** status info that the main task in the other CPU displays

char webserver_address[25] = {0 }; // for the main program to display
int connect_successes = 0;
int connect_failures = 0;
int client_requests = 0;

//*********** visitor history routines  ********

struct client_t { // history of the clients whose web browsers made requests
   IPV4address ip_address;
   long count;
   struct datetime first_time, recent_time;
   bool gave_password; }
clients[MAX_IP_ADDRESSES],
        *current_client;
long requests_processed = 0;
long wifi_connects = 0, wifi_connectfails = 0, wifi_disconnects = 0, wifi_resets = 0;

char *format_ip_address(IPV4address addr, char *str) {
   // str should be at least 21 bytes long:  xxx.xxx.xxx.xxx:nnnn
   byte *bytes = (byte *)&addr;
   sprintf(str, "%d.%d.%d.%d:%d", bytes[0], bytes[1], bytes[2], bytes[3], WIFI_PORT);
   return str; }

struct client_t * remember_ip_address(httpd_req_t *req, IPV4address addr) { // record an IP address in our table
   int empty_ndx = -1, min_ndx = -1, min_count = INT_MAX;
   for (int ndx = 0; ndx < MAX_IP_ADDRESSES; ++ndx) {
      if (clients[ndx].ip_address == addr) {// IP address is already in the table
         clients[ndx].recent_time = now;
         ++clients[ndx].count;
         return &clients[ndx]; }
      if (clients[ndx].count == 0) empty_ndx = ndx; // remember empty slot
      else if (clients[ndx].count < min_count) { // remember min count slot
         min_ndx = ndx; min_count = clients[ndx].count; } }
   if (empty_ndx >= 0) min_ndx = empty_ndx;
   clients[min_ndx].ip_address = addr; // create a new entry for it
   clients[min_ndx].count = 1;
   clients[min_ndx].gave_password = false;
   clients[min_ndx].first_time = clients[min_ndx].recent_time = now;
   return &clients[min_ndx]; }

IPV4address get_remote_ip(httpd_req_t *req) {
   int socket = httpd_req_to_sockfd(req);
   struct sockaddr_in6 addr_in;
   socklen_t addrlen = sizeof(addr_in);
   IPV4address v4addr = 0;
   if (lwip_getpeername(socket, (struct sockaddr *)&addr_in, &addrlen) != -1) {
      v4addr = *(IPV4address *)(addr_in.sin6_addr.un.u32_addr + 3);
      //char str[30];
      //dprint("Remote IP is %s, hex %08X\n", format_ip_address(v4addr, str), v4addr);
   }
   else dprint("Error getting client's IP address\n");
   return v4addr; }

void report_ip_address(httpd_req_t *req, const char *content) {
   IPV4address addr = get_remote_ip(req);
   char str[30];
   ++client_requests;
   dprint("IP %s %s %s %s\n",
          format_ip_address(addr, str),
          req->method == HTTP_GET ? "GET" : req->method == HTTP_POST ? "POST" : "???",
          req->uri,
          content);
   remember_ip_address(req, addr); }

void sort_clients(void) { // sort the client array by most recent visit time
   int next = 1;  // next element to sort in the insertion sort
   while (next < MAX_IP_ADDRESSES) {
      struct client_t temp;
      int insert;
      temp = clients[next];
      insert = next - 1; // possible place to insert it
      while (insert >= 0
             && compare_datetime(&clients[insert].recent_time, &temp.recent_time) < 0) {
         clients[insert + 1] = clients[insert];
         --insert; }
      clients[insert + 1] = temp;
      ++next; } }

void visitors_dump(void * parm, void (*print)(void * parm, const char *line, ...)) {
   sort_clients();
   for (int ndx = 0; ndx < MAX_IP_ADDRESSES; ++ndx)
      if (clients[ndx].count > 0) {
         char buf[30], datestr[30];
         print(parm, "IP %s visited %d times, first at %s",
               format_ip_address(clients[ndx].ip_address, buf),
               clients[ndx].count,
               format_datetime(&clients[ndx].first_time, datestr));
         if (compare_datetime(&clients[ndx].recent_time, &clients[ndx].first_time) != 0)
            print(parm, ", recently at %s", format_datetime(&clients[ndx].recent_time, datestr));
         print(parm, "%s<br>\r\n",
               clients[ndx].gave_password ? "; password given" : ""); } }

//*********** common routines for responses  ********

#define BUT_H_SPACING_PX 60  // button separation in pixels

// our standard response headers for HTML requests

#define RESPONSE_PROLOG  "<!DOCTYPE HTML>\r\n<html><head>" \
   "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=windows-1252\">"
#define RESPONSE_REFRESH_HEADER "<meta http-equiv=\"refresh\" content=\"5\">"  // optional auto-refresh
#define RESPONSE_HOMEBUTTON "<a href='/'><button>home</button></a><br><br>\r\n"

const char *response_headers[] = { // an array of pointers to remaining headers
   // note some without trailing commas, which concatenates short lines together
   "\r\n <style>\r\n",   // define our CSS styles...
   ".lcd {font-family: monospace; font-size:x-large; width:23ch; border:3px; margin-left:100px; border-style:solid; border-color:blue; border-radius:10px; padding:1em}\r\n",
   ".led{height:20px; width:20px; border-radius:50%; background-color:blue; display:inline-block; position:absolute}\r\n",
   ".button {height:25px; width:25px; border:4px solid Gray; border-radius:50%; background-color:LightGray; color:black; cursor: pointer;}\r\n",
   ".button:hover{background-color:black;}\r\n",
   ".arrowbutton {transform:rotate(180deg); font-size:40px; color:grey; background-color:white; border:0px; margin:0px}\r\n",
   ".arrowbutton:hover{color:black;}\r\n",
   ".tablecell {width:" stringify(BUT_H_SPACING_PX) "px;text-align:center;}\r\n"
   "</style></head><body>\r\n",
   "<h1 style=\"text-indent:115px\">" TITLE "</h1>\r\n",
   "<a href='/log'><button>log</button></a>&emsp;\r\n",
   "<a href='/temps'><button>temperature history</button></a>&emsp;\r\n",
   "<a href='/visitors'><button>visitors</button></a>&emsp;\r\n",
   0 };

//<input type="button" onclick="window.location.href='https://www.w3docs.com';" value="w3docs" />
//<a href='https://www.freecodecamp.org/'><button>Link To freeCodeCamp</button></a>


void send_standard_headers(httpd_req_t *req, bool homepage) {
   httpd_resp_set_hdr(req, "Connection", "close");
   httpd_resp_send_chunk(req, RESPONSE_PROLOG, HTTPD_RESP_USE_STRLEN);
   if (homepage) // add HTML code to auto-refresh the home page
      httpd_resp_send_chunk(req, RESPONSE_REFRESH_HEADER, HTTPD_RESP_USE_STRLEN);
   for (const char **ptr = response_headers; *ptr; ++ptr)
      httpd_resp_send_chunk(req, *ptr, HTTPD_RESP_USE_STRLEN);
   if (!homepage) // add the "home" navigation button
      httpd_resp_send_chunk(req, RESPONSE_HOMEBUTTON, HTTPD_RESP_USE_STRLEN); }

void send_standard_close(httpd_req_t *req) {
   httpd_resp_send_chunk(req, " </body></html>\n", HTTPD_RESP_USE_STRLEN);
   httpd_resp_send_chunk(req, NULL, 0); }

void expand_arrows_and_blanks(httpd_req_t *req, int row) {
   // expand our arrow symbols into HTML arrows, blanks into &nbsp, then send to client
   //todo: if cursor is blinking, underline the cursor character.
   char outmsg[MAXLINE];
   char *dst = outmsg, *src;
   int col;
   for (src = lcdbuf[row], col = 0; *src && dst - outmsg < MAXLINE - 15; ++src, ++col) {
      if (*src == LEFTARROW[0]) {
         strcpy(dst, HTML_LEFTARROW); dst += sizeof(HTML_LEFTARROW) - 1; }
      else if (*src == UPARROW[0]) {
         strcpy(dst, HTML_UPARROW); dst += sizeof(HTML_UPARROW) - 1; }
      else if (*src == RIGHTARROW[0]) {
         strcpy(dst, HTML_RIGHTARROW); dst += sizeof(HTML_RIGHTARROW) - 1; }
      else if (*src == DOWNARROW[0]) {
         strcpy(dst, HTML_DOWNARROW); dst += sizeof(HTML_DOWNARROW) - 1; }
      else if (*src == ' ') {
         strcpy(dst, "&nbsp;"); dst += 6; }
      else if (lcd_cursorblinking && row == lcdrow && col == lcdcol) {
         dst += sprintf(dst, "<u>%c</u>", *src); } // underline where blinking cursor is
      else *dst++ = *src; }
   strcpy(dst, "<br>\n");
   httpd_resp_send_chunk(req, outmsg, HTTPD_RESP_USE_STRLEN); }

void show_lcd_screen(httpd_req_t *req) {
   httpd_resp_send_chunk(req, "<p class=\"lcd\">\r\n", HTTPD_RESP_USE_STRLEN); // start LCD  box
   for (int row = 0; row < 4; ++row)  // show contents of the LCD display
      expand_arrows_and_blanks(req, row);
   httpd_resp_send_chunk(req, "</p>\n", HTTPD_RESP_USE_STRLEN); }

void show_buttons(httpd_req_t *req) {
#define BUTLINESIZE 300
   //we use the root (instead of /pushbutton) for the POST so that it will be the home page that is refreshed automatically
   httpd_resp_send_chunk(req, "<br><form action=\"/\" method=\"post\">\n", HTTPD_RESP_USE_STRLEN);
   {
      char line[BUTLINESIZE];
      //display the temperature control rotary, with curved arrow button on either side to cause it to rotate
      httpd_resp_send_chunk(req, "<div style=\"height:60px;display:flex;align-items:center\"><button class=\"arrowbutton\" type=\"submit\" name=\"temp\" value=\"up\">&cudarrl;</button>", HTTPD_RESP_USE_STRLEN);
      //don't add \r\n to avoid whitespace between arrows and the button symbol
      snprintf(line, sizeof(line), "<button style=\"height:35px;width:35px;margin:0px;"
               //"position:relative; top:50%%; transform:translateY(-50%%);" // center vertically
               "border:4px solid gray; border-radius:50%%; background-color:%s\"></button>",
               heater_mode == HEATING_NONE ? "LightGray" : heater_on ? "Red" : "LightBlue");
      httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
      httpd_resp_send_chunk(req, "<button class=\"arrowbutton\" type=\"submit\" name=\"temp\" value=\"down\">&larrpl;</button></div>&nbsp;&nbsp;temperature<br>\r\n", HTTPD_RESP_USE_STRLEN);

      // start a table with the button in row 1
      httpd_resp_send_chunk(req, "<br><br><table><tbody><tr>\r\n", HTTPD_RESP_USE_STRLEN);
      for (int but = 0; but < NUM_BUTTONS; ++but) { // draw the buttons
         snprintf(line, sizeof(line), "<td class=\"tablecell\"><button class=\"button\"%s type=\"submit\" name=\"button\" value=\"%d\"> </button></td>\r\n",
                  leds_on & led_masks[but] ? " style=\"border-color:LimeGreen\"" : "", // add green ring color if the light is on
                  but); // the button number
         httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN); }
      httpd_resp_send_chunk(req, "</tr>\r\n", HTTPD_RESP_USE_STRLEN); }
   {
      // draw the labels in rows 2, 3, and 4
      char row[BUTLINESIZE];
      static const char *button_labels[NUM_BUTTONS][3] = {
         "heat", "spa", "<b>&larr;</b>",
         "heat", "pool", "<b>&rarr;</b>",
         "spa", "jets", "<b>&darr;</b>",
         "pool", "light", "<b>&uarr;</b>",
         "filter", "spa", " ",
         "filter", "pool", " ",
         "spa", "level", " ",
         "program", " ", " " };
      for (int rownum = 0; rownum < 3; ++rownum) {
         int rowchars;
         rowchars = snprintf(row, sizeof(row), "<tr>");
         for (int but = 0; but < NUM_BUTTONS; ++but) {
            rowchars += snprintf(row + rowchars, sizeof(row) - rowchars,
                                 "<td class=\"tablecell\">%s</td>",
                                 button_labels[but][rownum]); }
         snprintf(row + rowchars, sizeof(row) - rowchars, "</tr>\r\n");
         httpd_resp_send_chunk(req, row, HTTPD_RESP_USE_STRLEN); }
      httpd_resp_send_chunk(req, "</table></form></div>\r\n", HTTPD_RESP_USE_STRLEN); } }


//******************** / (root)  **********************************

// handler for root URI
esp_err_t root_GET_handler(httpd_req_t *req) {
   report_ip_address(req, "");
   send_standard_headers(req, true);
   show_lcd_screen(req);
   show_buttons(req);
   send_standard_close(req);
   return ESP_OK; }

static const httpd_uri_t root = {
   .uri       = "/",
   .method    = HTTP_GET,
   .handler   = root_GET_handler,
   .user_ctx  = NULL };

//********************  /log  **********************************

void log_GET_printer(void * parm, const char *line) {
   char buf[210];
   snprintf(buf, sizeof(buf), "%s<br>\n", line);
   httpd_resp_send_chunk((httpd_req_t *)parm, buf, HTTPD_RESP_USE_STRLEN); }

esp_err_t log_GET_handler(httpd_req_t *req) {
   report_ip_address(req, "");
   send_standard_headers(req, false);
   log_dump(req, &log_GET_printer);
   send_standard_close(req);
   return ESP_OK; }

static const httpd_uri_t log_uri = {
   .uri       = "/log",
   .method    = HTTP_GET,
   .handler   = log_GET_handler };

//********************  /visitors  **********************************

void visitors_GET_printer(void * parm, const char *line, ...) {
   char buf[250];
   va_list arg_ptr;
   va_start(arg_ptr, line);
   vsnprintf(buf, sizeof(buf), line, arg_ptr);
   httpd_resp_send_chunk((httpd_req_t *)parm, buf, HTTPD_RESP_USE_STRLEN); }

esp_err_t visitors_GET_handler(httpd_req_t *req) {
   report_ip_address(req, "");
   send_standard_headers(req, false);
   visitors_dump(req, &visitors_GET_printer);
   send_standard_close(req);
   return ESP_OK; }

static const httpd_uri_t visitors_uri = {
   .uri       = "/visitors",
   .method    = HTTP_GET,
   .handler   = visitors_GET_handler };

//********************  /temp  **********************************

void temp_GET_printer(void * parm, const char *line) {
   char buf[100];
   snprintf(buf, sizeof(buf), "%s<br>\n", line);
   httpd_resp_send_chunk((httpd_req_t *)parm, buf, HTTPD_RESP_USE_STRLEN); }

esp_err_t temps_GET_handler(httpd_req_t *req) {
   report_ip_address(req, "");
   send_standard_headers(req, false);
   temphistory_dump(req, &temp_GET_printer);
   send_standard_close(req);
   return ESP_OK; }

static const httpd_uri_t gettemps = {
   .uri       = "/temps",
   .method    = HTTP_GET,
   .handler   = temps_GET_handler };

//********************  /favicon **********************************

esp_err_t favicon_GET_handler(httpd_req_t *req) {
   httpd_resp_set_hdr(req, "Connection", "close");
   httpd_resp_set_hdr(req, "Content-Type", "image/jpg");
   extern char iconimagejpg[]; // binary jpg encoding of the image
   extern int iconimagesize;   // its length
   report_ip_address(req, "");
   httpd_resp_send(req, iconimagejpg, iconimagesize);
   return ESP_OK; }

static const httpd_uri_t favicon = {
   .uri       = "/favicon.ico",
   .method    = HTTP_GET,
   .handler   = favicon_GET_handler,
   .user_ctx  = NULL };

//********************  post code  **********************************

// this is called when a POST button is pushed on the home page
static esp_err_t button_POST_handler(httpd_req_t *req) {
   char postdata[25];
   int datalen = req->content_len;
   // for some reason, req->content_len is zero even though there is data!
   datalen = httpd_req_recv(req, postdata, sizeof(postdata) - 1); // read bytes anyway
   postdata[datalen] = 0; // make it a C string
   report_ip_address(req, postdata);
   int button;
   if (sscanf(postdata, "button=%d", &button) == 1
         && button >= 0 && button <= 7) {
      dprint("got push of button %d\n", button);
      button_webpushed[button] = true; }
   else if (strcmp(postdata, "temp=up") == 0) {
      dprint("got push of temp up\n");
      temp_change(+1); }
   else if (strcmp(postdata, "temp=down") == 0) {
      dprint("got push of temp down\n");
      temp_change(-1); }
   else dprint("in button post handler, read %d unexpected bytes: %s\n", datalen, postdata);
   delay(500); // wait for the button to be processed in the other task
   send_standard_headers(req, true); // do home page response
   show_lcd_screen(req);
   show_buttons(req);   send_standard_close(req);
   return ESP_OK; }

static const httpd_uri_t send_button_push = {
   .uri       = "/",  // we use the root URI so that subsequent refresh works on the home page
   .method    = HTTP_POST,
   .handler   = button_POST_handler,
   .user_ctx  = NULL };

//****************** server startup ***************************

static httpd_handle_t server = NULL;

static httpd_handle_t start_webserver(void) {
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
   config.lru_purge_enable = true;
   config.server_port = WIFI_PORT;
   dprint("Starting server on port %d\n", config.server_port);
   ESP_CHECKERR(httpd_start(&server, &config));
   ESP_CHECKERR(httpd_register_uri_handler(server, &root));
   ESP_CHECKERR(httpd_register_uri_handler(server, &favicon));
   ESP_CHECKERR(httpd_register_uri_handler(server, &log_uri));
   ESP_CHECKERR(httpd_register_uri_handler(server, &gettemps));
   ESP_CHECKERR(httpd_register_uri_handler(server, &visitors_uri));
   ESP_CHECKERR(httpd_register_uri_handler(server, &send_button_push));
   return server; }

static void stop_webserver(httpd_handle_t server) {
   // Stop the httpd server
   httpd_stop(server); }


//********** Wifi connection routines **********************

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
   - we are connected to the AP with an IP
   - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

int wifi_get_rssi(void) {
   wifi_ap_record_t info;
   if (esp_wifi_sta_get_ap_info(&info) == ESP_OK)
      return info.rssi;
   else return 0; }

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
   if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      esp_wifi_connect(); }
   else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      if (s_retry_num < ESP_MAX_RETRY) {
         esp_wifi_connect();
         s_retry_num++;
         dprint("retry to connect to the AP\n"); }
      else {
         xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); }
      dprint("connect to the AP failed for %s\n", WIFI_SSID); }
   else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
      snprintf(webserver_address, sizeof(webserver_address), IPSTR ":%d", IP2STR(&event->ip_info.ip), WIFI_PORT);
      dprint("our IP address: %s\n", webserver_address);
      s_retry_num = 0;
      xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); } }

void wifi_init_sta(void) {
   s_wifi_event_group = xEventGroupCreate();
   ESP_CHECKERR(esp_netif_init());
   ESP_CHECKERR(esp_event_loop_create_default());
   esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();
   #ifdef WIFI_IPADDR // use a static IP address?
   // code from https://www.esp32.com/viewtopic.php?t=14689, approximately
   esp_netif_dhcpc_stop(my_sta);
   esp_netif_ip_info_t ip_info;
   // esp_netif_ip_info_t: C:\Users\len\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.1\tools\sdk\esp32\include\esp_netif\include\esp_netif_types.h
   esp_netif_set_ip4_addr(&ip_info.ip, WIFI_IPADDR);
   esp_netif_set_ip4_addr(&ip_info.gw, WIFI_GATEWAYADDR);
   esp_netif_set_ip4_addr(&ip_info.netmask, WIFI_SUBNET);
   esp_netif_set_ip_info(my_sta, &ip_info);
   #endif
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_CHECKERR(esp_wifi_init(&cfg));
   esp_event_handler_instance_t instance_any_id;
   esp_event_handler_instance_t instance_got_ip;
   ESP_CHECKERR(esp_event_handler_instance_register(WIFI_EVENT,
                ESP_EVENT_ANY_ID,
                &event_handler,
                NULL,
                &instance_any_id));
   ESP_CHECKERR(esp_event_handler_instance_register(IP_EVENT,
                IP_EVENT_STA_GOT_IP,
                &event_handler,
                NULL,
                &instance_got_ip));
   wifi_config_t wifi_config = {
      .sta = {
         {.ssid = WIFI_SSID },
         {.password = WIFI_PASSWORD },
         /* Setting a password implies station will connect to all security modes including WEP/WPA.
            However these modes are deprecated and not advisable to be used. Incase your Access point
            doesn't support WPA2, these mode can be enabled by commenting below line */
         //.threshold.authmode = WIFI_AUTH_WPA2_PSK,
         .pmf_cfg = {
            .capable = true,
            .required = false }, }, };
   ESP_CHECKERR(esp_wifi_set_mode(WIFI_MODE_STA) );
   ESP_CHECKERR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
   ESP_CHECKERR(esp_wifi_start() );
   /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
      number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
   EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);
   /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
      happened. */
   if (bits & WIFI_CONNECTED_BIT) {
      ++connect_successes;
      dprint("connected to SSID \"%s\" with password \"%s\"\n",
             WIFI_SSID, WIFI_PASSWORD); }
   else if (bits & WIFI_FAIL_BIT) {
      ++connect_failures;
      dprint("Failed to connect to SSID %s, password %s\n",
             WIFI_SSID, WIFI_PASSWORD); }
   else {
      dprint("UNEXPECTED EVENT\n"); }
   /* The event will not be processed after unregister */
   ESP_CHECKERR(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
   ESP_CHECKERR(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
   vEventGroupDelete(s_wifi_event_group); }

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
   httpd_handle_t* server = (httpd_handle_t*) arg;
   if (*server) {
      dprint("Stopping webserver\n");
      stop_webserver(*server);
      *server = NULL; } }

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data) {
   httpd_handle_t* server = (httpd_handle_t*) arg;
   if (*server == NULL) {
      dprint("Starting webserver\n");
      *server = start_webserver(); } }

void  webserver_task(void *parm) {
   dprint("CONFIG_HTTPD_MAX_REQ_HDR_LEN = %d\n", CONFIG_HTTPD_MAX_REQ_HDR_LEN);
   wifi_init_sta();
   /* Register event handlers to stop the server when Wi-Fi is disconnected,
      and re-start it upon connection.  */
   ESP_CHECKERR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
   ESP_CHECKERR(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

   server = start_webserver(); // start server for the first time

   while (1) {// now idle
      vTaskDelay(11 / portTICK_PERIOD_MS);
      watchdog_poke(); } }

//*
