/*
  enet.c - lwIP driver glue code for STM32H7xx processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if ETHERNET_ENABLE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <lwipopts.h>
#include <lwip/netif.h>
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"
#include "ethernet.h"

#if TCP_ECHOSERVER_ENABLE
#include "tcp_echoserver.h"
#endif

#include "grbl/report.h"
#include "grbl/nvs_buffer.h"

#include "networking/networking.h"
#if HTTP_ENABLE
#include "networking/httpd.h"
#endif

static volatile bool linkUp = false;
static char IPAddress[IP4ADDR_STRLEN_MAX];
static stream_type_t active_stream = StreamType_Null;
static network_services_t services = {0}, allowed_services;
static nvs_address_t nvs_address;
static network_settings_t ethernet, network;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static on_stream_changed_ptr on_stream_changed;
static char netservices[30] = ""; // must be large enough to hold all service names

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(newopt) {
        hal.stream.write(",ETH");
#if FTP_ENABLE
        if(services.ftp)
            hal.stream.write(",FTP");
#endif
    } else {
        hal.stream.write("[IP:");
        hal.stream.write(IPAddress);
        hal.stream.write("]" ASCII_EOL);

        if(active_stream == StreamType_Telnet || active_stream == StreamType_WebSocket) {
            hal.stream.write("[NETCON:");
            hal.stream.write(active_stream == StreamType_Telnet ? "Telnet" : "Websocket");
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

static void link_status_callback (struct netif *netif)
{
    bool isLinkUp = netif_is_link_up(netif);

    if(isLinkUp != linkUp) {
        linkUp = isLinkUp;
#if TELNET_ENABLE
        telnetd_notify_link_status(linkUp);
#endif
    }
}

static void netif_status_callback (struct netif *netif)
{
    ip4addr_ntoa_r(netif_ip_addr4(netif), IPAddress, IP4ADDR_STRLEN_MAX);

    if(netif->ip_addr.addr != 0) {

#if TELNET_ENABLE
        if(network.services.telnet && !services.telnet)
            services.telnet =  telnetd_init(network.telnet_port == 0 ? NETWORK_TELNET_PORT : network.telnet_port);
#endif

#if FTP_ENABLE
        if(network.services.ftp && !services.ftp)
            services.ftp = ftpd_init(network.ftp_port == 0 ? NETWORK_FTP_PORT : network.ftp_port);;
#endif

#if HTTP_ENABLE
        if(network.services.http && !services.http)
            services.http = httpd_init(network.http_port == 0 ? NETWORK_HTTP_PORT : network.http_port);
#endif

#if WEBSOCKET_ENABLE
        if(network.services.websocket && !services.websocket)
            services.websocket = websocketd_init(network.websocket_port == 0 ? NETWORK_WEBSOCKET_PORT : network.websocket_port);
#endif
    }
}

void link_check_state(struct netif *netif)
{
  static uint32_t EthernetLinkTimer;

  // Check Ethernet link state every 100ms
  if (HAL_GetTick() - EthernetLinkTimer >= 100)
  {
    EthernetLinkTimer = HAL_GetTick();
    ethernet_link_check_state(netif);
  }
}

static void enet_poll (sys_state_t state)
{
    static uint32_t last_ms0;
    uint32_t ms = hal.get_elapsed_ticks();

    link_check_state(netif_default);
    sys_check_timeouts();
    ethernetif_input(netif_default);

    if(linkUp) {

        if(ms - last_ms0 > 3) {
            last_ms0 = ms;
    #if TELNET_ENABLE
            if(services.telnet)
                telnetd_poll();
    #endif
    #if FTP_ENABLE
            if(services.ftp)
                ftpd_poll();
    #endif
    #if WEBSOCKET_ENABLE
            if(services.websocket)
                websocketd_poll();
    #endif
        }
    }

    on_execute_realtime(state);
}

bool enet_start (void)
{
    static struct netif ethif;

    if(nvs_address != 0) {

        *IPAddress = '\0';
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = enet_poll;

        memcpy(&network, &ethernet, sizeof(network_settings_t));

        lwip_init();

        if(network.ip_mode == IpMode_Static)
            netif_add(&ethif, (ip_addr_t *)&network.ip, (ip_addr_t *)&network.mask, (ip_addr_t *)&network.gateway, NULL, &ethernetif_init, &ethernet_input);
        else
            netif_add(&ethif, NULL, NULL, NULL, NULL, &ethernetif_init, &ethernet_input);

        netif_set_default(&ethif);
        netif_set_link_callback(netif_default, link_status_callback);
        netif_set_status_callback(netif_default, netif_status_callback);

        // Invoke the link & interface callback functions once manually, as not necessarily triggered on startup
        link_status_callback(netif_default);
        netif_status_callback(netif_default);

    #if LWIP_NETIF_HOSTNAME
        netif_set_hostname(netif_default, network.hostname);
    #endif

        if(network.ip_mode == IpMode_DHCP)
            dhcp_start(netif_default);

    #if TCP_ECHOSERVER_ENABLE
        // Echos all input on TCP port 7, useful for diagnostics and performance checks
        tcp_echoserver_init();
    #endif
    }

    return nvs_address != 0;
}

static inline void set_addr (char *ip, ip4_addr_t *addr)
{
    memcpy(ip, addr, sizeof(ip4_addr_t));
}

static void ethernet_settings_load (void);
static void ethernet_settings_restore (void);
static status_code_t ethernet_set_ip (setting_id_t setting, char *value);
static char *ethernet_get_ip (setting_id_t setting);
static status_code_t ethernet_set_services (setting_id_t setting, uint_fast16_t int_value);
static uint32_t ethernet_get_services (setting_id_t id);

static const setting_group_detail_t ethernet_groups [] = {
    { Group_Root, Group_Networking, "Networking" }
};

static const setting_detail_t ethernet_settings[] = {
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, netservices, NULL, NULL, Setting_NonCoreFn, ethernet_set_services, ethernet_get_services, NULL },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, ethernet.hostname, NULL, NULL },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL, Setting_NonCore, &ethernet.ip_mode, NULL, NULL },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.telnet_port, NULL, NULL },
#if FTP_ENABLE
    { Setting_FtpPort, Group_Networking, "FTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.ftp_port, NULL, NULL },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.http_port, NULL, NULL },
#endif
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.websocket_port, NULL, NULL }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t ethernet_settings_descr[] = {
    { Setting_NetworkServices, "Network services to enable. Consult driver documentation for availability." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_Hostname, "Network hostname." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_IpMode, "IP Mode." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_IpAddress, "Static IP address." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_Gateway, "Static gateway address." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_NetMask, "Static netmask." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_TelnetPort, "(Raw) Telnet port number listening for incoming connections." SETTINGS_HARD_RESET_REQUIRED },
#if FTP_ENABLE
    { Setting_FtpPort, "FTP port number listening for incoming connections." SETTINGS_HARD_RESET_REQUIRED },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, "HTTP port number listening for incoming connections." SETTINGS_HARD_RESET_REQUIRED },
#endif
    { Setting_WebSocketPort, "Websocket port number listening for incoming connections." SETTINGS_HARD_RESET_REQUIRED
                             "\\nNOTE: WebUI requires this to be HTTP port number + 1."
    }
};

#endif

static void ethernet_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = ethernet_groups,
    .n_groups = sizeof(ethernet_groups) / sizeof(setting_group_detail_t),
    .settings = ethernet_settings,
    .n_settings = sizeof(ethernet_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = ethernet_settings_descr,
    .n_descriptions = sizeof(ethernet_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = ethernet_settings_save,
    .load = ethernet_settings_load,
    .restore = ethernet_settings_restore
};

static status_code_t ethernet_set_ip (setting_id_t setting, char *value)
{
    ip_addr_t addr;

    if(ip4addr_aton(value, &addr) != 1)
        return Status_InvalidStatement;

    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_IpAddress:
            set_addr(ethernet.ip, &addr);
            break;

        case Setting_Gateway:
            set_addr(ethernet.gateway, &addr);
            break;

        case Setting_NetMask:
            set_addr(ethernet.mask, &addr);
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static char *ethernet_get_ip (setting_id_t setting)
{
    static char ip[IPADDR_STRLEN_MAX];

    switch(setting) {

        case Setting_IpAddress:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.ip, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_Gateway:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.gateway, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_NetMask:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.mask, ip, IPADDR_STRLEN_MAX);
            break;

        default:
            *ip = '\0';
            break;
    }

    return ip;
}

static status_code_t ethernet_set_services (setting_id_t setting, uint_fast16_t int_value)
{
    ethernet.services.mask = int_value & allowed_services.mask;

    return Status_OK;
}

static uint32_t ethernet_get_services (setting_id_t id)
{
    return (uint32_t)ethernet.services.mask;
}

void ethernet_settings_restore (void)
{
    strcpy(ethernet.hostname, NETWORK_HOSTNAME);

    ip4_addr_t addr;

    ethernet.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        set_addr(ethernet.ip, &addr);

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        set_addr(ethernet.gateway, &addr);

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        set_addr(ethernet.mask, &addr);
#else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        set_addr(ethernet.mask, &addr);
#endif

    ethernet.ftp_port = NETWORK_FTP_PORT;
    ethernet.telnet_port = NETWORK_TELNET_PORT;
    ethernet.http_port = NETWORK_HTTP_PORT;
    ethernet.websocket_port = NETWORK_WEBSOCKET_PORT;
    ethernet.services.mask = allowed_services.mask;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static void ethernet_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ethernet, nvs_address, sizeof(network_settings_t), true) != NVS_TransferResult_OK)
        ethernet_settings_restore();

    ethernet.services.mask &= allowed_services.mask;
}

static void stream_changed (stream_type_t type)
{
    if(type != StreamType_SDCard)
        active_stream = type;

    if(on_stream_changed)
        on_stream_changed(type);
}

bool enet_init (network_settings_t *settings)
{
    if((nvs_address = nvs_alloc(sizeof(network_settings_t)))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        on_stream_changed = grbl.on_stream_changed;
        grbl.on_stream_changed = stream_changed;

        settings_register(&setting_details);

        allowed_services.mask = networking_get_services_list((char *)netservices).mask;
    }

    return nvs_address != 0;
}

#endif
