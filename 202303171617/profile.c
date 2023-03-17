#include <stdio.h>
#include "platform_api.h"
#include "att_db.h"
#include "gap.h"
#include "btstack_event.h"
#include "btstack_defines.h"

#include "port_gen_os_driver.h"

// GATT characteristic handles
#include "../data/gatt.const"

const static uint8_t adv_data[] = {
    #include "../data/advertising.adv"
};

#include "../data/advertising.const"

const static uint8_t scan_data[] = {
    #include "../data/scan_response.adv"
};

#include "../data/scan_response.const"

const static uint8_t profile_data[] = {
    #include "../data/gatt.profile"
};

gen_handle_t gen_handle;
static void timer_cb(void* user_data);

gen_handle_t gen_handle_1;
static void timer_cb_1(void* user_data);

gen_handle_t gen_handle_2;
static void timer_cb_2(void* user_data);

static void timer_cb(void* user_data)
{
    platform_printf("TIMER RUNNING!!...");
    port_timer_start(gen_handle_1);
}

static void timer_cb_1(void* user_data)
{
    platform_printf("TIMER1 RUNNING!!...");
    port_timer_start(gen_handle_2);
    
}

static void timer_cb_2(void* user_data)
{
    platform_printf("TIMER2 RUNNING!!...");
    port_timer_start(gen_handle);
}



static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
                                  uint8_t * buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {

    default:
        return 0;
    }
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, const uint8_t *buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {

    default:
        return 0;
    }
}

static void user_msg_handler(uint32_t msg_id, void *data, uint16_t size)
{
    switch (msg_id)
    {
        // add your code
    //case MY_MESSAGE_ID:
    //    break;
    default:
        ;
    }
}

const static ext_adv_set_en_t adv_sets_en[] = { {.handle = 0, .duration = 0, .max_events = 0} };

uint8_t addr[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
static void setup_adv(void)
{
    
    gap_set_adv_set_random_addr(0, addr);
    gap_set_ext_adv_para(0,
                            CONNECTABLE_ADV_BIT | SCANNABLE_ADV_BIT | LEGACY_PDU_BIT,
                            2000/0.625, 2000/0.625,    // Primary_Advertising_Interval_Min, Primary_Advertising_Interval_Max
                            PRIMARY_ADV_ALL_CHANNELS,  // Primary_Advertising_Channel_Map
                            BD_ADDR_TYPE_LE_RANDOM,    // Own_Address_Type
                            BD_ADDR_TYPE_LE_PUBLIC,    // Peer_Address_Type (ignore)
                            NULL,                      // Peer_Address      (ignore)
                            ADV_FILTER_ALLOW_ALL,      // Advertising_Filter_Policy
                            0x00,                      // Advertising_Tx_Power
                            PHY_1M,                    // Primary_Advertising_PHY
                            0,                         // Secondary_Advertising_Max_Skip
                            PHY_1M,                    // Secondary_Advertising_PHY
                            0x00,                      // Advertising_SID
                            0x00);                     // Scan_Request_Notification_Enable
    gap_set_ext_adv_data(0, sizeof(adv_data), (uint8_t*)adv_data);
    gap_set_ext_scan_response_data(0, sizeof(scan_data), (uint8_t*)scan_data);
    gap_set_ext_adv_enable(1, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);
}

#define USER_MSG_ID_REQUEST_SEND            1
#define USER_MSG_ID_KEY_EVENT               2
#define USER_MSG_ID_FIRST_WAKE_UP           3

void stack_on_first_wake_up(void)
{
    btstack_push_user_msg(USER_MSG_ID_FIRST_WAKE_UP, NULL, 0);
}

static void user_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    static const bd_addr_t rand_addr = { 0xFE, 0xEB, 0x9B, 0x3B, 0xCA, 0x10 };
    uint8_t event = hci_event_packet_get_type(packet);
    const btstack_user_msg_t *p_user_msg;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (event)
    {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
            break;
        gap_set_adv_set_random_addr(0, rand_addr);
        platform_config(PLATFORM_CFG_LL_LEGACY_ADV_INTERVAL, (1250 << 16) | 750);
        setup_adv();
        platform_set_timer(stack_on_first_wake_up, 200);
        break;

    case HCI_EVENT_COMMAND_COMPLETE:
        switch (hci_event_command_complete_get_command_opcode(packet))
        {
        // add your code to check command complete response
        // case :
        //     break;
        default:
            break;
        }
        break;

    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet))
        {
        case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE:
            att_set_db(decode_hci_le_meta_event(packet, le_meta_event_enh_create_conn_complete_t)->handle,
                       profile_data);
            break;
        default:
            break;
        }

        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        gap_set_ext_adv_enable(1, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);
        break;

    case ATT_EVENT_CAN_SEND_NOW:
        // add your code
        break;

    case BTSTACK_EVENT_USER_MSG:
        p_user_msg = hci_event_packet_get_user_msg(packet);
        user_msg_handler(p_user_msg->msg_id, p_user_msg->data, p_user_msg->len);
        break;

    default:
        break;
    }
}

static btstack_packet_callback_registration_t hci_event_callback_registration;

uint32_t setup_profile(void *data, void *user_data)
{
    gen_handle = port_timer_create(200, NULL, timer_cb);
    gen_handle_1 = port_timer_create(200, NULL, timer_cb_1);
    gen_handle_2 = port_timer_create(200, NULL, timer_cb_2);
    port_timer_start(gen_handle);
    
    platform_printf("setup profile\n");
    // Note: security has not been enabled.
    att_server_init(att_read_callback, att_write_callback);
    hci_event_callback_registration.callback = &user_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(&user_packet_handler);
    return 0;
}

