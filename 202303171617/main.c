#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "port_gen_os_driver.h"
#include "../data/setup_soc.cgen"

#ifdef DETECT_KEY
#error Sorry: feature DETECT_KEY not ported
#endif

#if (INGCHIPS_FAMILY != INGCHIPS_FAMILY_918)
// WIP: Porting to ING916
#endif


//static uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
//{
//    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
//                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nP3 : 0x%08X\n"
//                    "R12: 0x%08X\n",
//                    info->pc, info->lr, info->psr,
//                    info->r0, info->r1, info->r2, info->r3, info->r12);
//    for (;;);
//}

//static uint32_t cb_assertion(assertion_info_t *info, void *_)
//{
//    platform_printf("[ASSERTION] @ %s:%d\n",
//                         info->file_name,
//                    info->line_no);
//    for (;;);
//}

//static uint32_t cb_heap_out_of_mem(uint32_t tag, void *_)
//{
//    platform_printf("[OOM] @ %d\n", tag);
//    for (;;);
//}

//#define PRINT_PORT    APB_UART0

//uint32_t cb_putc(char *c, void *dummy)
//{
//    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
//    UART_SendData(PRINT_PORT, (uint8_t)*c);
//    return 0;
//}

//int fputc(int ch, FILE *f)
//{
//    cb_putc((char *)&ch, NULL);
//    return ch;
//}

//#ifdef LISTEN_TO_POWER_SAVING
//uint32_t on_lle_reset(void *dummy, void *user_data)
//{
//    (void)(dummy);
//    (void)(user_data);
//    *(uint32_t *)(0x40090064) = 0x400 | (0x01 << 8);
//    *(uint32_t *)(0x4007005c) = 0x80;
//    
//    platform_printf("lle reset\n");
//    return 0;
//}
//#endif

//void setup_peripherals(void)
//{
//    cube_setup_peripherals();
//    
//#ifdef LISTEN_TO_POWER_SAVING
//    PINCTRL_SetPadMux(1, IO_SOURCE_DEBUG_BUS);
//    on_lle_reset(NULL, NULL);
//#endif
//    
//    
//}



//uint32_t on_deep_sleep_wakeup(void *dummy, void *user_data)
//{
//    (void)(dummy);
//    (void)(user_data);
//    setup_peripherals();
//    static int test_i = 0;
//    test_i++;
//    platform_printf("wakeup loop: %d\n", test_i);
//    return 0;
//}

//uint32_t query_deep_sleep_allowed(void *dummy, void *user_data)
//{
//    (void)(dummy);
//    (void)(user_data);
//    // TODO: return 0 if deep sleep is not allowed now; else deep sleep is allowed
//    if (IS_DEBUGGER_ATTACHED())
//        return 1;
//    
//    static int sleep_test_i = 0;
//    sleep_test_i++;
//    platform_printf("sleep loop: %d\n", sleep_test_i);
//    return 1;
//}


#include "../src/main_shared.inc"
// TODO: add RTOS source code to the project.
uintptr_t app_main()
{
    _app_main();
    #if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)
    #elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
        platform_config(PLATFORM_CFG_DEEP_SLEEP_TIME_REDUCTION, 4500);
    #endif
    //platform_config(PLATFORM_CFG_POWER_SAVING, 0);
    
//    cube_soc_init();
//    
    platform_set_evt_callback(PLATFORM_CB_EVT_PROFILE_INIT, setup_profile, NULL);

//    // setup handlers
//    platform_set_evt_callback(PLATFORM_CB_EVT_HARD_FAULT, (f_platform_evt_cb)cb_hard_fault, NULL);
//    platform_set_evt_callback(PLATFORM_CB_EVT_ASSERTION, (f_platform_evt_cb)cb_assertion, NULL);
//    platform_set_evt_callback(PLATFORM_CB_EVT_HEAP_OOM, (f_platform_evt_cb)cb_heap_out_of_mem, NULL);
//    platform_set_evt_callback(PLATFORM_CB_EVT_ON_DEEP_SLEEP_WAKEUP, on_deep_sleep_wakeup, NULL);
//    platform_set_evt_callback(PLATFORM_CB_EVT_QUERY_DEEP_SLEEP_ALLOWED, query_deep_sleep_allowed, NULL);
//    platform_set_evt_callback(PLATFORM_CB_EVT_PUTC, (f_platform_evt_cb)cb_putc, NULL);
    
    platform_config(PLATFORM_CFG_POWER_SAVING, PLATFORM_CFG_ENABLE);
//    
//#ifdef LISTEN_TO_POWER_SAVING
//    platform_set_evt_callback(PLATFORM_CB_EVT_LLE_INIT, on_lle_reset, NULL);
//#endif
//    
//    
//    setup_peripherals();

    

    return (uintptr_t)os_impl_get_driver();
}

#define expand2(X)  #X
#define expand(X)   expand2(X)

const char welcome_msg[] = "Built with Azure ThreadX (" expand(THREADX_MAJOR_VERSION) "." expand(THREADX_MINOR_VERSION)  "." expand(THREADX_PATCH_VERSION) ")";


