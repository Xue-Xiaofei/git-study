#include "port_gen_os_driver.h"
#include "platform_api.h"
#include "tx_api.h"
#include "tx_low_power.h"
#include "ingsoc.h"

#ifndef RTOS_HEAP_SIZE
#define RTOS_HEAP_SIZE             (20 * 1024)
#endif

#ifndef CS_MAX_NEST_DEPTH
#define CS_MAX_NEST_DEPTH               20
#endif

#define TICK_PER_SECOND             1024

#define ms_to_ticks(ms) ((ms) * TICK_PER_SECOND / 1000)

static uint32_t heap[RTOS_HEAP_SIZE / sizeof(uint32_t)];
static TX_BYTE_POOL pool;

static void *port_malloc(size_t size)
{
    void *p;
    return tx_byte_allocate(&pool, &p, size, 0) == TX_SUCCESS ? p : NULL;
}

static void port_free(void *p)
{
    tx_byte_release(p);
}

typedef VOID (*fun_void_ul_f)(ULONG id);

gen_handle_t port_timer_create(
        uint32_t timeout_in_ms,
        void *user_data,
        void (* timer_cb)(void *)
    )
{
    TX_TIMER *timer = (TX_TIMER *)port_malloc(sizeof(TX_TIMER));
    tx_timer_create(timer, NULL,
                    (fun_void_ul_f)timer_cb, (ULONG)user_data, 
                    ms_to_ticks(timeout_in_ms), 0, TX_NO_ACTIVATE);
    return timer;
}

void port_timer_start(gen_handle_t timer)
{
    TX_TIMER *t = (TX_TIMER *)timer;
    tx_timer_activate(t);
}

void port_timer_stop(gen_handle_t timer)
{
    TX_TIMER *t = (TX_TIMER *)timer;
    tx_timer_deactivate(t);
}

void port_timer_delete(gen_handle_t timer)
{
    TX_TIMER *t = (TX_TIMER *)timer;
    tx_timer_delete(t);
    port_free(t);
}

// ThreadX: Numerically smaller values imply higher priority
#define APP_PRIO_LOW               4
#define APP_PRIO_HIGH              2

gen_handle_t port_task_create(
        const char *name,
        void (*entry)(void *),
        void *parameter,
        uint32_t stack_size,                    // stack size in bytes
        enum gen_os_task_priority priority
    )
{
    TX_THREAD *thread = (TX_THREAD *)port_malloc(sizeof(TX_THREAD));
    VOID *stack = port_malloc(stack_size);
    tx_thread_create(thread, (char *)name, 
                     (fun_void_ul_f)entry, (ULONG)parameter,
                     stack, stack_size,
                     priority == GEN_TASK_PRIORITY_LOW ? APP_PRIO_LOW : APP_PRIO_HIGH,
                     2,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
    return (gen_handle_t)thread;
}

typedef struct
{
    TX_QUEUE queue;
    int msg_len;
    uint32_t msg[];
} tx_queue_sup_t;

gen_handle_t port_queue_create(int len, int item_size)
{
    // msg size is in 32-bit words 
    int word_size = (item_size + 3) >> 2;
    int queue_byte_size = len * word_size * 4;
    tx_queue_sup_t *queue = (tx_queue_sup_t *)port_malloc(sizeof(tx_queue_sup_t) + word_size * 4);
    queue->msg_len = item_size;
    VOID *queue_start = (VOID *)port_malloc(queue_byte_size);

    if (tx_queue_create(&queue->queue, NULL, word_size, queue_start, queue_byte_size) != TX_SUCCESS)
        platform_raise_assertion(__FILE__, __LINE__);

    return (gen_handle_t)queue;
}

int port_queue_send_msg(gen_handle_t queue, void *msg)
{
    tx_queue_sup_t *q = (tx_queue_sup_t *)queue;
    return tx_queue_send(&q->queue, msg, TX_WAIT_FOREVER);
}

// return 0 if msg received; otherwise failed (timeout)
int port_queue_recv_msg(gen_handle_t queue, void *msg)
{
    tx_queue_sup_t *q = (tx_queue_sup_t *)queue;
    if (TX_SUCCESS == tx_queue_receive(&q->queue, q->msg, TX_WAIT_FOREVER))
    {
        memcpy(msg, q->msg, q->msg_len);
        return 0;
    }
    else
        return 1;
}    

gen_handle_t port_event_create()
{
    TX_EVENT_FLAGS_GROUP *group = (TX_EVENT_FLAGS_GROUP *)port_malloc(sizeof(TX_EVENT_FLAGS_GROUP));
    tx_event_flags_create(group, NULL);
    
    return (gen_handle_t)group;
}  

// return 0 if msg received; otherwise failed (timeout)
int port_event_wait(gen_handle_t event)
{
    ULONG actual_flags_ptr;
    return tx_event_flags_get((TX_EVENT_FLAGS_GROUP *)event, 1,
                    TX_AND_CLEAR, &actual_flags_ptr, TX_WAIT_FOREVER);
}

// event_set(event) will release the task in waiting.
void port_event_set(gen_handle_t event)
{    
    tx_event_flags_set((TX_EVENT_FLAGS_GROUP *)event, 1, TX_OR);
}

extern void _tx_timer_interrupt(void);
extern void __tx_PendSVHandler(void);
static void port_tx_initialize_kernel(void);
static void port_tx_start(void);
static void idle_thread_func(ULONG param);
static TX_THREAD idle_thread_handle;
static uint8_t idle_thread_stack[TX_MINIMUM_STACK];

static void no_op(void)
{
    platform_raise_assertion(__FILE__, __LINE__);
}

UINT interrupt_states[CS_MAX_NEST_DEPTH] = {0};
int nest_levelxxx[100];
int nest_level = 0;

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_LR(void) 
{ 
  register uint32_t result; 

  __ASM volatile ("MOV %0, LR\n" : "=r" (result) ); 
  return(result); 
} 

void port_enter_critical(void)
{
    UINT v = __disable_interrupts();
    if (nest_level == 0)
        interrupt_states[nest_level] = v;
    nest_level++;
}

void port_leave_critical(void)
{
    nest_level--;
    if (nest_level < 0)
        platform_raise_assertion(__FILE__, __LINE__);
    if (0 == nest_level)
        __restore_interrupt(interrupt_states[nest_level]);
}

const gen_os_driver_t gen_os_driver =
{
    .timer_create = port_timer_create,
    .timer_start = port_timer_start,
    .timer_stop = port_timer_stop,
    .timer_delete = port_timer_delete,
    
    .task_create = port_task_create,
    
    .queue_create = port_queue_create,
    .queue_send_msg = port_queue_send_msg,
    .queue_recv_msg = port_queue_recv_msg,
    
    .event_create = port_event_create,
    .event_set = port_event_set,
    .event_wait = port_event_wait,
    
    .malloc = port_malloc,
    .free = port_free,
    .enter_critical = port_enter_critical,
    .leave_critical = port_leave_critical,
    .os_start = port_tx_start,
    .tick_isr = _tx_timer_interrupt,
    .svc_isr = no_op,
    .pendsv_isr = __tx_PendSVHandler,
};
    
const gen_os_driver_t *os_impl_get_driver(void)
{
    tx_byte_pool_create(&pool, NULL, heap, sizeof(heap));
    port_tx_initialize_kernel();

    return &gen_os_driver;
}

void vApplicationMallocFailedHook(void)
{
    platform_raise_assertion(__FILE__, __LINE__);
}

void platform_get_heap_status(platform_heap_status_t *status)
{
    static ULONG min_ever_free = RTOS_HEAP_SIZE;
    ULONG available_bytes;
    tx_byte_pool_info_get(&pool, NULL, &available_bytes,
                    NULL, NULL, NULL, NULL);
    if (available_bytes < min_ever_free) min_ever_free = available_bytes;
    status->bytes_free = (int)available_bytes;
    status->bytes_minimum_ever_free = min_ever_free;
}

#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"
#include "tx_timer.h"

#ifdef TX_SAFETY_CRITICAL
TX_SAFETY_CRITICAL_EXCEPTION_HANDLER
#endif

void _systick_init(void);
void _setup_sys_handlers(void);

// _tx_initialize_kernel_enter is splitted into two parts:
// one in `os_impl_get_driver`
// the other is `enter`
static void port_tx_initialize_kernel(void)
{
    /* Determine if the compiler has pre-initialized ThreadX.  */
    if (_tx_thread_system_state != TX_INITIALIZE_ALMOST_DONE)
    {

        /* No, the initialization still needs to take place.  */

        /* Ensure that the system state variable is set to indicate
           initialization is in progress.  Note that this variable is
           later used to represent interrupt nesting.  */
        _tx_thread_system_state =  TX_INITIALIZE_IN_PROGRESS;

        /* Call any port specific preprocessing.  */
        TX_PORT_SPECIFIC_PRE_INITIALIZATION

        /* Invoke the low-level initialization to handle all processor specific
           initialization issues.  */
        _tx_initialize_low_level();
        
        _systick_init();
        _setup_sys_handlers();

        /* Invoke the high-level initialization to exercise all of the
           ThreadX components and the application's initialization
           function.  */
        _tx_initialize_high_level();

        /* Call any port specific post-processing.  */
        TX_PORT_SPECIFIC_POST_INITIALIZATION
    }

    /* Optional processing extension.  */
    TX_INITIALIZE_KERNEL_ENTER_EXTENSION

    /* Ensure that the system state variable is set to indicate
       initialization is in progress.  Note that this variable is
       later used to represent interrupt nesting.  */
    _tx_thread_system_state =  TX_INITIALIZE_IN_PROGRESS;
}

static void port_tx_start(void)
{
#ifdef POWER_SAVING
    void create_idle_task(void);
    create_idle_task();
#endif

    /* Set the system state in preparation for entering the thread
       scheduler.  */
    _tx_thread_system_state =  TX_INITIALIZE_IS_FINISHED;

    /* Call any port specific pre-scheduler processing.  */
    TX_PORT_SPECIFIC_PRE_SCHEDULER_INITIALIZATION

    /* Enter the scheduling loop to start executing threads!  */
    _tx_thread_schedule();

#ifdef TX_SAFETY_CRITICAL

    /* If we ever get here, raise safety critical exception.  */
    TX_SAFETY_CRITICAL_EXCEPTION(__FILE__, __LINE__, 0);
#endif
}

#define _SYSTICK_PRI    (*(uint8_t  *)(0xE000ED23UL))

#define NVIC_ADDR       0xe000e000

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_CLK_BIT	        ( 0UL << 2UL )
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )
#define portSY_FULL_READ_WRITE              15

#define RTC_CYCLES_PER_TICK                 (RTC_CLK_FREQ / TICK_PER_SECOND)
#define MAXIMUM_SUPPRESSED_TICKS            (0xffffff / RTC_CYCLES_PER_TICK)
#define MISSED_COUNTS_FACTOR                45
#define STOPPED_TIMER_COMPENSATION          (MISSED_COUNTS_FACTOR / ( OSC_CLK_FREQ / RTC_CLK_FREQ ))
#define EXPECTED_IDLE_TIME_BEFORE_SLEEP     2

void _setup_sys_handlers(void)
{
    // Setup System Handlers 4-7 Priority Registers
    io_write(NVIC_ADDR + 0xD18, 0x00000000);

    // SVCl, Rsrv, Rsrv, Rsrv
    // Setup System Handlers 8-11 Priority Registers
    // Note: SVC must be lowest priority, which is 0xFF
    io_write(NVIC_ADDR + 0xD1C, 0xFF000000);
    
    // SysT, PnSV, Rsrv, DbgM
    // Setup System Handlers 12-15 Priority Registers
    // Note: PnSV must be lowest priority, which is 0xFF
    io_write(NVIC_ADDR + 0xD20, 0x40FF0000);
}

void _systick_init(void)
{
    portNVIC_SYSTICK_LOAD_REG = RTC_CYCLES_PER_TICK - 1;    
    portNVIC_SYSTICK_CURRENT_VALUE_REG  = 0;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT | portNVIC_SYSTICK_CLK_BIT;
}

#ifdef POWER_SAVING

struct    
{
    uint8_t enhanced_ticks;
} pm_info = {0};

// This is a re-implementation of FreeRTOS's suppress ticks and sleep function.
static uint32_t _suppress_ticks_and_sleep(uint32_t expected_ticks)
{
    TX_INTERRUPT_SAVE_AREA
    uint32_t ulCompleteTickPeriods;
    
    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

    //platform_printf("%d\n", expected_ticks);
    
    uint32_t ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + (RTC_CYCLES_PER_TICK * (expected_ticks - 1UL));
    if( ulReloadValue > STOPPED_TIMER_COMPENSATION )
    {
        ulReloadValue -= STOPPED_TIMER_COMPENSATION;
    }

    TX_DISABLE

    portNVIC_SYSTICK_LOAD_REG = ulReloadValue;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
    
    if (pm_info.enhanced_ticks)
        while (0 == portNVIC_SYSTICK_CURRENT_VALUE_REG) ;

    platform_pre_sleep_processing();
    platform_post_sleep_processing();

    _setup_sys_handlers();
    
    TX_RESTORE

    TX_DISABLE

    if (pm_info.enhanced_ticks)
        while (portNVIC_SYSTICK_CURRENT_VALUE_REG == portNVIC_SYSTICK_CURRENT_VALUE_REG);

    portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );

    if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
    {
        uint32_t ulCalculatedLoadValue;

        ulCalculatedLoadValue = (RTC_CYCLES_PER_TICK - 1UL) - (ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG);

        if ((ulCalculatedLoadValue < STOPPED_TIMER_COMPENSATION ) || ( ulCalculatedLoadValue > RTC_CYCLES_PER_TICK))
        {
            ulCalculatedLoadValue = (RTC_CYCLES_PER_TICK - 1UL);
        }

        portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

        ulCompleteTickPeriods = expected_ticks - 1UL;
    }
    else
    {
        uint32_t ulCompletedSysTickDecrements = (expected_ticks * RTC_CYCLES_PER_TICK) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

        ulCompleteTickPeriods = ulCompletedSysTickDecrements / RTC_CYCLES_PER_TICK;

        portNVIC_SYSTICK_LOAD_REG = ((ulCompleteTickPeriods + 1UL) * RTC_CYCLES_PER_TICK) - ulCompletedSysTickDecrements;
    }

    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

    if (pm_info.enhanced_ticks)
        while (0 == portNVIC_SYSTICK_CURRENT_VALUE_REG) ;
    portNVIC_SYSTICK_LOAD_REG = RTC_CYCLES_PER_TICK - 1UL;

    TX_RESTORE
    return ulCompleteTickPeriods;
}

void rtos_enable_enhanced_ticks(void)
{
    platform_config(PLATFORM_CFG_RTOS_ENH_TICK, 1);
    pm_info.enhanced_ticks = 1;
}

uint32_t rtos_sleep_process(uint32_t timeout_tick)
{

    TX_INTERRUPT_SAVE_AREA

    if (timeout_tick > MAXIMUM_SUPPRESSED_TICKS)
        timeout_tick = MAXIMUM_SUPPRESSED_TICKS;

    timeout_tick = platform_pre_suppress_ticks_and_sleep_processing(timeout_tick);
    if (timeout_tick < EXPECTED_IDLE_TIME_BEFORE_SLEEP)
    {
        __WFI();
        return 0;
    }

    TX_DISABLE
    
    uint32_t delta_ticks = _suppress_ticks_and_sleep(timeout_tick);

    TX_RESTORE

    return delta_ticks;
}

void thread_idle_entry(ULONG thread_input)
{   
    TX_INTERRUPT_SAVE_AREA
    ULONG   tx_low_power_next_expiration;
    uint32_t   timers_active;  

    for (;;)
    {
        TX_DISABLE

        timers_active =  tx_timer_get_next(&tx_low_power_next_expiration);

        if (timers_active == TX_FALSE)
            tx_low_power_next_expiration = 0xFFFFFFFF;

        TX_RESTORE;

        tx_low_power_next_expiration = platform_pre_suppress_ticks_and_sleep_processing(tx_low_power_next_expiration);

        if (tx_low_power_next_expiration > EXPECTED_IDLE_TIME_BEFORE_SLEEP)
        {
            uint32_t tx_low_power_adjust_ticks = rtos_sleep_process(tx_low_power_next_expiration);           
            if (tx_low_power_adjust_ticks > 0)
            {
                tx_time_increment(tx_low_power_adjust_ticks);
                tx_time_set(tx_time_get() + tx_low_power_adjust_ticks);
            }            
            platform_os_idle_resumed_hook();
        }
        else
        {
            __WFI();
        }
    }
}

#define IDLE_STACK_SIZE         300

static TX_THREAD   thread_idle;
static uint8_t idle_stack[IDLE_STACK_SIZE];
void   thread_idle_entry(ULONG thread_input);

void create_idle_task(void)
{
    tx_thread_create(&thread_idle, "idle", thread_idle_entry, 0,  
            idle_stack, IDLE_STACK_SIZE,
            TX_MAX_PRIORITIES - 1, TX_MAX_PRIORITIES - 1, TX_NO_TIME_SLICE, TX_AUTO_START);
}

#endif
