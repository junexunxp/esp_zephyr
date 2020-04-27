// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <stddef.h>
#include <zephyr.h>
#include <arch/cpu.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <kernel.h>
#include <random/rand32.h>



#if CONFIG_ESP32_VHCI_CONTROLLER
#define IRAM_ATTR
/* Macro definition
 ************************************************************************
 */

#define BTDM_LOG_TAG                        "BTDM_INIT"

#define BTDM_INIT_PERIOD                    (5000)    /* ms */

/* Bluetooth system and controller config */
#define BTDM_CFG_BT_DATA_RELEASE            (1<<0)
#define BTDM_CFG_HCI_UART                   (1<<1)
#define BTDM_CFG_CONTROLLER_RUN_APP_CPU     (1<<2)
#define BTDM_CFG_SCAN_DUPLICATE_OPTIONS     (1<<3)
#define BTDM_CFG_SEND_ADV_RESERVED_SIZE     (1<<4)
#define BTDM_CFG_BLE_FULL_SCAN_SUPPORTED    (1<<5)

/* Sleep mode */
#define BTDM_MODEM_SLEEP_MODE_NONE          (0)
#define BTDM_MODEM_SLEEP_MODE_ORIG          (1)
#define BTDM_MODEM_SLEEP_MODE_EVED          (2)  // sleep mode for BLE controller, used only for internal test.

/* Low Power Clock Selection */
#define BTDM_LPCLK_SEL_XTAL      (0)
#define BTDM_LPCLK_SEL_XTAL32K   (1)
#define BTDM_LPCLK_SEL_RTC_SLOW  (2)
#define BTDM_LPCLK_SEL_8M        (3)

/* Sleep and wakeup interval control */
#define BTDM_MIN_SLEEP_DURATION          (12) // threshold of interval in slots to allow to fall into modem sleep
#define BTDM_MODEM_WAKE_UP_DELAY         (4)  // delay in slots of modem wake up procedure, including re-enable PHY/RF
#define ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL  0x20200106


#define OSI_FUNCS_TIME_BLOCKING  0xffffffff
#define OSI_VERSION              0x00010002
#define OSI_MAGIC_VALUE          0xFADEBEAD



typedef struct{
	void *reserved; /*first word reserved use by fifo*/
	void *ptr;
}bt_kfifo_s;

struct k_fifo mutex_fifo, sem_fifo, queue_fifo;


/* Types definition
 ************************************************************************
 */

/* VHCI function interface */
typedef struct vhci_host_callback {
    void (*notify_host_send_available)(void);               /*!< callback used to notify that the host can send packet to controller */
    int (*notify_host_recv)(uint8_t *data, uint16_t len);   /*!< callback used to notify that the controller has a packet to send to the host*/
} vhci_host_callback_t;


/* OSI function */
struct osi_funcs_t {
    uint32_t _version;
    xt_handler (*_set_isr)(int n, xt_handler f, void *arg);
    void (*_ints_on)(unsigned int mask);
    void (*_interrupt_disable)(void);
    void (*_interrupt_restore)(void);
    void (*_task_yield)(void);
    void (*_task_yield_from_isr)(void);
    void *(*_semphr_create)(uint32_t max, uint32_t init);
    void (*_semphr_delete)(void *semphr);
    int32_t (*_semphr_take_from_isr)(void *semphr, void *hptw);
    int32_t (*_semphr_give_from_isr)(void *semphr, void *hptw);
    int32_t (*_semphr_take)(void *semphr, uint32_t block_time_ms);
    int32_t (*_semphr_give)(void *semphr);
    void *(*_mutex_create)(void);
    void (*_mutex_delete)(void *mutex);
    int32_t (*_mutex_lock)(void *mutex);
    int32_t (*_mutex_unlock)(void *mutex);
    void *(* _queue_create)(uint32_t queue_len, uint32_t item_size);
    void (* _queue_delete)(void *queue);
    int32_t (* _queue_send)(void *queue, void *item, uint32_t block_time_ms);
    int32_t (* _queue_send_from_isr)(void *queue, void *item, void *hptw);
    int32_t (* _queue_recv)(void *queue, void *item, uint32_t block_time_ms);
    int32_t (* _queue_recv_from_isr)(void *queue, void *item, void *hptw);
    int32_t (* _task_create)(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id);
    void (* _task_delete)(void *task_handle);
    bool (* _is_in_isr)(void);
    int (* _cause_sw_intr_to_core)(int core_id, int intr_no);
    void *(* _malloc)(uint32_t size);
    void *(* _malloc_internal)(uint32_t size);
    void (* _free)(void *p);
    int32_t (* _read_efuse_mac)(uint8_t mac[6]);
    void (* _srand)(unsigned int seed);
    int (* _rand)(void);
    uint32_t (* _btdm_lpcycles_2_us)(uint32_t cycles);
    uint32_t (* _btdm_us_2_lpcycles)(uint32_t us);
    bool (* _btdm_sleep_check_duration)(uint32_t *slot_cnt);
    void (* _btdm_sleep_enter_phase1)(uint32_t lpcycles);  /* called when interrupt is disabled */
    void (* _btdm_sleep_enter_phase2)(void);
    void (* _btdm_sleep_exit_phase1)(void);  /* called from ISR */
    void (* _btdm_sleep_exit_phase2)(void);  /* called from ISR */
    void (* _btdm_sleep_exit_phase3)(void);  /* called from task */
    bool (* _coex_bt_wakeup_request)(void);
    void (* _coex_bt_wakeup_request_end)(void);
    int (* _coex_bt_request)(uint32_t event, uint32_t latency, uint32_t duration);
    int (* _coex_bt_release)(uint32_t event);
    int (* _coex_register_bt_cb)(coex_func_cb_t cb);
    uint32_t (* _coex_bb_reset_lock)(void);
    void (* _coex_bb_reset_unlock)(uint32_t restore);
    uint32_t _magic;
};



/**
 * @brief Controller config options, depend on config mask.
 *        Config mask indicate which functions enabled, this means
 *        some options or parameters of some functions enabled by config mask.
 */
typedef struct {
    /*
     * Following parameters can be configured runtime, when call esp_bt_controller_init()
     */
    uint16_t controller_task_stack_size;    /*!< Bluetooth controller task stack size */
    uint8_t controller_task_prio;           /*!< Bluetooth controller task priority */
    uint8_t hci_uart_no;                    /*!< If use UART1/2 as HCI IO interface, indicate UART number */
    uint32_t hci_uart_baudrate;             /*!< If use UART1/2 as HCI IO interface, indicate UART baudrate */
    uint8_t scan_duplicate_mode;            /*!< scan duplicate mode */
    uint8_t scan_duplicate_type;            /*!< scan duplicate type */
    uint16_t normal_adv_size;               /*!< Normal adv size for scan duplicate */
    uint16_t mesh_adv_size;                 /*!< Mesh adv size for scan duplicate */
    uint16_t send_adv_reserved_size;        /*!< Controller minimum memory value */
    uint32_t  controller_debug_flag;        /*!< Controller debug log flag */
    uint8_t mode;                           /*!< Controller mode: BR/EDR, BLE or Dual Mode */
    uint8_t ble_max_conn;                   /*!< BLE maximum connection numbers */
    uint8_t bt_max_acl_conn;                /*!< BR/EDR maximum ACL connection numbers */
    uint8_t bt_sco_datapath;                /*!< SCO data path, i.e. HCI or PCM module */
    bool auto_latency;                      /*!< BLE auto latency, used to enhance classic BT performance */
    /*
     * Following parameters can not be configured runtime when call esp_bt_controller_init()
     * It will be overwrite with a constant value which in menuconfig or from a macro.
     * So, do not modify the value when esp_bt_controller_init()
     */
    uint8_t bt_max_sync_conn;               /*!< BR/EDR maximum ACL connection numbers. Effective in menuconfig */
    uint8_t ble_sca;                        /*!< BLE low power crystal accuracy index */
    uint32_t magic;                         /*!< Magic number */
} esp_bt_controller_config_t;

#define BT_CONTROLLER_INIT_CONFIG_DEFAULT()   	{							   \
		.controller_task_stack_size = ESP_TASK_BT_CONTROLLER_STACK, 		   \
		.controller_task_prio = ESP_TASK_BT_CONTROLLER_PRIO,				   \
		.hci_uart_no = BT_HCI_UART_NO_DEFAULT,								   \
		.hci_uart_baudrate = BT_HCI_UART_BAUDRATE_DEFAULT,					   \
		.scan_duplicate_mode = SCAN_DUPLICATE_MODE, 						   \
		.scan_duplicate_type = SCAN_DUPLICATE_TYPE_VALUE,					   \
		.normal_adv_size = NORMAL_SCAN_DUPLICATE_CACHE_SIZE,				   \
		.mesh_adv_size = MESH_DUPLICATE_SCAN_CACHE_SIZE,					   \
		.send_adv_reserved_size = SCAN_SEND_ADV_RESERVED_SIZE,				   \
		.controller_debug_flag = CONTROLLER_ADV_LOST_DEBUG_BIT, 			   \
		.mode = BTDM_CONTROLLER_MODE_EFF,									   \
		.ble_max_conn = CONFIG_BTDM_CTRL_BLE_MAX_CONN_EFF,					   \
		.bt_max_acl_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN_EFF,		   \
		.bt_sco_datapath = CONFIG_BTDM_CTRL_BR_EDR_SCO_DATA_PATH_EFF,		   \
		.auto_latency = BTDM_CTRL_AUTO_LATENCY_EFF, 						   \
		.bt_max_sync_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF,		   \
		.ble_sca = CONFIG_BTDM_BLE_SLEEP_CLOCK_ACCURACY_INDEX_EFF,			   \
		.magic = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,						   \
	};

/* External functions or values
 ************************************************************************
 */

/* not for user call, so don't put to include file */
/* OSI */
extern int btdm_osi_funcs_register(void *osi_funcs);
/* Initialise and De-initialise */
extern int btdm_controller_init(uint32_t config_mask, esp_bt_controller_config_t *config_opts);
extern void btdm_controller_deinit(void);
extern int btdm_controller_enable(esp_bt_mode_t mode);
extern void btdm_controller_disable(void);
extern uint8_t btdm_controller_get_mode(void);
extern const char *btdm_controller_get_compile_version(void);
extern void btdm_rf_bb_init_phase2(void); // shall be called after PHY/RF is enabled
/* Sleep */
extern void btdm_controller_enable_sleep(bool enable);
extern void btdm_controller_set_sleep_mode(uint8_t mode);
extern uint8_t btdm_controller_get_sleep_mode(void);
extern bool btdm_power_state_active(void);
extern void btdm_wakeup_request(bool request_lock);
extern void btdm_wakeup_request_end(void);
/* Low Power Clock */
extern bool btdm_lpclk_select_src(uint32_t sel);
extern bool btdm_lpclk_set_div(uint32_t div);
/* VHCI */
extern bool API_vhci_host_check_send_available(void);
extern void API_vhci_host_send_packet(uint8_t *data, uint16_t len);
extern int API_vhci_host_register_callback(const vhci_host_callback_t *callback);
/* TX power */
extern int ble_txpwr_set(int power_type, int power_level);
extern int ble_txpwr_get(int power_type);
extern int bredr_txpwr_set(int min_power_level, int max_power_level);
extern int bredr_txpwr_get(int *min_power_level, int *max_power_level);
extern void bredr_sco_datapath_set(uint8_t data_path);
extern void btdm_controller_scan_duplicate_list_clear(void);
/* Coexistence */
extern int coex_bt_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration);
extern int coex_bt_release_wrapper(uint32_t event);
extern int coex_register_bt_cb_wrapper(coex_func_cb_t cb);
extern uint32_t coex_bb_reset_lock_wrapper(void);
extern void coex_bb_reset_unlock_wrapper(uint32_t restore);
extern void coex_ble_adv_priority_high_set(bool high);

extern char _bss_start_btdm;
extern char _bss_end_btdm;
extern char _data_start_btdm;
extern char _data_end_btdm;
extern uint32_t _data_start_btdm_rom;
extern uint32_t _data_end_btdm_rom;

extern uint32_t _bt_bss_start;
extern uint32_t _bt_bss_end;
extern uint32_t _nimble_bss_start;
extern uint32_t _nimble_bss_end;
extern uint32_t _btdm_bss_start;
extern uint32_t _btdm_bss_end;
extern uint32_t _bt_data_start;
extern uint32_t _bt_data_end;
extern uint32_t _nimble_data_start;
extern uint32_t _nimble_data_end;
extern uint32_t _btdm_data_start;
extern uint32_t _btdm_data_end;

/* Local Function Declare
 *********************************************************************
 */
#if CONFIG_SPIRAM_USE_MALLOC
static bool btdm_queue_generic_register(const btdm_queue_item_t *queue);
static bool btdm_queue_generic_deregister(btdm_queue_item_t *queue);
#endif /* CONFIG_SPIRAM_USE_MALLOC */
static void IRAM_ATTR interrupt_disable(void);
static void IRAM_ATTR interrupt_restore(void);
static void IRAM_ATTR task_yield_from_isr(void);
static void *semphr_create_wrapper(uint32_t max, uint32_t init);
static void semphr_delete_wrapper(void *semphr);
static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw);
static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw);
static int32_t  semphr_take_wrapper(void *semphr, uint32_t block_time_ms);
static int32_t  semphr_give_wrapper(void *semphr);
static void *mutex_create_wrapper(void);
static void mutex_delete_wrapper(void *mutex);
static int32_t mutex_lock_wrapper(void *mutex);
static int32_t mutex_unlock_wrapper(void *mutex);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static void queue_delete_wrapper(void *queue);
static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_ms);
static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw);
static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_ms);
static int32_t IRAM_ATTR queue_recv_from_isr_wrapper(void *queue, void *item, void *hptw);
static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id);
static void task_delete_wrapper(void *task_handle);
static bool IRAM_ATTR is_in_isr_wrapper(void);
static void IRAM_ATTR cause_sw_intr(void *arg);
static int IRAM_ATTR cause_sw_intr_to_core_wrapper(int core_id, int intr_no);
static void *malloc_internal_wrapper(size_t size);
static int32_t IRAM_ATTR read_mac_wrapper(uint8_t mac[6]);
static void IRAM_ATTR srand_wrapper(unsigned int seed);
static int IRAM_ATTR rand_wrapper(void);
static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles);
static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us);
static bool IRAM_ATTR btdm_sleep_check_duration(uint32_t *slot_cnt);
static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles);
static void btdm_sleep_enter_phase2_wrapper(void);
static void IRAM_ATTR btdm_sleep_exit_phase1_wrapper(void);
static void btdm_sleep_exit_phase3_wrapper(void);
static bool coex_bt_wakeup_request(void);
static void coex_bt_wakeup_request_end(void);

/* Local variable definition
 ***************************************************************************
 */
/* OSI funcs */
static const struct osi_funcs_t osi_funcs_ro = {
    ._version = OSI_VERSION,
    ._set_isr = xt_set_interrupt_handler,
    ._ints_on = xt_ints_on,
    ._interrupt_disable = interrupt_disable,
    ._interrupt_restore = interrupt_restore,
    ._task_yield = vPortYield,
    ._task_yield_from_isr = task_yield_from_isr,
    ._semphr_create = semphr_create_wrapper,
    ._semphr_delete = semphr_delete_wrapper,
    ._semphr_take_from_isr = semphr_take_from_isr_wrapper,
    ._semphr_give_from_isr = semphr_give_from_isr_wrapper,
    ._semphr_take = semphr_take_wrapper,
    ._semphr_give = semphr_give_wrapper,
    ._mutex_create = mutex_create_wrapper,
    ._mutex_delete = mutex_delete_wrapper,
    ._mutex_lock = mutex_lock_wrapper,
    ._mutex_unlock = mutex_unlock_wrapper,
    ._queue_create = queue_create_wrapper,
    ._queue_delete = queue_delete_wrapper,
    ._queue_send = queue_send_wrapper,
    ._queue_send_from_isr = queue_send_from_isr_wrapper,
    ._queue_recv = queue_recv_wrapper,
    ._queue_recv_from_isr = queue_recv_from_isr_wrapper,
    ._task_create = task_create_wrapper,
    ._task_delete = task_delete_wrapper,
    ._is_in_isr = is_in_isr_wrapper,
    ._cause_sw_intr_to_core = cause_sw_intr_to_core_wrapper,
    ._malloc = malloc,
    ._malloc_internal = malloc_internal_wrapper,
    ._free = free,
    ._read_efuse_mac = read_mac_wrapper,
    ._srand = srand_wrapper,
    ._rand = rand_wrapper,
    ._btdm_lpcycles_2_us = btdm_lpcycles_2_us,
    ._btdm_us_2_lpcycles = btdm_us_2_lpcycles,
    ._btdm_sleep_check_duration = btdm_sleep_check_duration,
    ._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
    ._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
    ._btdm_sleep_exit_phase1 = btdm_sleep_exit_phase1_wrapper,
    ._btdm_sleep_exit_phase2 = NULL,
    ._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
    ._coex_bt_wakeup_request = coex_bt_wakeup_request,
    ._coex_bt_wakeup_request_end = coex_bt_wakeup_request_end,
    ._coex_bt_request = coex_bt_request_wrapper,
    ._coex_bt_release = coex_bt_release_wrapper,
    ._coex_register_bt_cb = coex_register_bt_cb_wrapper,
    ._coex_bb_reset_lock = coex_bb_reset_lock_wrapper,
    ._coex_bb_reset_unlock = coex_bb_reset_unlock_wrapper,
    ._magic = OSI_MAGIC_VALUE,
};
/**
 * @brief Bluetooth mode for controller enable/disable
 */
typedef enum {
	ESP_BT_MODE_IDLE	   = 0x00,	 /*!< Bluetooth is not running */
	ESP_BT_MODE_BLE 	   = 0x01,	 /*!< Run BLE mode */
	ESP_BT_MODE_CLASSIC_BT = 0x02,	 /*!< Run Classic BT mode */
	ESP_BT_MODE_BTDM	   = 0x03,	 /*!< Run dual mode */
} esp_bt_mode_t;
#define SOC_MEM_BT_DATA_START               0x3ffae6e0
#define SOC_MEM_BT_DATA_END                 0x3ffaff10
#define SOC_MEM_BT_EM_START                 0x3ffb0000
#define SOC_MEM_BT_EM_END                   0x3ffb7cd8
#define SOC_MEM_BT_EM_BTDM0_START           0x3ffb0000
#define SOC_MEM_BT_EM_BTDM0_END             0x3ffb09a8
#define SOC_MEM_BT_EM_BLE_START             0x3ffb09a8
#define SOC_MEM_BT_EM_BLE_END               0x3ffb1ddc
#define SOC_MEM_BT_EM_BTDM1_START           0x3ffb1ddc
#define SOC_MEM_BT_EM_BTDM1_END             0x3ffb2730
#define SOC_MEM_BT_EM_BREDR_START           0x3ffb2730
#define SOC_MEM_BT_EM_BREDR_NO_SYNC_END     0x3ffb6388  //Not calculate with synchronize connection support
#define SOC_MEM_BT_EM_BREDR_END             0x3ffb7cd8  //Calculate with synchronize connection support
#define SOC_MEM_BT_EM_SYNC0_START           0x3ffb6388
#define SOC_MEM_BT_EM_SYNC0_END             0x3ffb6bf8
#define SOC_MEM_BT_EM_SYNC1_START           0x3ffb6bf8
#define SOC_MEM_BT_EM_SYNC1_END             0x3ffb7468
#define SOC_MEM_BT_EM_SYNC2_START           0x3ffb7468
#define SOC_MEM_BT_EM_SYNC2_END             0x3ffb7cd8
#define SOC_MEM_BT_BSS_START                0x3ffb8000
#define SOC_MEM_BT_BSS_END                  0x3ffb9a20
#define SOC_MEM_BT_MISC_START               0x3ffbdb28
#define SOC_MEM_BT_MISC_END                 0x3ffbdb5c
#define SOC_MEM_BT_EM_PER_SYNC_SIZE         0x870

#define SOC_MEM_BT_EM_BREDR_REAL_END        (SOC_MEM_BT_EM_BREDR_NO_SYNC_END + 2 * SOC_MEM_BT_EM_PER_SYNC_SIZE)

/* the mode column will be modified by release function to indicate the available region */
static btdm_dram_available_region_t btdm_dram_available_region[] = {
    //following is .data
    {ESP_BT_MODE_BTDM,          SOC_MEM_BT_DATA_START,      SOC_MEM_BT_DATA_END         },
    //following is memory which HW will use
    {ESP_BT_MODE_BTDM,          SOC_MEM_BT_EM_BTDM0_START,  SOC_MEM_BT_EM_BTDM0_END     },
    {ESP_BT_MODE_BLE,           SOC_MEM_BT_EM_BLE_START,    SOC_MEM_BT_EM_BLE_END      },
    {ESP_BT_MODE_BTDM,          SOC_MEM_BT_EM_BTDM1_START,  SOC_MEM_BT_EM_BTDM1_END     },
    {ESP_BT_MODE_CLASSIC_BT,    SOC_MEM_BT_EM_BREDR_START,  SOC_MEM_BT_EM_BREDR_REAL_END},
    //following is .bss
    {ESP_BT_MODE_BTDM,          SOC_MEM_BT_BSS_START,       SOC_MEM_BT_BSS_END          },
    {ESP_BT_MODE_BTDM,          SOC_MEM_BT_MISC_START,      SOC_MEM_BT_MISC_END         },
};

/* Reserve the full memory region used by Bluetooth Controller,
 *    some may be released later at runtime. */
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_EM_START,   SOC_MEM_BT_EM_BREDR_REAL_END,  rom_bt_em);
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_BSS_START,  SOC_MEM_BT_BSS_END,            rom_bt_bss);
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_MISC_START, SOC_MEM_BT_MISC_END,           rom_bt_misc);
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_DATA_START, SOC_MEM_BT_DATA_END,           rom_bt_data);

static DRAM_ATTR struct osi_funcs_t *osi_funcs_p;



// measured average low power clock period in micro seconds
static DRAM_ATTR uint32_t btdm_lpcycle_us = 0;
static DRAM_ATTR uint8_t btdm_lpcycle_us_frac = 0; // number of fractional bit for btdm_lpcycle_us




static inline void btdm_check_and_init_bb(void)
{

      btdm_rf_bb_init_phase2();

}


static u32_t global_interrupt_mux;
static void IRAM_ATTR interrupt_disable(void)
{
    global_interrupt_mux = irq_lock();
}

static void IRAM_ATTR interrupt_restore(void)
{
    irq_unlock(global_interrupt_mux);
}

static void IRAM_ATTR task_yield_from_isr(void)
{
    portYIELD_FROM_ISR();
}

static void *semphr_create_wrapper(uint32_t max, uint32_t init)
{
	BT_DBG("call semphr create\n");
	bt_kfifo_s *sfifo = malloc(sizeof(bt_kfifo_s));
	if(!sfifo){
		BT_ERR("Not enough space to create fifo data\n");
		return NULL;
	}
	
	struct k_sem *new_sem = malloc_internal_wrapper(sizeof(struct k_sem));
	if(!new_sem){
		BT_ERR("Not enough space to create new sem\n");
		free(sfifo);
		return NULL;
	}
	int ret = k_sem_init(new_sem, init, max);
	if(ret){
		free(new_sem);
		free(sfifo);
		BT_ERR("sem init error, code 0x%x\n",ret);
		return NULL;
	}
	sfifo->ptr = new_sem;
	k_fifo_put(&sem_fifo, sfifo);
	return new_sem;
}

static void semphr_delete_wrapper(void *semphr)
{
	if(!semphr){
		BT_ERR("Semphr delete wrapper invalid param\n");
		return;
	}
	if(k_fifo_is_empty(&sem_fifo)){
		BT_ERR("no itmes found in sem_fifo\n");
		free(semphr);
		return;
	}
	
	bt_kfifo_s *sfifo = k_fifo_get(&sem_fifo,K_NO_WAIT);
	bt_kfifo_s *hfifo = sfifo;
	
	do{
		if(sfifo->ptr == semphr){
			break;
		}
		k_fifo_put(&sem_fifo, sfifo);
		sfifo = k_fifo_get(&sem_fifo,K_NO_WAIT);
		if(sfifo == hfifo){
			sfifo = NULL;
			BT_WARN("searched all sem_fifo items, but no matching found\n");
			break;
		}
	}while(sfifo);
	
	if(sfifo){
		free(sfifo);
	}
	free(semphr);
}

static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
	ARG_UNUSED(hptw);
    return (int32_t)k_sem_take(semphr, K_NO_WAIT);
}

static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
	ARG_UNUSED(hptw);
    k_sem_give(semphr);
	return 0;
}

static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_ms)
{
	if (block_time_ms == OSI_FUNCS_TIME_BLOCKING) {
		return (int32_t)k_sem_take(semphr, K_FOREVER);
	}else{
    	return (int32_t)k_sem_take(semphr, K_MSEC(block_time_ms));
	}
}

static int32_t semphr_give_wrapper(void *semphr)
{
    k_sem_give(semphr);
	return 0;
}

static void *mutex_create_wrapper(void)
{
	bt_kfifo_s *mfifo = malloc(sizeof(bt_kfifo_s));
	if(!mfifo){
		BT_ERR("Not enough space to create fifo data\n");
		return NULL;
	}
	struct k_mutex *new_mutex = malloc(sizeof(struct k_mutex));
	if(!new_mutex){
		free(mfifo);
		BT_ERR("Not enough space to create new mutex\n");
		return NULL;
	}
	int ret = k_mutex_init(new_mutex);
	if(ret){
		free(new_mutex);
		free(mfifo);
		BT_ERR("mutex init error, code 0x%x\n",ret);
		return NULL;
	}
	mfifo->ptr = new_mutex;
	k_fifo_put(&mutex_fifo, mfifo);
	return new_mutex;
}

static void mutex_delete_wrapper(void *mutex)
{
	if(!mutex){
		BT_ERR("mutex delete wrapper invalid param\n");
		return;
	}
	
	if(k_fifo_is_empty(&mutex_fifo)){
		BT_ERR("no itmes found in mutex_fifo\n");
		free(mutex);
		return;
	}

	bt_kfifo_s *sfifo = k_fifo_get(&mutex_fifo,K_NO_WAIT);
	bt_kfifo_s *hfifo = sfifo;
	
	do{
		if(sfifo->ptr == mutex){
			break;
		}
		k_fifo_put(&mutex_fifo, sfifo);
		sfifo = k_fifo_get(&mutex_fifo,K_NO_WAIT);
		if(sfifo == hfifo){
			sfifo = NULL;
			BT_WARN("searched all mutex_fifo items, but no matching found\n");
			return;
		}
	}while(sfifo);
	
	if(sfifo){
		free(sfifo);
	}
	free(mutex);
}

static int32_t mutex_lock_wrapper(void *mutex)
{
    return (int32_t)k_mutex_lock((struct k_mutex *)mutex, K_FOREVER);
}

static int32_t mutex_unlock_wrapper(void *mutex)
{
    return (int32_t)k_mutex_unlock((struct k_mutex *)mutex);
}

static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{

	bt_kfifo_s *qfifo = malloc(sizeof(bt_kfifo_s));
	if(!qfifo){
		BT_ERR("Not enough space to create queue fifo data\n");
		return NULL;
	}
	struct k_msgq *nq = malloc(struct k_msgq);
	if(!nq){
		BT_ERR("Not enough space to create k msg queue\n");
		free(qfifo);
		return NULL;
	}
	int ret = k_msgq_alloc_init(nq,item_size, queue_len);
	if(ret){
		free(qfifo);
		free(nq);
		BT_ERR("k msgq alloc init error code %d\n",ret);
		return NULL;
	}
	qfifo->ptr = nq;
	k_fifo_put(&queue_fifo, qfifo);
	return nq;
}

static void queue_delete_wrapper(void *queue)
{
	if(!queue){
	    BT_ERR("queue delete error\n");
		return;
	}
	
	if(k_fifo_is_empty(&queue_fifo)){
		BT_ERR("no itmes found in queue_fifo\n");
		return;
	}

	bt_kfifo_s *sfifo = k_fifo_get(&queue_fifo,K_NO_WAIT);
	bt_kfifo_s *hfifo = sfifo;
	
	do{
		if(sfifo->ptr == queue){
			break;
		}
		k_fifo_put(&queue_fifo, sfifo);
		sfifo = k_fifo_get(&queue_fifo,K_NO_WAIT);
		if(sfifo == hfifo){
			sfifo = NULL;
			BT_WARN("searched all mutex_fifo items, but no matching found\n");
			return;
		}
	}while(sfifo);
	
	if(sfifo){
		free(sfifo);
	}
	k_msgq_cleanup(queue);
}

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_ms)
{
	if (block_time_ms == OSI_FUNCS_TIME_BLOCKING) {
		return (int32_t)k_msgq_put(queue, item,K_FOREVER);
	}else{
		return (int32_t)k_msgq_put(queue, item, K_MSEC(block_time_ms));
	}
}

static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
    return (int32_t)k_msgq_put(queue, item,K_NO_WAIT);
}

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_ms)
{
    if (block_time_ms == OSI_FUNCS_TIME_BLOCKING) {
        return (int32_t)k_msgq_get(queue, item, K_FOREVER);
    } else {
        return (int32_t)k_msgq_get(queue, item, K_MSEC(block_time_ms));
    }
}

static int32_t IRAM_ATTR queue_recv_from_isr_wrapper(void *queue, void *item, void *hptw)
{
   return (int32_t)k_msgq_get(queue, item, K_NO_WAIT);
}

static struct k_thread  controller_thread;
static K_THREAD_STACK_DEFINE(controller_thread_stack, CONFIG_BT_RX_STACK_SIZE);

static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{

	*task_handle = k_thread_create(&controller_thread,controller_thread_stack, K_THREAD_STACK_SIZEOF(controller_thread_stack), 
		task_func, NULL, NULL, NULL,K_PRIO_COOP(CONFIG_BT_RX_PRIO),0, K_NO_WAIT);
	
	k_thread_name_set(&controller_thread, name);
	return 0;
}

static void task_delete_wrapper(void *task_handle)
{
    k_thread_abort((k_tid_t )task_handle);
}

static bool IRAM_ATTR is_in_isr_wrapper(void)
{
    return k_is_in_isr();
}

static void IRAM_ATTR cause_sw_intr(void *arg)
{
    /* just convert void * to int, because the width is the same */
    uint32_t intr_no = (uint32_t)arg;
	//Not use
}

static int IRAM_ATTR cause_sw_intr_to_core_wrapper(int core_id, int intr_no)
{
    return 0;
}

static void *malloc_internal_wrapper(size_t size)
{
    return malloc(size);
}
static const u8_t bt_addr[6] = {0x01,0x20,0x03,0x30,0x04,0x40};
static int32_t IRAM_ATTR read_mac_wrapper(uint8_t mac[6])
{
	if (mac == NULL) {
		BT_ERR("mac address param is NULL");
		return -1;
	}
	memcpy(mac,bt_addr,6);
	
}

static void IRAM_ATTR srand_wrapper(unsigned int seed)
{
    /* empty function */
}

static int IRAM_ATTR rand_wrapper(void)
{
    return (int)sys_rand32_get();
}

static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles)
{
    // The number of lp cycles should not lead to overflow. Thrs: 100s
    // clock measurement is conducted
    uint64_t us = (uint64_t)btdm_lpcycle_us * cycles;
    us = (us + (1 << (btdm_lpcycle_us_frac - 1))) >> btdm_lpcycle_us_frac;
    return (uint32_t)us;
}

/*
 * @brief Converts a duration in slots into a number of low power clock cycles.
 */
static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us)
{
    // The number of sleep duration(us) should not lead to overflow. Thrs: 100s
    // Compute the sleep duration in us to low power clock cycles, with calibration result applied
    // clock measurement is conducted
    uint64_t cycles = ((uint64_t)(us) << btdm_lpcycle_us_frac) / btdm_lpcycle_us;

    return (uint32_t)cycles;
}

static bool IRAM_ATTR btdm_sleep_check_duration(uint32_t *slot_cnt)
{
    if (*slot_cnt < BTDM_MIN_SLEEP_DURATION) {
        return false;
    }
    /* wake up in advance considering the delay in enabling PHY/RF */
    *slot_cnt -= BTDM_MODEM_WAKE_UP_DELAY;
    return true;
}

static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles)
{
#ifdef CONFIG_PM_ENABLE

#endif
}

static void btdm_sleep_enter_phase2_wrapper(void)
{

}

static void IRAM_ATTR btdm_sleep_exit_phase1_wrapper(void)
{

}

static void btdm_sleep_exit_phase3_wrapper(void)
{

}

#ifdef CONFIG_PM_ENABLE
static void IRAM_ATTR btdm_slp_tmr_callback(void *arg)
{

}
#endif

#define BTDM_ASYNC_WAKEUP_REQ_HCI   0
#define BTDM_ASYNC_WAKEUP_REQ_COEX   1
#define BTDM_ASYNC_WAKEUP_REQMAX    2

static bool async_wakeup_request(int event)
{
    bool request_lock = false;
    switch (event) {
        case BTDM_ASYNC_WAKEUP_REQ_HCI:
            request_lock = true;
            break;
        case BTDM_ASYNC_WAKEUP_REQ_COEX:
            request_lock = false;
            break;
        default:
            return false;
    }

    bool do_wakeup_request = false;

    if (!btdm_power_state_active()) {
#if CONFIG_PM_ENABLE

#endif
        do_wakeup_request = true;
        btdm_wakeup_request(request_lock);
    }

    return do_wakeup_request;
}

static void async_wakeup_request_end(int event)
{
    bool request_lock = false;
    switch (event) {
        case BTDM_ASYNC_WAKEUP_REQ_HCI:
            request_lock = true;
            break;
        case BTDM_ASYNC_WAKEUP_REQ_COEX:
            request_lock = false;
            break;
        default:
            return;
    }

    if (request_lock) {
        btdm_wakeup_request_end();
    }

    return;
}

static bool coex_bt_wakeup_request(void)
{
    return async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_COEX);
}

static void coex_bt_wakeup_request_end(void)
{
    async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_COEX);
    return;
}

bool esp_vhci_host_check_send_available(void)
{
    return API_vhci_host_check_send_available();
}

void esp_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
    bool do_wakeup_request = async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_HCI);

    API_vhci_host_send_packet(data, len);

    if (do_wakeup_request) {
        async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_HCI);
    }
}


static uint32_t btdm_config_mask_load(void)
{
    uint32_t mask = 0x0;

#if CONFIG_BTDM_CTRL_HCI_MODE_UART_H4
    mask |= BTDM_CFG_HCI_UART;
#endif
#if CONFIG_BTDM_CTRL_PINNED_TO_CORE == 1
    mask |= BTDM_CFG_CONTROLLER_RUN_APP_CPU;
#endif
#if CONFIG_BTDM_CTRL_FULL_SCAN_SUPPORTED
    mask |= BTDM_CFG_BLE_FULL_SCAN_SUPPORTED;
#endif /* CONFIG_BTDM_CTRL_FULL_SCAN_SUPPORTED */
    mask |= BTDM_CFG_SCAN_DUPLICATE_OPTIONS;

    mask |= BTDM_CFG_SEND_ADV_RESERVED_SIZE;

    return mask;
}

static void btdm_controller_mem_init(void)
{
    /* initialise .data section */
    memcpy(&_data_start_btdm, (void *)_data_start_btdm_rom, &_data_end_btdm - &_data_start_btdm);
    BT_DBG(".data initialise [0x%08x] <== [0x%08x]", (uint32_t)&_data_start_btdm, _data_start_btdm_rom);

    //initial em, .bss section
    for (int i = 1; i < sizeof(btdm_dram_available_region)/sizeof(btdm_dram_available_region_t); i++) {
        if (btdm_dram_available_region[i].mode) {
            memset((void *)btdm_dram_available_region[i].start, 0x0, btdm_dram_available_region[i].end - btdm_dram_available_region[i].start);
            BT_DBG(".bss initialise [0x%08x] - [0x%08x]", btdm_dram_available_region[i].start, btdm_dram_available_region[i].end);
        }
    }
}




int esp_bt_controller_init(void )
{
    int err;
    uint32_t btdm_cfg_mask = 0;
	k_fifo_init(&mutex_fifo);
	k_fifo_init(&sem_fifo);
	k_fifo_init(&queue_fifo);

    if (btdm_osi_funcs_register(osi_funcs_ro) != 0) {
        return -1;
    }


    btdm_controller_mem_init();


    btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_ORIG);

    btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE);

    btdm_cfg_mask = btdm_config_mask_load();
	
	esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (btdm_controller_init(btdm_cfg_mask, &cfg) != 0) {
        return -1;
    }

    return err;
}


int esp_bt_controller_enable(void )
{
    int ret;

    // inititalize bluetooth baseband
    btdm_check_and_init_bb();

    ret = btdm_controller_enable(1);
	return ret;
}

int esp_bt_controller_disable(void)
{


    btdm_controller_disable();
	return 0;
}



/* extra functions */
int esp_ble_tx_power_set(int power_type, int power_level)
{
    if (ble_txpwr_set(power_type, power_level) != 0) {
        return -1;
    }

    return 0;
}
#endif /*  CONFIG_BT_ENABLED */
