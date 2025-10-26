#include <stdio.h>
#include "pm.h"
#include "se_services_port.h"
#include "clock_runtime.h"

/**
 * @brief Configure the power domains and memory blocks for runtime operation.
 * 
 * This function:
 * - Initializes the secure enclave service port
 * - Retrieves the current run profile
 * - Applies power domain and memory retention configuration
 * 
 * @return 0 on success, non-zero on failure
 */
int32_t config_run_power_domain(void)
{
    run_profile_t runp = {0};
    uint32_t error_code;
    uint32_t service_error_code;

    se_services_port_init();

    error_code = SERVICES_get_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if (error_code)
    {
        return -1;
    }

    // Configure desired power domains
    runp.power_domains =
          PD_VBAT_AON_MASK      // PD0: AON LPRTC, LPGPIO, LPTIMER
        | PD_SSE700_AON_MASK    // PD2: LPUART, LPI2C, SERAM controller
        | PD_RTSS_HE_MASK       // PD3: HE core, LPCPI, etc.
        | PD_SESS_MASK          // PD5: Secure enclave
        | PD_SYST_MASK;         // PD6: System-level clocks, interconnect

    // Configure memory to retain during runtime
    runp.memory_blocks =
          MRAM_MASK
        | SERAM_MASK
        | FWRAM_MASK
        | SRAM4_1_MASK | SRAM4_2_MASK
        | SRAM5_1_MASK | SRAM5_2_MASK;

    error_code = SERVICES_set_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if (error_code)
    {
        return -2;
    }
    
    return 0;
}

/**
 * @brief Configure the power domains and memory blocks for IDLE operation.
 * 
 * This function:
 * - Retrieves the current run profile
 * - Applies power domain and memory retention configuration for HE off state
 * 
 * @return 0 on success, non-zero on failure
 */
int32_t config_off_power_domain(void)
{
    off_profile_t offp = {0};
    uint32_t error_code;
    uint32_t service_error_code;
            /* Get the current off configuration from SE */
            error_code = SERVICES_get_off_cfg(se_services_s_handle,
                &offp,
                &service_error_code);
    if(error_code)
    {
        return -1;
    }

    offp.power_domains = PD_SSE700_AON_MASK |   // PD 2 
                         PD_SYST_MASK;          // PD 6

    offp.aon_clk_src   = CLK_SRC_LFXO;          
    offp.stby_clk_src  = CLK_SRC_HFXO;

    offp.ewic_cfg      = EWIC_RTC_A | EWIC_VBAT_GPIO;
    offp.wakeup_events = WE_LPRTC | WE_LPGPIO4;

    // offp.vtor_address  = SCB->VTOR;

    offp.memory_blocks = MRAM_MASK;
    /*
    * Enable the HE TCM retention only if the VTOR is present.
    * This is just for this test application.
    */
    if(RTSS_Is_TCM_Addr((const volatile void*)SCB->VTOR))
    {
        offp.memory_blocks = SRAM4_1_MASK | SRAM4_2_MASK
            | SRAM5_1_MASK | SRAM5_2_MASK
            | SERAM_MASK;
    }
    else
    {
        /* Enable SERAM if HE VTOR is in MRAM */
        offp.memory_blocks |= SERAM_MASK;
    }

    error_code = SERVICES_set_off_cfg(se_services_s_handle,
                    &offp,
                    &service_error_code);
    if(error_code)
    {
        return -2;
    }
    return 0;
}


/**
 * @brief Configure the power domains and memory blocks for IDLE operation.
 * 
 * This function: Sets the HE core to IDLE mode
 * 
 * @return 0 on success, non-zero on failure
*/
void set_cpu_idle(void)
{
    // Disable interrupts before going to sleep
    __disable_irq();

    // Go for Sleep
    pm_core_enter_deep_sleep_request_subsys_off();
}



