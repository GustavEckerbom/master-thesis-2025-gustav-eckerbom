
/* System Includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "Camera_Sensor.h"
#include "Camera_Sensor_i2c.h"
#include "Driver_GPIO.h"
#include "Driver_CPI.h"
#include "sys_ctrl_cpi.h"
#include "board.h"
#include "genx320.h"
#include "system_utils.h"
#include "sys_ctrl_cpi.h"
#include "psee_issd.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "clock_runtime.h"
#include "M55_HE.h"

/* CGU base address and offsets */
#define CGU_OSC_CTRL          (*(volatile uint32_t *)(CGU_BASE + 0x00))
#define CGU_PLL_LOCK_CTRL     (*(volatile uint32_t *)(CGU_BASE + 0x04))
#define CGU_PLL_CLK_SEL       (*(volatile uint32_t *)(CGU_BASE + 0x08))
#define CGU_ESCLK_SEL         (*(volatile uint32_t *)(CGU_BASE + 0x10))
#define CGU_CLK_ENA           (*(volatile uint32_t *)(CGU_BASE + 0x14))
#define CGU_IRQ               (*(volatile uint32_t *)(CGU_BASE + 0x20))


#define SYS_LCTRL_ST              (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x500))
#define SYS_LCTRL_SET             (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x504))
#define SYS_LCTRL_CLR             (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x508))
#define HOSTCPUCLK_CTRL           (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x800))
#define HOSTCPUCLK_DIV0           (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x804))
#define HOSTCPUCLK_DIV1           (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x808))
#define ACLK_CTRL                 (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x820))
#define ACLK_DIV0                 (*(volatile uint32_t *)(CLKCTL_SYS_BASE + 0x824))

// These are typically declared in system_<device>.c or provided by CMSIS
extern uint32_t SystemCoreClock;
extern uint32_t SystemHFOSCClock;
extern uint32_t SystemAPBClock;
extern uint32_t SystemAHBClock;
extern uint32_t SystemAXIClock;
extern uint32_t SystemREFClock;


/* Helper to read CGU registers */
#define CGU_REG(offset) (*(volatile uint32_t *)(CGU_BASE + (offset)))
void print_all_clkctl_sys_registers(void)
{
    uint32_t lctrl_st   = SYS_LCTRL_ST;
    uint32_t lctrl_set  = SYS_LCTRL_SET;
    uint32_t lctrl_clr  = SYS_LCTRL_CLR;
    uint32_t hclk_ctrl  = HOSTCPUCLK_CTRL;
    uint32_t hclk_div0  = HOSTCPUCLK_DIV0;
    uint32_t hclk_div1  = HOSTCPUCLK_DIV1;
    uint32_t aclk_ctrl  = ACLK_CTRL;
    uint32_t aclk_div0  = ACLK_DIV0;

    printf("===== CLKCTL_SYS Register Dump =====\r\n");

    printf("SYS_LCTRL_ST   [0x500] = 0x%08lX\r\n", lctrl_st);
    printf("  LOCK_CLR_DIS     (bit 31): %lu\r\n", (lctrl_st >> 31) & 0x1);
    printf("  INT_RTR_LOCK     (bit 1) : %lu\r\n", (lctrl_st >> 1) & 0x1);

    printf("SYS_LCTRL_SET  [0x504] = 0x%08lX\r\n", lctrl_set);
    printf("SYS_LCTRL_CLR  [0x508] = 0x%08lX\r\n", lctrl_clr);

    printf("HOSTCPUCLK_CTRL [0x800] = 0x%08lX\r\n", hclk_ctrl);
    printf("  CLKSELECT_CUR (bits 15:8) : 0x%02lX\r\n", (hclk_ctrl >> 8) & 0xFF);
    printf("  CLKSELECT     (bits 7:0)  : 0x%02lX\r\n", hclk_ctrl & 0xFF);

    printf("HOSTCPUCLK_DIV0 [0x804] = 0x%08lX\r\n", hclk_div0);
    printf("  CLKDIV_CUR (bits 20:16): 0x%02lX (÷%lu)\r\n",
           (hclk_div0 >> 16) & 0x1F, ((hclk_div0 >> 16) & 0x1F) + 1);
    printf("  CLKDIV     (bits 4:0)  : 0x%02lX (÷%lu)\r\n",
           hclk_div0 & 0x1F, (hclk_div0 & 0x1F) + 1);

    printf("HOSTCPUCLK_DIV1 [0x808] = 0x%08lX\r\n", hclk_div1);
    printf("  CLKDIV_CUR (bits 20:16): 0x%02lX (÷%lu)\r\n",
           (hclk_div1 >> 16) & 0x1F, ((hclk_div1 >> 16) & 0x1F) + 1);
    printf("  CLKDIV     (bits 4:0)  : 0x%02lX (÷%lu)\r\n",
           hclk_div1 & 0x1F, (hclk_div1 & 0x1F) + 1);

    printf("ACLK_CTRL      [0x820] = 0x%08lX\r\n", aclk_ctrl);
    printf("  ENTRY_DELAY     (bits 31:24): 0x%02lX\r\n", (aclk_ctrl >> 24) & 0xFF);
    printf("  CLKSELECT_CUR   (bits 15:8) : 0x%02lX\r\n", (aclk_ctrl >> 8) & 0xFF);
    printf("  CLKSELECT       (bits 7:0)  : 0x%02lX\r\n", aclk_ctrl & 0xFF);

    printf("ACLK_DIV0       [0x824] = 0x%08lX\r\n", aclk_div0);
    printf("  CLKDIV_CUR (bits 20:16): 0x%02lX (÷%lu)\r\n",
           (aclk_div0 >> 16) & 0x1F, ((aclk_div0 >> 16) & 0x1F) + 1);
    printf("  CLKDIV     (bits 4:0)  : 0x%02lX (÷%lu)\r\n",
           aclk_div0 & 0x1F, (aclk_div0 & 0x1F) + 1);

    printf("=====================================\r\n");
}

void print_rtss_he_clk_config(void)
{
    uint32_t clk_ena = CGU_CLK_ENA;
    uint32_t pll_clk_sel = CGU_PLL_CLK_SEL;
    uint32_t esclk_sel = CGU_ESCLK_SEL;

    printf("RTSS_HE_CLK (ES1) Clock Configuration:\n\r");

    // Print raw register values for full transparency
    printf("  CLK_ENA      = 0x%08lX\n\r", clk_ena);
    printf("  PLL_CLK_SEL  = 0x%08lX\n\r", pll_clk_sel);
    printf("  ESCLK_SEL    = 0x%08lX\n\r", esclk_sel);

    // Check if RTSS_HE_CLK is enabled
    if (clk_ena & (1 << 13)) {
        printf("  ES1 Clock Enable: ENABLED (CLK_ENA[13] = 1)\n\r");
    } else {
        printf("  ES1 Clock Enable: DISABLED (CLK_ENA[13] = 0)\n\r");
    }

    // Determine the source: PLL or Oscillator
    if (pll_clk_sel & (1 << 20)) {
        printf("  ES1 Clock Source Select: PLL (PLL_CLK_SEL[20] = 1)\n\r");

        uint32_t es1_pll = (esclk_sel >> 4) & 0x3;
        printf("  ES1_PLL field (ESCLK_SEL[5:4]) = 0x%X\n\r", es1_pll);
        switch (es1_pll) {
            case 0:
                printf("  -> PLL Frequency: 80 MHz (Option 0)\n\r");
                break;
            case 1:
                printf("  -> PLL Frequency: 80 MHz (Option 1)\n\r");
                break;
            case 2:
                printf("  -> PLL Frequency: 160 MHz (Option 2)\n\r");
                break;
            case 3:
                printf("  -> PLL Frequency: 160 MHz (Option 3)\n\r");
                break;
            default:
                printf("  -> PLL Frequency: Unknown (invalid ES1_PLL value)\n\r");
        }

    } else {
        printf("  ES1 Clock Source Select: Oscillator (PLL_CLK_SEL[20] = 0)\n\r");

        uint32_t es1_osc = (esclk_sel >> 12) & 0x3;
        printf("  ES1_OSC field (ESCLK_SEL[13:12]) = 0x%X\n", es1_osc);
        switch (es1_osc) {
            case 0:
                printf("  -> Oscillator Frequency: 76.8 MHz (ring) (Option 0)\n\r");
                break;
            case 1:
                printf("  -> Oscillator Frequency: 38.4 MHz (ring) (Option 1)\n\r");
                break;
            case 2:
                printf("  -> Oscillator Frequency: 76.8 MHz (crystal) (Option 2)\n\r");
                break;
            case 3:
                printf("  -> Oscillator Frequency: 38.4 MHz (crystal) (Option 3)\n\r");
                break;
            default:
                printf("  -> Oscillator Frequency: Unknown (invalid ES1_OSC value)\n\r");
        }
    }
}

void print_mcu_clock_frequencies(void)
{
    if (system_update_clock_values() != 0) {
        printf("Error: Failed to update clock values from services\n\r");
        return;
    }

    printf("MCU Clock Frequencies:\n");
    printf("  SystemCoreClock  = %lu Hz\n\r", SystemCoreClock);
    printf("  SystemHFOSCClock = %lu Hz\n\r", SystemHFOSCClock);
    printf("  SystemAPBClock   = %lu Hz\n\r", SystemAPBClock);
    printf("  SystemAHBClock   = %lu Hz\n\r", SystemAHBClock);
    printf("  SystemAXIClock   = %lu Hz\n\r", SystemAXIClock);
    printf("  SystemREFClock   = %lu Hz\n\r", SystemREFClock);
}

void print_all_cgu_registers(void)
{
    uint32_t osc_ctrl      = CGU_OSC_CTRL;
    uint32_t pll_lock_ctrl = CGU_PLL_LOCK_CTRL;
    uint32_t pll_clk_sel   = CGU_PLL_CLK_SEL;
    uint32_t esclk_sel     = CGU_ESCLK_SEL;
    uint32_t clk_ena       = CGU_CLK_ENA;
    uint32_t irq_status    = CGU_IRQ;

    printf("\n\r===== CGU Register Dump =====\n\r");

    // OSC_CTRL
    printf("OSC_CTRL        [0x00] = 0x%08lX\n\r", osc_ctrl);
    printf("  PERIPH_XTAL_SEL (bit 4)  : %lu\n\r", (osc_ctrl >> 4) & 0x1);
    printf("  SYS_XTAL_SEL    (bit 0)  : %lu\n\r", (osc_ctrl >> 0) & 0x1);
    printf("  XTAL_DEAD       (bit 20) : %lu\n\r", (osc_ctrl >> 20) & 0x1);
    printf("  CLKMON_ENA      (bit 16) : %lu\n\r", (osc_ctrl >> 16) & 0x1);

    // PLL_LOCK_CTRL
    printf("PLL_LOCK_CTRL   [0x04] = 0x%08lX\n\r", pll_lock_ctrl);
    printf("  PLL_LOCK        (bit 0)  : %lu\n\r", (pll_lock_ctrl >> 0) & 0x1);
    printf("  PLL_CALIB       (bit 4)  : %lu\n\r", (pll_lock_ctrl >> 4) & 0x1);

    // PLL_CLK_SEL
    printf("PLL_CLK_SEL     [0x08] = 0x%08lX\n\r", pll_clk_sel);
    printf("  ES1 (RTSS_HE_CLK) source (bit 20): %lu\n\r", (pll_clk_sel >> 20) & 0x1);
    printf("  ES0 (RTSS_HP_CLK) source (bit 16): %lu\n\r", (pll_clk_sel >> 16) & 0x1);
    printf("  SYS PLL source       (bit 4) : %lu\n\r", (pll_clk_sel >> 4) & 0x1);
    printf("  SYSREF source        (bit 0) : %lu\n\r", (pll_clk_sel >> 0) & 0x1);

    // ESCLK_SEL
    printf("ESCLK_SEL       [0x10] = 0x%08lX\n\r", esclk_sel);
    printf("  ES1_OSC (RTSS_HE_CLK) [13:12] = 0x%lX\n\r", (esclk_sel >> 12) & 0x3);
    printf("  ES0_OSC (RTSS_HP_CLK) [9:8]   = 0x%lX\n\r", (esclk_sel >> 8) & 0x3);
    printf("  ES1_PLL (RTSS_HE_CLK) [5:4]   = 0x%lX\n\r", (esclk_sel >> 4) & 0x3);
    printf("  ES0_PLL (RTSS_HP_CLK) [1:0]   = 0x%lX\n\r", (esclk_sel >> 0) & 0x3);

    // CLK_ENA
    printf("CLK_ENA         [0x14] = 0x%08lX\n\r", clk_ena);
    printf("  ES1 Enable (RTSS_HE_CLK) (bit 13): %lu\n\r", (clk_ena >> 13) & 0x1);
    printf("  ES0 Enable (RTSS_HP_CLK) (bit 12): %lu\n\r", (clk_ena >> 12) & 0x1);
    printf("  CPUPLL Enable (bit 4): %lu\n\r", (clk_ena >> 4) & 0x1);
    printf("  SYSPLL Enable (bit 0): %lu\n\r", (clk_ena >> 0) & 0x1);
    printf("  CLK38P4M Enable (bit 23): %lu\n\r", (clk_ena >> 23) & 0x1);
    printf("  CLK20M Enable (bit 22): %lu\n\r", (clk_ena >> 22) & 0x1);
    printf("  CLK100M Enable (bit 21): %lu\n\r", (clk_ena >> 21) & 0x1);
    printf("  CLK160M Enable (bit 20): %lu\n\r", (clk_ena >> 20) & 0x1);

    // CGU_IRQ
    printf("CGU_IRQ         [0x20] = 0x%08lX (Read clears XTAL_DEAD)\n\r", irq_status);
    printf("  XTAL_DEAD interrupt (bit 0): %lu\n\r", irq_status & 0x1);

    printf("================================\n\r");
}