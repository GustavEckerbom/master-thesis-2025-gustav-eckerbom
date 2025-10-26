#ifndef POWER_CONFIG_H
#define POWER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize secure enclave and configure power/memory domains
 * 
 * Applies run-time power domain and memory retention settings.
 * 
 * @return 0 on success, non-zero on failure
 */
int config_run_power_domain(void);
int config_off_power_domain(void);
void set_cpu_idle(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_CONFIG_H
