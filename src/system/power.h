#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

#include <stdbool.h>

void sys_interface_suspend(void);
void sys_interface_resume(void);

/* Request wake-on-motion enable/disable */
void sys_request_WOM(bool enable);
void sys_request_system_off(void);
void sys_request_system_reboot(void);

bool vin_read(void);

#endif