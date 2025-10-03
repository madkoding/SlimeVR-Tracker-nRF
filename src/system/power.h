#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

void sys_interface_suspend(void);
void sys_interface_resume(void);

bool sys_imu_wakeup_available(void);

void sys_request_WOM(bool);
void sys_request_system_off(void);
void sys_request_system_reboot(void);

bool vin_read(void);

#endif