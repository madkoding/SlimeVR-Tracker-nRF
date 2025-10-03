# Problemas Críticos Encontrados - Sistema se "Congela"

## Problema Principal
El sistema pierde la funcionalidad del botón SW0 y se "congela", lo que indica que algo está haciendo que todo el sistema se caiga o quede en un estado inconsistente.

---

## 1. 🔴 CRÍTICO: Race Condition en `sensor_sensor_scanning`

### Ubicaciones Problemáticas:

#### `src/sensor/sensor.c:199-213` - Timeout puede dejar flag en `true`
```c
int sensor_scan(void)
{
	// Timeout protection for concurrent scan attempts
	int64_t timeout_start = k_uptime_get();
	while (sensor_sensor_scanning)
	{
		if (k_uptime_get() - timeout_start > 5000) // 5s timeout
		{
			LOG_ERR("Timeout waiting for concurrent sensor scan");
			sensor_sensor_scanning = false;  // ⚠️ Se fuerza false pero puede haber otro thread escaneando
			break;
		}
		k_usleep(1); // already scanning
	}
	if (sensor_sensor_init)
		return 0; // already initialized
	sensor_sensor_scanning = true;  // ⚠️ Puede estar ya en true desde otro thread
```

**Problema:** Si dos threads intentan escanear simultáneamente, el timeout puede forzar `false` mientras otro thread todavía está escaneando, causando estado inconsistente.

---

#### `src/sensor/sensor.c:385-410` - `k_thread_abort()` no limpia flags
```c
int sensor_request_scan(bool force)
{
	if (sensor_sensor_init && !force)
		return 0; // already initialized
	main_imu_suspend();
	k_thread_abort(&sensor_thread_id); // ⚠️ ABORTA sin limpiar estado
	LOG_INF("Aborted sensor thread");
	main_suspended = false;
	sensor_sensor_init = false;  // ⚠️ Limpia esto...
	// ⚠️ PERO NO LIMPIA sensor_sensor_scanning!
	// ⚠️ PERO NO LIMPIA main_running!
	// ⚠️ PERO NO restaura interfaces si estaban suspendidas!
```

**Problema GRAVE:** 
- `k_thread_abort()` mata el thread abruptamente
- El thread puede estar en medio de un scan con `sensor_sensor_scanning = true`
- El thread puede tener `main_running = true`
- Las interfaces I2C/SPI pueden estar en estado `resumed`
- Los GPIO interrupts pueden estar configurados

**Consecuencias:**
- Otros threads quedan esperando a que `sensor_sensor_scanning` sea `false` → **DEADLOCK**
- Las interfaces quedan suspendidas → **BOTÓN NO FUNCIONA** (si comparte bus)
- El thread del botón puede quedarse bloqueado esperando recursos

---

#### `src/sensor/sensor.c:1196-1222` - Timeouts en `main_imu_suspend()`
```c
void main_imu_suspend(void)
{
	main_suspended = true;
	if (!main_running) // don't suspend if already stopped
		return;
	
	// Timeout protection for sensor scanning
	int64_t timeout_start = k_uptime_get();
	while (sensor_sensor_scanning)
	{
		if (k_uptime_get() - timeout_start > 1000) // 1s timeout
		{
			LOG_ERR("Timeout waiting for sensor scan, forcing exit");
			sensor_sensor_scanning = false;  // ⚠️ Fuerza false pero no limpia otros estados
			break;
		}
		k_usleep(1);
	}
	
	// Timeout protection for main loop
	timeout_start = k_uptime_get();
	while (main_running)
	{
		if (k_uptime_get() - timeout_start > 1000) // 1s timeout
		{
			LOG_ERR("Timeout waiting for sensor thread, forcing exit");
			main_running = false;  // ⚠️ Fuerza false pero el thread sigue corriendo!
			break;
		}
		k_usleep(1);
	}
	
	k_thread_suspend(&sensor_thread_id);  // ⚠️ Suspende sin garantizar estado limpio
```

**Problema:** Los timeouts fuerzan salida pero:
- No limpian las interfaces (pueden quedar suspendidas)
- No limpian los GPIO callbacks
- El thread puede estar en medio de una transacción I2C/SPI
- `main_running = false` pero el thread sigue en el loop

---

## 2. 🔴 CRÍTICO: Interfaces Suspendidas Permanentemente

### Problema: `sys_interface_suspend/resume` desbalanceado

Si ocurre un error o excepción después de `sys_interface_resume()` pero antes de `sys_interface_suspend()`, las interfaces (I2C/SPI) quedan suspendidas permanentemente.

### Ubicaciones Problemáticas:

#### `src/sensor/sensor.c:179-194` - En `sensor_scan_thread()`
```c
void sensor_scan_thread(void)
{
	int err;
	sys_interface_resume(); // ✅ Resume interfaces
	err = sensor_scan(); // ⚠️ Si esto falla o el thread es abortado...
	if (err)
	{
		k_msleep(5);
		LOG_INF("Retrying sensor detection");
		sensor_imu_dev.addr = 0x00;
		err = sensor_scan(); // ⚠️ Si falla aquí...
	}
	sys_interface_suspend(); // ❌ NUNCA SE LLAMA si hay abort o error antes
}
```

**Problema:** Si `k_thread_abort()` es llamado desde `sensor_request_scan()` mientras este thread está corriendo, las interfaces quedan en estado `resumed` permanentemente.

---

#### `src/sensor/sensor.c:715-1115` - En `sensor_loop()` - Ventana ENORME
```c
void sensor_loop(void)
{
	// ...
	main_running = true;
	sys_interface_resume(); // ✅ Resume interfaces al inicio
	int err = sensor_init(); // ⚠️ Si falla...
	// ...
	while (1)
	{
		// ...
		if (main_ok)
		{
			// Resume devices
			sys_interface_resume(); // ✅ Resume aquí también
			
			// ⚠️ MUCHAS OPERACIONES AQUÍ (300+ líneas)
			// - mag_oneshot()
			// - temp_read()
			// - fifo_read()
			// - mag_read()
			// - fifo_process()
			// - fusion updates
			// - connection updates
			// - etc.
			
			// ⚠️ Si cualquier cosa falla o el thread es suspendido/abortado...
			
			// Suspend devices
			sys_interface_suspend(); // ❌ Puede no alcanzar este punto
		}
		
		// ⚠️ Si main_suspended = true aquí:
		if (main_suspended)
			k_thread_suspend(&sensor_thread_id); // ❌ Thread suspendido con interfaces activas!
	}
}
```

**Problema GRAVE:** 
- Hay cientos de líneas entre `resume()` y `suspend()`
- Cualquier error, timeout, o llamada a `main_imu_suspend()` puede interrumpir
- Las interfaces quedan activas consumiendo energía
- Otros threads (como el botón) pueden tener problemas accediendo a las interfaces

---

#### `src/sensor/sensor.c:492-498` - En `sensor_shutdown()`
```c
void sensor_shutdown(void)
{
	int err = sensor_request_scan(false); // ⚠️ Esto puede abortar threads
	if (mag_available || !err)
	{
		sys_interface_resume(); // ✅ Resume
		if (mag_available)
			sensor_mag->shutdown(); // ⚠️ Si falla...
		if (!err)
			sensor_imu->shutdown(); // ⚠️ Si falla...
		sys_interface_suspend(); // ❌ Puede no alcanzar
	}
	else
	{
		LOG_ERR("Failed to shutdown sensors");
		// ⚠️ No intenta suspender interfaces en caso de error
	}
}
```

---

## 3. 🟡 IMPORTANTE: GPIO Interrupt del Botón Puede Perderse

### `src/system/system.c:280-288` - Sin recovery para botón

```c
static void button_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool pressed = button_read();
	int64_t current_time = k_uptime_get();
	if (press_time && !pressed && current_time - press_time > 50) // debounce
		last_press_duration = current_time - press_time;
	else if (press_time && pressed) // unusual press event on button already pressed
		return;
	press_time = pressed ? current_time : 0;
}
```

**Problema:** El sensor tiene recovery para GPIO interrupts (líneas 1115-1155) pero el botón NO.

**Comparación con sensor:**
- Sensor: Tiene timeout de 50ms, cuenta timeouts consecutivos, y re-inicializa GPIO si falla
- Botón: Solo registra eventos, **no tiene detección de fallas ni recovery**

---

## 4. 🟡 IMPORTANTE: Estados No Limpiados al Abortar Thread

### `src/sensor/sensor.c:390` - Lista de estados no limpiados

Cuando se llama `k_thread_abort(&sensor_thread_id)`, no se limpian:

```c
// Estados globales que NO se limpian:
static bool main_running = false;        // ⚠️ Puede quedar en true
static bool sensor_sensor_scanning;      // ⚠️ Puede quedar en true  
static bool main_suspended;              // ⚠️ Puede quedar inconsistente
static bool main_wfi;                    // ⚠️ Puede quedar en true
static int consecutive_sensor_timeouts;  // ⚠️ No se resetea

// Recursos que NO se liberan/deshabilitan:
// - GPIO interrupts (sensor_cb_data)
// - Interfaces I2C/SPI (pueden quedar resumed)
// - Estados del sensor (pueden estar a mitad de operación)
// - FIFO del sensor (puede tener datos pendientes)
```

---

## 5. 🔴 CRÍTICO: Orden de Operaciones en Shutdown

### `src/system/power.c:235-250` - `sys_request_system_off()`

```c
void sys_request_system_off(void)
{
	LOG_INF("System off requested");
	// TODO: fails, its possible that it is getting stuck at main_imu_suspend
	main_imu_suspend(); // ⚠️ Puede bloquearse aquí (ver problema #1)
	configure_system_off(); // Llama sensor_shutdown()
	sensor_scan_clear();
	// ...
}
```

**Problema:** Si `main_imu_suspend()` se bloquea esperando a `sensor_sensor_scanning` o `main_running`, el sistema nunca se apaga y queda congelado.

---

## 6. 🟡 IMPORTANTE: Thread del Botón Puede Bloquearse

### `src/system/system.c:313-380` - `button_thread()`

```c
static void button_thread(void)
{
	// ...
	while (1)
	{
		// ...
		if (num_presses == 1)
			sys_request_system_reboot(); // ⚠️ Puede bloquearse
		// ...
		if (sys_user_shutdown()) // ⚠️ Puede bloquearse
		{
			// ...
		}
		k_msleep(20);
	}
}
```

**Problema:** Si el thread del botón llama a `sys_request_system_reboot()` o `sys_user_shutdown()`, estos pueden bloquearse esperando al sensor thread, y el thread del botón queda congelado → **El botón deja de funcionar**.

---

## 7. 🔴 CRÍTICO: Sensor Loop puede Suspenderse Con Estado Inconsistente

### `src/sensor/sensor.c:1176` - Suspensión en medio del loop

```c
void sensor_loop(void)
{
	// ...
	while (1)
	{
		// ... muchas operaciones ...
		
		if (main_suspended) // ⚠️ Puede activarse en cualquier momento
			k_thread_suspend(&sensor_thread_id); // ❌ Suspende con estado desconocido
		
		main_running = true; // ⚠️ Esta línea está DESPUÉS de la suspensión
	}
}
```

**Problemas:**
1. El thread puede suspenderse con:
   - Interfaces resumidas
   - GPIO interrupts activos
   - `main_running = false` (línea 1176 es ANTES de `main_running = true` en línea 1178)
2. La variable `main_running` se establece al FINAL del loop, pero se verifica para timeouts al PRINCIPIO

---

## Resumen de Soluciones Necesarias

### Prioridad CRÍTICA:
1. **Agregar cleanup completo en `sensor_request_scan()`** antes de abortar thread
2. **Usar try-finally o error handling** para garantizar `sys_interface_suspend()`
3. **Atomicity para `sensor_sensor_scanning`** usando mutex/semáforo
4. **Re-inicializar TODOS los estados** después de abortar thread
5. **Verificar orden de `main_running` en el loop**

### Prioridad IMPORTANTE:
6. **Agregar recovery para GPIO interrupt del botón** similar al del sensor
7. **Mover `main_running = true`** al principio del loop
8. **Timeouts no-bloqueantes** para shutdown operations
9. **Verificar que interfaces estén resumed antes de operaciones**

### Prioridad MEDIA:
10. **Logging mejorado** para debug de estados
11. **Watchdog timer** para detectar deadlocks
12. **Estado de health check** para verificar consistencia

---

## Escenario Más Probable del Bug

1. Usuario presiona botón SW0
2. `button_thread()` llama a `sys_request_system_reboot()` o similar
3. Esto llama a `main_imu_suspend()` que espera a `sensor_sensor_scanning == false`
4. Pero el sensor thread fue abortado con `sensor_sensor_scanning == true`
5. `main_imu_suspend()` timeout después de 1 segundo y fuerza `sensor_sensor_scanning = false`
6. Pero las interfaces quedan en estado inconsistente (suspended o resumed mal)
7. El siguiente scan intenta ejecutar pero las interfaces no responden
8. Se produce otro timeout
9. El sistema entra en loop de timeouts y errores
10. El thread del botón se bloquea esperando a que termine el reboot
11. **El botón deja de responder permanentemente**

---

## Siguiente Paso Recomendado

Implementar las siguientes fixes en orden:

1. **Fix #1:** Limpiar todos los estados en `sensor_request_scan()` antes de `k_thread_abort()`
2. **Fix #2:** Garantizar `sys_interface_suspend()` con error handling
3. **Fix #3:** Usar semáforo para `sensor_sensor_scanning` en lugar de bool
4. **Fix #4:** Agregar recovery para botón GPIO
5. **Fix #5:** Mover `main_running = true` al principio del loop

¿Proceder con las correcciones?
