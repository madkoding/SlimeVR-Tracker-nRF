# Fix para Tracker que Deja de Enviar Datos tras Errores del IMU

## Problema Identificado

El tracker dejaba de enviar datos después de experimentar errores o timeouts del IMU, pero continuaba funcionando internamente. Cuando se conectaba por USB, el IMU seguía respondiendo al movimiento, pero no se enviaban datos al receptor.

### Causa Raíz

El problema principal estaba en la variable `main_ok` en `src/sensor/sensor.c`:

1. **Sin Recuperación Automática**: Cuando `main_ok` se establecía en `false` (línea 1558), el bucle principal continuaba ejecutándose pero saltaba toda la lógica de procesamiento de datos dentro del bloque `if (main_ok)`.

2. **Estado Zombie**: El thread del sensor permanecía activo pero no procesaba ni enviaba datos, creando un estado "zombie" donde:
   - El IMU seguía funcionando
   - Las interrupciones seguían llegando
   - El loop principal seguía corriendo
   - **PERO** no se procesaban datos ni se enviaba nada

3. **Falta de Reintentos**: No había ningún mecanismo de recuperación automática para intentar reinicializar el sensor cuando entraba en este estado.

### Puntos de Fallo Identificados

1. **Timeouts Consecutivos**: Después de 20 timeouts de interrupción del sensor (línea 1500-1558)
2. **Fallo en Reconfiguración GPIO**: Si la reconfiguración del GPIO fallaba después de los timeouts (línea 1558)
3. **Errores de Comunicación Acumulados**: Los errores SPI/I2C se acumulaban sin recuperación hasta llegar al límite crítico

## Cambios Implementados

### 1. ✅ Sistema de Recuperación Automática

**Archivo**: `src/sensor/sensor.c` (líneas ~843-846, ~888-942)

**Agregado**: Sistema completo de recuperación cuando `main_ok = false`

```c
// Recovery tracking for main_ok failures
static int64_t last_recovery_attempt_time = 0;
static int recovery_attempt_count = 0;
#define RECOVERY_COOLDOWN_MS 5000  // Wait 5 seconds between recovery attempts
#define MAX_RECOVERY_ATTEMPTS 3    // Try 3 times before giving up
```

**Lógica de Recuperación**:
```c
// Recovery logic when main_ok is false
if (!main_ok) {
    int64_t current_time = k_uptime_get();
    
    // Check if we should attempt recovery
    if (current_time - last_recovery_attempt_time >= RECOVERY_COOLDOWN_MS) {
        recovery_attempt_count++;
        last_recovery_attempt_time = current_time;
        
        LOG_WRN("Sensor in error state, attempting recovery #%d/%d", 
            recovery_attempt_count, MAX_RECOVERY_ATTEMPTS);
        
        if (recovery_attempt_count <= MAX_RECOVERY_ATTEMPTS) {
            // Clear error status
            set_status(SYS_STATUS_SENSOR_ERROR, false);
            
            // Suspend interfaces
            sys_interface_suspend();
            k_msleep(100);
            
            // Try to reinitialize the sensor
            LOG_INF("Re-initializing sensor...");
            sys_interface_resume();
            int err = sensor_init();
            
            if (err == 0) {
                LOG_INF("Sensor recovery successful!");
                main_ok = true;
                recovery_attempt_count = 0;
                consecutive_sensor_timeouts = 0;
                packet_errors = 0;
                ssi_reset_error_counter();
            } else {
                LOG_ERR("Sensor recovery failed with error %d", err);
                sys_interface_suspend();
                
                if (recovery_attempt_count >= MAX_RECOVERY_ATTEMPTS) {
                    LOG_ERR("Maximum recovery attempts reached, requesting system reboot");
                    sensor_retained_write();
                    sys_request_system_reboot();
                }
            }
        }
    }
    
    // Sleep while in error state to avoid busy loop
    k_msleep(100);
    continue;
}
```

**Razón**: 
- Detecta cuando el sensor entra en estado de error (`main_ok = false`)
- Intenta recuperación automática cada 5 segundos
- Reinicializa completamente el sensor (suspend → resume → init)
- Resetea todos los contadores de error
- Después de 3 intentos fallidos, solicita reinicio del sistema
- Evita busy loop mientras está en estado de error

**Impacto**:
- ✅ Recuperación automática sin intervención manual
- ✅ El tracker vuelve a enviar datos después de errores temporales
- ✅ Previene el estado "zombie" donde el sensor funciona pero no envía datos
- ✅ Reinicio automático del sistema solo como último recurso
- ⚠️ Cooldown de 5 segundos entre intentos para evitar thrashing

---

### 2. ✅ Recuperación Proactiva de Errores de Comunicación

**Archivo**: `src/sensor/sensor.c` (líneas ~968-980)

**Agregado**: Detección y recuperación temprana de errores de comunicación

```c
// Check for communication errors before reading - proactive recovery
int comm_errors = ssi_get_consecutive_errors();
if (comm_errors >= 10) {  // Half of MAX_COMM_ERRORS
    LOG_WRN("Detected %d communication errors, attempting proactive recovery", comm_errors);
    
    // Try a gentle recovery without disrupting the main loop
    sys_interface_suspend();
    k_msleep(10);
    sys_interface_resume();
    ssi_reset_error_counter();
    
    LOG_INF("Proactive bus recovery completed");
}
```

**Razón**: 
- Detecta errores de comunicación antes de que lleguen al límite crítico (20)
- Intenta recuperación del bus cuando se detectan 10 errores (50% del límite)
- Recuperación suave sin interrumpir el flujo principal
- Previene acumulación de errores que llevarían a `main_ok = false`

**Impacto**:
- ✅ Recuperación temprana evita fallos críticos
- ✅ Mantiene el sistema funcionando durante errores transitorios
- ✅ Reduce la probabilidad de entrar en estado de error completo
- ✅ Solo 10ms de latencia durante la recuperación
- ✅ Transparente para el usuario final

---

## Flujo de Recuperación

### Escenario 1: Errores de Comunicación Menores
1. Se detectan errores de comunicación acumulándose
2. Al llegar a 10 errores → Recuperación proactiva (10ms)
3. Bus se reinicializa sin interrumpir el envío de datos
4. Contador de errores se resetea
5. **Resultado**: Tracker continúa funcionando sin interrupciones visibles

### Escenario 2: Errores Mayores que Causan main_ok = false
1. Múltiples timeouts o fallo crítico establece `main_ok = false`
2. Loop detecta el estado de error
3. Espera 5 segundos (cooldown)
4. Intenta reinicializar el sensor completamente
5. **Si tiene éxito**: 
   - `main_ok = true`
   - Todos los contadores se resetean
   - Tracker vuelve a funcionar normalmente
6. **Si falla**:
   - Reintenta hasta 3 veces (5s entre intentos)
   - Después de 3 fallos → Reinicia el sistema automáticamente

### Escenario 3: Errores Irrecuperables
1. 3 intentos de recuperación fallan
2. Se guarda el estado de fusión (para mantener calibración)
3. Sistema se reinicia automáticamente
4. Al reiniciar, el tracker vuelve a estado operativo

---

## Logs a Monitorear

### Logs Normales (Operación Correcta)
```
[sensor] Sensor recovery successful!
[sensor_interface] SPI communication recovered after N errors
[sensor] Proactive bus recovery completed
```

### Logs de Advertencia (Recuperación en Curso)
```
[sensor] Sensor in error state, attempting recovery #1/3
[sensor] Detected 10 communication errors, attempting proactive recovery
[sensor_interface] SPI comm errors detected: 10 (early warning)
```

### Logs Críticos (Requieren Atención)
```
[sensor] Sensor recovery failed with error -1
[sensor] Maximum recovery attempts reached, requesting system reboot
[sensor] Too many consecutive sensor timeouts, attempting safe recovery
```

---

## Testing Recomendado

### Test 1: Recuperación de Errores de Comunicación
1. Dejar el tracker funcionando normalmente
2. Causar errores transitorios (por ejemplo, interferencia RF)
3. ✅ Verificar que aparecen logs de "Proactive bus recovery"
4. ✅ Verificar que el tracker continúa enviando datos sin interrupciones

### Test 2: Recuperación de Estado de Error
1. Forzar condición de error (cubrir el tracker para causar timeouts)
2. ✅ Verificar que aparece "Sensor in error state, attempting recovery"
3. ✅ Verificar que después de 5 segundos intenta recuperación
4. ✅ Verificar que vuelve a enviar datos después de la recuperación

### Test 3: Reinicio Automático
1. Forzar error irrecuperable (desconectar IMU físicamente)
2. ✅ Verificar 3 intentos de recuperación espaciados 5 segundos
3. ✅ Verificar mensaje "Maximum recovery attempts reached"
4. ✅ Verificar que el sistema se reinicia automáticamente

### Test 4: Operación de Larga Duración
1. Dejar tracker funcionando por 24-48 horas
2. ✅ Monitorear logs de errores y recuperaciones
3. ✅ Verificar que no hay interrupciones prolongadas en el envío de datos
4. ✅ Verificar que las recuperaciones (si ocurren) son exitosas

---

## Comparación: Antes vs Después

### Antes
- ❌ Tracker deja de enviar datos tras errores
- ❌ Requiere reinicio manual para recuperarse
- ❌ Estado "zombie" donde funciona pero no envía datos
- ❌ Sin logs que indiquen intentos de recuperación
- ❌ Usuario debe detectar el problema y reiniciar manualmente

### Después
- ✅ Recuperación automática de errores de comunicación
- ✅ Recuperación automática del estado de error completo
- ✅ Reinicio automático como último recurso
- ✅ Logs detallados de todos los intentos de recuperación
- ✅ Funcionamiento transparente para el usuario
- ✅ Múltiples niveles de recuperación (proactiva → reactiva → reinicio)

---

## Archivos Modificados

1. `src/sensor/sensor.c` - Sistema de recuperación automática y detección proactiva de errores

## Notas Adicionales

- **Cooldown de 5 segundos**: Previene thrashing cuando hay problemas persistentes
- **Máximo 3 intentos**: Balance entre persistencia y evitar loops infinitos
- **Preservación del estado**: Se guarda el estado de fusión antes del reinicio
- **Recuperación en capas**: Proactiva (10 errores) → Reactiva (estado de error) → Reinicio (3 fallos)
- **Transparencia**: La mayoría de recuperaciones son invisibles para el usuario

---

## Próximos Pasos si el Problema Persiste

Si después de estos cambios un tracker aún deja de enviar datos:

1. **Revisar logs USB**: Conectar por USB y capturar logs durante el incidente
2. **Hardware**: Verificar conexiones físicas del IMU, capacitores, alimentación
3. **Interferencia**: Verificar posibles fuentes de EMI/RFI
4. **Temperatura**: Verificar si ocurre bajo condiciones térmicas específicas
5. **Patrón de movimiento**: Documentar si ocurre con movimientos específicos
6. **Tiempo hasta fallo**: Documentar cuánto tiempo funciona antes del problema

---

## Autor

madkoding - Octubre 2025

## Relacionado

- `ICM45686_TIMEOUT_FIX.md` - Fix para timeouts del ICM-45686
- `CHANGES_ICM45686_TIMEOUT_FIX.md` - Cambios implementados para timeouts
- `FIFO_OVERFLOW_FIX.md` - Fix para congelamiento por FIFO overflow
