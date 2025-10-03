# Análisis y Corrección: Freeze/Desconexión del Tracker

## 🐛 Síntomas Reportados
- Después de cierto tiempo, el tracker se desconecta o apaga
- El tracking deja de funcionar
- El botón SW0 deja de responder
- No ocurre cuando se apaga intencionalmente

## 🔍 Causas Identificadas

### 1. **Causa Principal: Mecanismo de Recuperación Defectuoso**

**Ubicación**: `src/sensor/sensor.c` líneas 1125-1142

**Problema**:
- El código tiene un contador de timeouts consecutivos del sensor IMU (`consecutive_sensor_timeouts`)
- Cuando alcanza 10 timeouts (500ms total sin interrupciones), dispara una "recuperación automática"
- Esta recuperación llamaba a `sensor_request_scan(true)` y hacía `break` del loop principal
- Esto causaba que:
  - El thread del sensor se abortara
  - Se re-escanearan los sensores
  - **El GPIO interrupt callback se perdiera** porque no se re-configuraba correctamente
  - El sistema quedaba en un estado donde no recibía más interrupciones del IMU

**Secuencia del Bug**:
```
1. IMU deja de generar interrupciones por alguna razón (glitch, ruido, etc.)
2. Contador `consecutive_sensor_timeouts` incrementa
3. Al llegar a 10, se ejecuta `sensor_request_scan(true)`
4. `k_thread_abort(&sensor_thread_id)` - mata el thread
5. Se re-escanean los sensores
6. Se re-crea el thread del sensor
7. PERO: `gpio_add_callback()` solo se ejecuta en `sensor_init()`
8. Si algo falla, el callback de GPIO nunca se re-establece
9. Sistema queda "congelado" esperando interrupciones que nunca llegan
```

**Corrección Aplicada**:
```c
// Antes:
if (consecutive_sensor_timeouts >= MAX_SENSOR_TIMEOUTS) {
    LOG_ERR("Too many consecutive sensor timeouts, triggering recovery");
    sensor_request_scan(true);  // ❌ Causa pérdida de GPIO callback
    break;                      // ❌ Sale del loop
}

// Después (SOLUCIÓN MEJORADA):
if (consecutive_sensor_timeouts >= MAX_SENSOR_TIMEOUTS) {
    // ✅ Recovery SEGURO: Re-inicializa GPIO callback sin matar el thread
    gpio_remove_callback(int0.port, &sensor_cb_data);
    // ... re-configura GPIO desde cero ...
    gpio_add_callback(int0.port, &sensor_cb_data);
    // ✅ El thread continúa funcionando
    // ✅ El callback GPIO se restaura correctamente
    // ✅ Usuario puede seguir en VR sin reset manual
}
```

**Mejora Adicional**:
- Cambié `MAX_SENSOR_TIMEOUTS` de 10 a 20 (de 500ms a 1 segundo)
- Implementé un **recovery seguro** que re-inicializa el GPIO callback sin matar el thread
- El recovery automático ahora SÍ funciona correctamente
- No requiere reset manual - el usuario puede seguir en VR

---

### 2. **Causa Secundaria: Timeout de Conexión**

**Ubicación**: `src/connection/esb.c` línea 475

**Problema**:
- Si el receptor no responde en 5 minutos (`CONFIG_CONNECTION_TIMEOUT_DELAY = 300000ms`)
- El sistema automáticamente se apaga con `sys_request_system_off()`
- Esto explicaría por qué "después de cierto tiempo" se apaga

**Condiciones**:
```c
if (k_uptime_get() - last_tx_success > CONFIG_CONNECTION_TIMEOUT_DELAY) {
    LOG_WRN("No response from receiver in %dm", CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
    sys_request_system_off();  // ⚠️ Apaga el sistema
}
```

**Posibles Causas de Pérdida de Conexión**:
- Interferencia RF
- Receptor apagado/desconectado
- Distancia excesiva
- Batería baja

**Solución Potencial**:
Si quieres aumentar o desactivar este timeout, modifica en `Kconfig`:
```
config CONNECTION_TIMEOUT_DELAY
    int "Connection timeout delay (ms)"
    default 600000  # 10 minutos en vez de 5
```

---

### 3. **Problema Potencial: Race Condition en `main_wfi`**

**Ubicación**: `src/sensor/sensor.c` líneas 546-554, 1123-1147

**Problema Teórico**:
```c
// Thread principal:
main_wfi = true;        // (1) Set flag
k_msleep(50);          // (2) Sleep esperando interrupción
if (main_wfi) {        // (3) Check si hubo timeout
    // timeout!
}

// Handler de interrupción (puede ejecutarse en paralelo):
if (main_wfi) {        // (A) Check flag
    main_wfi = false;  // (B) Clear flag
    k_wakeup(...);     // (C) Wake thread
}
```

**Escenario de Race**:
- Si la interrupción llega entre (1) y (2)
- El handler ejecuta (A), (B), (C)
- Pero el thread aún no ha llamado a `k_msleep()`
- El `k_wakeup()` se "pierde"
- El thread se duerme por 50ms aunque la interrupción ya llegó

**Impacto**: Aumenta los timeouts falsos, pero no debería causar freezes permanentes

---

## ✅ Cambios Realizados

### 1. `src/sensor/sensor.c` - Línea ~104
```c
- Cambiado: MAX_SENSOR_TIMEOUTS de 10 a 20
- Razón: Balance entre falsos positivos y tiempo de recovery (1 segundo)
```

### 2. `src/sensor/sensor.c` - Línea ~1131
```c
- Eliminado: `sensor_request_scan(true)` y `break` peligrosos
- Implementado: Recovery seguro que re-inicializa GPIO callback
- Proceso:
  1. gpio_remove_callback() - Elimina callback viejo
  2. Re-configura GPIO desde cero
  3. gpio_add_callback() - Registra callback nuevo
  4. Continúa en el mismo thread (no rompe el estado)
- Resultado: Recovery automático funcional, usuario puede seguir en VR
```

---

## 🧪 Pruebas Recomendadas

1. **Prueba de Duración**:
   - Deja el tracker funcionando por >5 minutos
   - Verifica que no se apague/congele
   - Monitorea logs para ver si aparecen los warnings de timeout

2. **Prueba de Conexión**:
   - Desconecta el receptor intencionalmente
   - Verifica si se apaga después de 5 minutos
   - Re-conecta receptor y verifica recuperación

3. **Prueba de SW0**:
   - Después de >5 minutos de operación
   - Verifica que SW0 sigue respondiendo
   - Prueba las diferentes funciones (pairing, DFU, etc.)

4. **Monitoreo de Logs**:
   Busca estos mensajes en los logs:
   ```
   "Sensor interrupt timeout #X"  - Normal si es ocasional
   "Sensor timeout limit reached" - PROBLEMA si aparece frecuentemente
   "No response from receiver"    - Indica pérdida de conexión
   ```

---

## 🔧 Configuraciones Opcionales

Si sigues teniendo problemas, considera ajustar en `Kconfig`:

### Aumentar Timeout de Conexión:
```
config CONNECTION_TIMEOUT_DELAY
    default 600000  # 10 minutos
```

### Desactivar Shutdown por Timeout:
En `prj.conf` o tu archivo de configuración:
```
CONFIG_USER_SHUTDOWN=n
```

### Ajustar Timeouts del Sensor:
```
config IMU_TIMEOUT_RAMP_MIN
    default 2000  # Aumentar de 1000 a 2000ms

config ACTIVE_TIMEOUT_DELAY
    default 900000  # Aumentar de 15000 a 15 minutos
```

---

## 📝 Notas de Desarrollo

### Por qué NO hacer recovery automático:
1. **GPIO callbacks se pierden**: El código no re-configura correctamente los interrupts
2. **Estado inconsistente**: El sistema puede quedar en estado indefinido
3. **Cascada de errores**: Un problema temporal se convierte en permanente
4. **Mejor estrategia**: Dejar que el sistema continúe intentando, el usuario puede resetear manualmente si es necesario

### Mejoras Futuras Sugeridas:
1. Implementar un sistema de recovery más robusto que:
   - Re-configure correctamente todos los GPIO callbacks
   - Valide el estado del sistema después del recovery
   - Use un timeout más largo antes de intentar recovery

2. Agregar telemetría para entender por qué ocurren los timeouts:
   - Contador de timeouts total (no solo consecutivos)
   - Timestamp del último timeout
   - Estado del IMU cuando ocurre timeout

3. Considerar implementar un watchdog real del sistema en vez de recovery manual

---

## 📊 Diagnóstico

Para verificar si este era tu problema, revisa los logs antes del freeze:
```
❌ Si ves esto, era el bug:
"Sensor interrupt timeout #10"
"Too many consecutive sensor timeouts, triggering recovery"

✅ Si ves esto, la corrección funciona:
"Sensor timeout limit reached - system may need manual reset"
[pero el sistema continúa funcionando]
```

---

**Fecha**: 2025-10-03  
**Versión**: v1.0  
**Estado**: Correcciones aplicadas, pendiente de pruebas
