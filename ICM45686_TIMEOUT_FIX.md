# ICM-45686 SPI Timeout Issues - Análisis y Soluciones

## Problema
El ICM-45686 por SPI funciona normalmente pero en momentos aleatorios deja de responder (timeouts), requiriendo un reset para funcionar nuevamente.

## Causas Raíz Identificadas

### 1. **Startup Time Insuficiente (CRÍTICO)**
**Archivo**: `src/sensor/imu/ICM45686.c:61`
```c
k_msleep(1); // fuck i dont wanna wait that long
```
- El datasheet especifica: 10ms para acelerómetro, 30ms para giroscopio
- Solo se espera 1ms, lo que puede causar inicialización incompleta
- **Síntoma**: Funciona inicialmente pero falla aleatoriamente cuando el chip no ha completado su startup interno

### 2. **Sin Detección de Errores SPI Tempranos**
**Archivo**: `src/sensor/interface.c:27`
```c
#define MAX_COMM_ERRORS 100
```
- Los errores SPI se acumulan silenciosamente hasta llegar a 100
- No hay acción correctiva hasta ese punto
- **Síntoma**: Errores graduales que eventualmente causan fallo total

### 3. **No Hay Recuperación del Bus SPI en Timeouts**
**Archivo**: `src/sensor/sensor.c:1488-1520`
- Cuando hay timeout de interrupción, solo se reinicializa el GPIO
- El bus SPI no se reinicializa ni se verifica su estado
- Si el SPI está en estado inválido, permanece así
- **Síntoma**: Después de timeout, todas las comunicaciones fallan

### 4. **Power Management Sin Validación**
**Archivo**: `src/system/power.c:77-95`
- El bus SPI se suspende/resume frecuentemente
- No hay verificación de que el resume fue exitoso
- No hay re-sincronización con el IMU después de resume
- **Síntoma**: Después de ciclos de suspend/resume, el SPI puede quedar desincronizado

### 5. **FIFO Overflow Sin Manejo Robusto**
- Modo: Stop-on-full (0x80)
- Si el FIFO se llena, el IMU puede requerir reset o limpieza especial
- No hay código para detectar y recuperarse de FIFO full
- **Síntoma**: Después de periodo de alta actividad, el IMU deja de responder

### 6. **Falta de Watchdog de Comunicación**
- No hay timeout de comunicación SPI individual
- Una transacción colgada puede bloquear todo el sistema
- **Síntoma**: Sistema completo se congela hasta reset manual

## Soluciones Propuestas (Ordenadas por Prioridad)

### **Prioridad 1: Aumentar Startup Delay**
```c
// En icm45_init(), cambiar:
k_msleep(1); 
// Por:
k_msleep(50); // Esperar 50ms para startup completo (10ms accel + 30ms gyro + margen)
```
**Impacto**: Bajo (solo afecta el inicio)
**Efectividad**: Alta - Elimina problemas de inicialización incompleta

### **Prioridad 2: Agregar Detección Temprana de Errores SPI**
```c
// En interface.c, después de cada transacción SPI crítica:
if (err != 0 && consecutive_comm_errors > 10) { // Umbral más bajo
    LOG_WRN("Early SPI error detection: %d errors", consecutive_comm_errors);
    // Intentar re-configurar el SPI
    sensor_interface_spi_configure(dev, current_frequency, dummy_reads);
}
```

### **Prioridad 3: Implementar Recuperación de Bus SPI**
```c
// Nueva función en interface.c:
int ssi_recover_bus(enum sensor_interface_dev dev) {
    LOG_WRN("Attempting SPI bus recovery for dev %d", dev);
    
    // 1. Reiniciar contador de errores
    consecutive_comm_errors = 0;
    
    // 2. Re-inicializar el device
    if (sensor_interface_dev_spec[dev] == SENSOR_INTERFACE_SPEC_SPI) {
        // Suspend y resume del PM
        sys_interface_suspend();
        k_msleep(10);
        sys_interface_resume();
        
        // Re-configurar SPI
        return sensor_interface_spi_configure(dev, 
                                               sensor_interface_dev_spi[dev]->config.frequency,
                                               sensor_interface_dev_spi_dummy_reads[dev]);
    }
    return -1;
}
```

### **Prioridad 4: Verificar Estado Después de Resume**
```c
// En sys_interface_resume(), agregar verificación:
void sys_interface_resume(void) {
    // ... código existente ...
    
    // Verificar que el IMU responde después de resume
    uint8_t whoami;
    int retry_count = 0;
    while (retry_count < 3) {
        if (ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_WHO_AM_I, &whoami) == 0) {
            if (whoami == ICM45686_WHOAMI_VALUE) {
                break; // Resume exitoso
            }
        }
        k_msleep(5);
        retry_count++;
    }
    
    if (retry_count >= 3) {
        LOG_ERR("IMU no responde después de resume, reinicializando...");
        main_imu_restart();
    }
}
```

### **Prioridad 5: Mejorar Manejo de FIFO**
```c
// En icm45_fifo_read(), agregar detección de overflow:
uint16_t icm45_fifo_read(uint8_t *data, uint16_t len) {
    uint8_t fifo_count_bytes[2];
    int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, fifo_count_bytes, 2);
    
    if (err) {
        LOG_ERR("Failed to read FIFO count");
        consecutive_comm_errors++;
        return 0;
    }
    
    uint16_t fifo_count = (fifo_count_bytes[0] << 8) | fifo_count_bytes[1];
    
    // Detectar FIFO lleno (2048 bytes)
    if (fifo_count >= 2040) {
        LOG_WRN("FIFO near full (%d bytes), flushing", fifo_count);
        // Flush FIFO
        ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x00); // Stop FIFO
        k_msleep(1);
        ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x0F); // Restart FIFO
        return 0;
    }
    
    // ... resto del código ...
}
```

### **Prioridad 6: Implementar SPI Watchdog**
```c
// Agregar timeout a las transacciones SPI críticas
#define SPI_TRANSACTION_TIMEOUT_MS 100

int ssi_write_read_with_timeout(/* ... parámetros ... */) {
    int64_t start_time = k_uptime_get();
    int err = ssi_write_read(/* ... */);
    int64_t elapsed = k_uptime_get() - start_time;
    
    if (elapsed > SPI_TRANSACTION_TIMEOUT_MS) {
        LOG_ERR("SPI transaction timeout: %lld ms", elapsed);
        return -ETIMEDOUT;
    }
    
    return err;
}
```

## Implementación Inmediata Recomendada

**Cambio Mínimo de Mayor Impacto**:
1. Cambiar `k_msleep(1)` a `k_msleep(50)` en `icm45_init()`
2. Reducir `MAX_COMM_ERRORS` de 100 a 10
3. Agregar llamada a `ssi_recover_bus()` en el manejador de timeout

## Testing

Después de implementar, monitorear:
1. Logs de "Communication error"
2. Logs de "Sensor interrupt timeout"
3. Contador `consecutive_comm_errors`
4. Tiempo de uptime antes de fallo

## Notas Adicionales

- **USE_IMU_WAKE_UP**: No es la causa del problema, pero puede exacerbar problemas de power management
- El ICM-45686 es más sensible a timing que el ICM-42688
- Considerar agregar capacitor de desacople adicional en la línea de alimentación del IMU
- Verificar integridad de señales SPI con osciloscopio si el problema persiste
