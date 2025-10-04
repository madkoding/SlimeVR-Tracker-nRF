# Resumen de Cambios - Fix para Tracker que Deja de Enviar Datos

## Fecha
4 de Octubre, 2025

## Problema
Un tracker dejaba de enviar datos después de errores o timeouts del IMU, pero seguía funcionando internamente (respondía al movimiento cuando se conectaba por USB).

## Causa Raíz
La variable `main_ok` se establecía en `false` tras errores críticos, y el código no tenía ningún mecanismo de recuperación automática. El thread del sensor continuaba corriendo pero saltaba toda la lógica de procesamiento dentro de `if (main_ok)`, creando un estado "zombie".

## Solución Implementada

### 1. Sistema de Recuperación Automática
**Archivo**: `src/sensor/sensor.c`

- Agregado tracking de intentos de recuperación
- Implementado ciclo de recuperación con cooldown de 5 segundos
- Hasta 3 intentos de reinicialización del sensor
- Reinicio automático del sistema como último recurso
- Reseteo de todos los contadores de error en recuperación exitosa

### 2. Recuperación Proactiva de Errores SPI/I2C
**Archivo**: `src/sensor/sensor.c`

- Chequeo proactivo de errores de comunicación antes de leer el FIFO
- Recuperación cuando se detectan 10 errores (50% del límite crítico)
- Previene acumulación de errores que llevarían a fallo completo
- Latencia mínima (10ms) durante recuperación

### 3. Fix de VQF
**Archivo**: `vqf-c/src/vqf.h`

- Agregado `#include <stdbool.h>` para resolver errores de compilación
- No relacionado con el problema principal, pero necesario para compilar

## Archivos Modificados

1. `src/sensor/sensor.c` - Sistema de recuperación y detección proactiva
2. `vqf-c/src/vqf.h` - Fix de include para stdbool
3. `SENSOR_RECOVERY_FIX.md` - Documentación completa del fix (NUEVO)

## Testing Requerido

1. **Test de Larga Duración**: Dejar tracker funcionando 24-48 horas
2. **Test de Cobertura**: Cubrir el tracker repetidamente para forzar timeouts
3. **Test de Interferencia**: Probar con fuentes de interferencia RF
4. **Test de Recuperación**: Verificar logs de recuperación automática

## Logs a Monitorear

### Recuperación Exitosa
```
[sensor] Sensor in error state, attempting recovery #1/3
[sensor] Re-initializing sensor...
[sensor] Sensor recovery successful!
```

### Recuperación Proactiva
```
[sensor] Detected 10 communication errors, attempting proactive recovery
[sensor] Proactive bus recovery completed
```

### Reinicio Necesario
```
[sensor] Maximum recovery attempts reached, requesting system reboot
```

## Compilación Exitosa

```
Memory region         Used Size  Region Size  %age Used
           FLASH:      228880 B       948 KB     23.58%
             RAM:       45696 B       252 KB     17.71%
```

## Branch
`freeze-fix`

## Siguiente Paso
Probar en hardware real y monitorear logs durante operación prolongada.
