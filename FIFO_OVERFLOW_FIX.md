# Fix para Congelamiento Durante Movimiento Rápido / Cobertura del Tracker

## Problema Identificado

El tracker dejaba de responder cuando:
1. **Movimiento muy rápido**: El sensor IMU genera datos a una tasa muy alta
2. **Cobertura del sensor**: Bloquear físicamente el sensor puede causar interferencia

### Causa Raíz

El problema ocurría por **desbordamiento del FIFO** del sensor IMU:

- El FIFO (First In First Out) del sensor almacena temporalmente las lecturas del acelerómetro y giroscopio
- Durante movimiento rápido, el FIFO se llena más rápido de lo que el firmware puede procesarlo
- Cuando el FIFO se desborda completamente:
  - Las interrupciones pueden dejar de dispararse correctamente
  - Los datos se corrompen
  - El sistema entra en un estado de espera infinita

## Soluciones Implementadas

### 1. Protección contra Desbordamiento del FIFO

**Ubicación**: `src/sensor/sensor.c` líneas ~790-830

```c
// Detecta cuando se leen casi el máximo de paquetes posibles
// Si ocurre 3 veces consecutivas, drena completamente el FIFO
```

**Funcionamiento**:
- Monitorea la cantidad de paquetes leídos del FIFO
- Si se detecta que está cerca del máximo del buffer (≥ max - 5 paquetes)
- Después de 3 lecturas consecutivas saturadas, hace una lectura adicional para vaciar completamente el FIFO
- Previene que el FIFO permanezca lleno y cause bloqueos

### 2. Detección de Alta Tasa de Datos Sostenida

**Ubicación**: `src/sensor/sensor.c` líneas ~802-825

```c
#define HIGH_PACKET_THRESHOLD 80  // Paquetes que indican tasa muy alta
#define MAX_HIGH_PACKET_ITERATIONS 10  // Iteraciones antes de actuar
```

**Funcionamiento**:
- Detecta cuando se leen consistentemente ≥80 paquetes por lectura
- Si esto ocurre durante 10 iteraciones consecutivas:
  - Drena agresivamente el FIFO con 3 lecturas de 512 bytes
  - Registra advertencias en el log
  - Resetea contadores

### 3. Mejora en el Manejo de Interrupciones

**Ubicación**: `src/sensor/sensor.c` líneas ~586-598, ~1149-1165

**Características**:
- Variable `interrupt_pending` volátil para detectar interrupciones durante ráfagas
- Verifica si hubo interrupciones pendientes antes de declarar timeout
- Resetea contadores de timeout cuando se detectan interrupciones válidas
- Previene falsos positivos de timeout durante movimiento rápido

### 4. Recuperación Mejorada de Errores de Paquetes

**Ubicación**: `src/sensor/sensor.c` líneas ~945-970

**Funcionamiento**:
- Si se detectan paquetes pero no se procesan correctamente (datos corruptos)
- Después de 3 errores consecutivos:
  - Intenta vaciar el FIFO completo
  - Descarta datos potencialmente corruptos
  - Resetea el contador de errores
- Evita llegar al umbral de 10 errores que causa reinicio del sistema

## Logs de Diagnóstico

Los siguientes mensajes en el log serial indican que las protecciones están funcionando:

### Operación Normal
```
No hay mensajes especiales - el sistema funciona silenciosamente
```

### Protecciones Activadas

#### FIFO Overflow
```
[WRN] FIFO overflow detected (85 packets), clearing FIFO to prevent freeze
[WRN] Cleared 23 extra packets from FIFO overflow
```

#### Alta Tasa de Datos
```
[WRN] Sustained high data rate detected (92 packets, 10 iterations)
[WRN] This may indicate very fast movement or sensor covering - draining FIFO
[WRN] Drained 47 extra packets to clear sustained overload
```

#### Recuperación de Paquetes Corruptos
```
[WRN] No packets processed (42 packets read)
[WRN] Multiple packet processing errors, attempting FIFO flush
[WRN] Discarded 15 packets during recovery
```

#### Interrupciones durante Ráfaga
```
(No hay log visible - se maneja internamente reseteando timeouts)
```

## Probando la Solución

### Test 1: Movimiento Rápido
1. Enciende el tracker y conéctalo al servidor SlimeVR
2. Mueve el tracker muy rápidamente en todas direcciones
3. Verifica que sigue respondiendo
4. Revisa el log serial para ver si aparecen advertencias de FIFO overflow

### Test 2: Cobertura del Sensor
1. Con el tracker funcionando normalmente
2. Cubre completamente el tracker con la mano (bloquea el sensor)
3. Muévelo mientras está cubierto
4. Verifica que el tracker no se congela
5. Revisa el log para advertencias de alta tasa de datos

### Test 3: Estrés Prolongado
1. Deja el tracker en un lugar donde se mueva constantemente (ej: ventilador)
2. Déjalo por 30 minutos
3. Verifica que sigue respondiendo
4. Presiona el botón para confirmar que el sistema responde

## Parámetros Ajustables

Si necesitas ajustar la sensibilidad de las protecciones:

### `HIGH_PACKET_THRESHOLD` (línea ~105)
- **Por defecto**: 80 paquetes
- **Aumentar**: Si ves muchas advertencias durante uso normal
- **Disminuir**: Si el problema persiste con movimiento muy rápido

### `MAX_HIGH_PACKET_ITERATIONS` (línea ~106)
- **Por defecto**: 10 iteraciones
- **Aumentar**: Tolerar más tiempo de alta tasa antes de actuar
- **Disminuir**: Actuar más rápido ante movimiento rápido

### `MAX_SENSOR_TIMEOUTS` (línea ~103)
- **Por defecto**: 20 timeouts (1 segundo)
- **No se recomienda modificar** - es el último recurso antes de reconfigurar el hardware

## Limitaciones

1. **Pérdida temporal de precisión**: Durante el drenado agresivo del FIFO, algunos datos se descartan
   - Esto es preferible a un congelamiento total
   - La fusión de sensores se recupera rápidamente

2. **Advertencias en el log**: Movimiento muy rápido generará advertencias
   - Esto es normal y esperado
   - Indica que las protecciones están funcionando

3. **No soluciona problemas de hardware**: Si el sensor tiene fallas físicas, el problema puede persistir

## Archivos Modificados

- `src/sensor/sensor.c`: Todas las mejoras principales

## Próximos Pasos

Si el problema persiste después de estos cambios:

1. **Verifica el hardware**:
   - Conexiones I2C/SPI flojas
   - Interferencia electromagnética
   - Sensor defectuoso

2. **Ajusta parámetros**:
   - Reduce la frecuencia de muestreo del sensor (ODR)
   - Aumenta el intervalo de actualización del loop principal

3. **Revisa el log completo**:
   - Busca errores de comunicación I2C/SPI
   - Verifica si hay errores de memoria
   - Comprueba el estado de la batería

## Compilación

Después de aplicar estos cambios:

```bash
west build -b tu_board -p
west flash
```

O si usas el script de build:
```bash
./build.sh tu_board
```
