# Tarjeta SuperMini nRF52840 - SlimeVR Tracker

Este documento describe la configuración para la tarjeta SuperMini nRF52840 utilizada como tracker SlimeVR.

## Información del Dispositivo

- **SoC**: nRF52840 (ARM Cortex-M4F @ 64MHz)
- **Flash**: 1MB
- **RAM**: 256KB
- **Conectividad**: Bluetooth 5.0, Thread, Zigbee, ANT
- **USB**: Compatible con USB Device Stack
- **Bootloader**: Adafruit UF2 (offset 0x1000)
- **Factor de Forma**: Ultra compacto (similar a XIAO)

## Configuración de Pines del Sistema

### Pines de Control

| Función | Pin GPIO | Configuración | Descripción |
|---------|----------|---------------|-------------|
| LED Principal | P0.15 | PWM, Open Source | LED de estado principal |
| Botón SW0 | P0.11 | Input, Pull-up, Active Low | Botón de control habilitado |
| Interrupt IMU | P0.17 | Input | Pin de interrupción del IMU |
| Clock Externo | P0.20 | Open Drain (opcional) | Clock externo para cristal oscilador |

### Medición de Batería

| Función | Pin/Canal | Configuración | Descripción |
|---------|-----------|---------------|-------------|
| ADC Batería | ADC Canal 12 | VDDHDIV5 | Medición directa del voltaje de alimentación |
| Divisor | 1:5 | Hardware interno | Divisor de voltaje interno del nRF52840 |

## Configuraciones de Comunicación con IMU

Esta tarjeta soporta comunicación con IMUs mediante dos protocolos:

### I2C (Recomendado para principiantes)
- **Archivo de configuración**: [README_SUPERMINI_I2C.md](README_SUPERMINI_I2C.md)
- **Ventajas**: Fácil conexión, solo 2 líneas de datos, múltiples dispositivos
- **Comando**: `west build -b supermini_uf2_i2c`

### SPI (Mayor rendimiento)
- **Archivo de configuración**: [README_SUPERMINI_SPI.md](README_SUPERMINI_SPI.md)
- **Ventajas**: Mayor velocidad, menor latencia, mejor para alta frecuencia
- **Comando**: `west build -b supermini_uf2_spi`

### Configuración Automática
- **Comando**: `west build -b supermini_uf2`
- Detecta automáticamente IMUs en I2C y SPI

## IMUs Optimizados para Máxima Precisión

| IMU | Dirección I2C | SPI | Precisión | Notas |
|-----|---------------|-----|-----------|-------|
| **LSM6DSR** ⭐ | 0x6A/0x6B | ✓ | **Superior** | Recomendado para tracking de precisión, drift ±1°/h |
| **ICM45686** | 0x68/0x69 | ✓ | Muy Buena | Bajo consumo, drift ±2°/h |

⭐ **CONFIGURACIONES OPTIMIZADAS**:
- **I2C a 100kHz**: Máxima precisión y duración de batería ([ver análisis técnico](README_SUPERMINI_I2C.md#-análisis-técnico-por-qué-100khz-es-superior))
- **Buffers reducidos**: 128 bytes para menor consumo
- **LSM6DSR**: Superior estabilidad térmica y menor drift

### 🔬 ¿Por qué 100kHz en I2C?

**No es solo batería, es precisión mejorada:**
- **📉 33% menos drift** térmico
- **🔇 33% menos ruido** en lecturas 
- **🔋 37% más duración** de batería
- **📊 Mejor estabilidad** en sesiones largas

👉 **[Ver análisis técnico completo](README_SUPERMINI_I2C.md#-análisis-técnico-por-qué-100khz-es-superior)**

### IMUs Adicionales Compatibles

| IMU | Dirección I2C | SPI | Notas |
|-----|---------------|-----|-------|
| **BMI160** | 0x68/0x69 | ✓ | Sensor clásico, bien soportado |
| **BMI270** | 0x68/0x69 | ✓ | Successor del BMI160, mejor precisión |
| **ICM20948** | 0x68/0x69 | ✓ | 9-DOF con magnetómetro |
| **ICM42688** | 0x68/0x69 | ✓ | Alto rendimiento, bajo consumo |
| **LSM6DS3** | 0x6A/0x6B | ✓ | ST Microelectronics |
| **LSM6DSO** | 0x6A/0x6B | ✓ | Successor del LSM6DS3 |
| **LSM6DSV** | 0x6A/0x6B | ✓ | Última generación ST |

## Compilación del Firmware

### Configuración I2C
```bash
west build -b supermini_uf2_i2c
```

### Configuración SPI
```bash
west build -b supermini_uf2_spi
```

### Configuración Automática (I2C + SPI)
```bash
west build -b supermini_uf2
```

## Programación del Firmware

1. **Conecta la SuperMini** por USB
2. **Activa el modo bootloader** (doble clic en reset o botón específico)
3. **Aparecerá** como dispositivo de almacenamiento USB
4. **Arrastra** el archivo `.uf2` generado al dispositivo
5. **Se reiniciará** automáticamente con el nuevo firmware

## Notas Importantes

- **Voltaje de Operación**: 3.3V - Verifica compatibilidad del IMU
- **Pull-ups I2C**: Se requieren resistencias externas de 4.7kΩ
- **Bootloader UF2**: Facilita la programación sin herramientas adicionales
- **Consumo**: Optimizado para uso con batería
- **Tamaño**: Factor de forma ultra compacto

## Archivos de Configuración Detallados

- **[README_SUPERMINI_I2C.md](README_SUPERMINI_I2C.md)**: Configuración completa I2C con ejemplos y troubleshooting
- **[README_SUPERMINI_SPI.md](README_SUPERMINI_SPI.md)**: Configuración completa SPI con especificaciones técnicas

## Soporte y Documentación

Para configuraciones específicas, troubleshooting y ejemplos avanzados, consulta los archivos de configuración individuales según el protocolo que uses.