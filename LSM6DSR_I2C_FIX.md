# Corrección del problema de activación del LSM6DSR con I2C

## Problema resuelto

El acelerómetro LSM6DSR no se activaba correctamente cuando se usaba con interfaz I2C a 100kHz. La investigación reveló que el problema **NO** era la frecuencia I2C (100kHz es óptima para LSM6DSR), sino una configuración incorrecta de la interfaz en el código.

## Causa raíz identificada

Los drivers de sensores LSM6D forzaban la configuración SPI incluso cuando el hardware estaba conectado por I2C:

```c
// Código problemático anterior
sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
```

## Cambios implementados

### 1. Corrección de drivers de sensores LSM6D

**Archivos modificados:**
- `/src/sensor/imu/LSM6DSO.c`
- `/src/sensor/imu/LSM6DSV.c` 
- `/src/sensor/imu/LSM6DSM.c`

**Cambio aplicado:**
```c
// Nuevo código adaptativo
int spi_ret = sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
if (spi_ret == 0) {
    LOG_INF("Using SPI interface for LSM6D sensor");
} else {
    LOG_INF("SPI interface not available, using I2C interface for LSM6D sensor");
}
```

### 2. Corrección de direcciones I2C en archivos DTS

**Archivos modificados:**
- `/boards/nordic/supermini_uf2/supermini_uf2_i2c.dts`
- `/boards/nordic/supermini_uf2/supermini_uf2.dts`

**Cambio aplicado:**
```dts
// Dirección I2C específica para LSM6DSR
imu: imu@6b {// LSM6DSR I2C address is 0x6B (when SDO is pulled high, 0x6A when low)
    compatible = "i2c-device";
    label = "imu";
    reg = <0x6b>;
};
```

## Validación de la solución

### Configuración I2C confirmada como correcta:
- **Frecuencia**: 100kHz (I2C_BITRATE_STANDARD) ✅
- **Dirección**: 0x6B para LSM6DSR ✅
- **Configuración ULP**: CTRL5 = 0x00 para LSM6DSR ✅

### Flujo de interfaz corregido:
1. El sistema escanea SPI primero, luego I2C
2. Se registra la interfaz correcta (SPI o I2C)
3. Los drivers ahora verifican qué interfaz está disponible
4. Se configura solo la interfaz activa

## Resultado esperado

Con estos cambios:
- El LSM6DSR debería activarse correctamente con I2C a 100kHz
- Los logs mostrarán qué interfaz se está usando
- El sistema funcionará tanto con hardware SPI como I2C
- Se mantiene la compatibilidad con todos los sensores LSM6D

## Notas técnicas

- La frecuencia I2C de 100kHz es óptima para LSM6DSR (balance entre velocidad y consumo)
- La dirección 0x6B asume que el pin SDO está conectado a VDD (estado alto)
- Si SDO está conectado a GND, usar dirección 0x6A en su lugar
- El sistema de detección automática de direcciones sigue funcionando como respaldo
