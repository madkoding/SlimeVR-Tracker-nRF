# SuperMini nRF52840 - Configuración SPI

Este documento detalla la configuración SPI específica para la tarjeta SuperMini nRF52840.

## Información General SPI

La SuperMini nRF52840 utiliza el bus SPI3 del nRF52840 para comunicación de alta velocidad con IMUs. Esta configuración ofrece mejor rendimiento que I2C.

## Configuración de Pines SPI

### Mapeo de Pines

| Función | Pin GPIO | Pin Físico | Descripción |
|---------|----------|------------|-------------|
| **MISO** | P0.24 | GPIO 24 | Master In, Slave Out (IMU → SuperMini) |
| **MOSI** | P0.06 | GPIO 6 | Master Out, Slave In (SuperMini → IMU) |
| **SCK** | P0.08 | GPIO 8 | Serial Clock (reloj SPI) |
| **CS** | P0.22 | GPIO 22 | Chip Select (Active Low) |
| **INT** | P0.17 | GPIO 17 | Pin de interrupción del IMU |

### Especificaciones Técnicas SPI

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| **Bus SPI** | SPI3 | Tercer bus SPI del nRF52840 |
| **Velocidad Máxima** | 24MHz | Máxima velocidad soportada |
| **Velocidad Típica** | 8MHz | Velocidad recomendada para IMUs |
| **Modo SPI** | Mode 3 | CPOL=1, CPHA=1 |
| **Word Size** | 8 bits | Tamaño de palabra estándar |
| **Voltaje** | 3.3V | Nivel lógico |

### Pines de Control Adicionales

| Función | Pin GPIO | Configuración | Descripción |
|---------|----------|---------------|-------------|
| Botón SW0 | P0.11 | Input, Pull-up, Active Low | Botón de control habilitado |
| LED Principal | P0.15 | PWM, Open Source | LED de estado |

## Tabla de Conexiones SPI

### Conexión con IMU

| Pin SuperMini | Pin IMU | Cable | Función | Notas |
|---------------|---------|-------|---------|--------|
| **P0.24** | SDO/MISO | Azul | Datos SPI (IMU→MCU) | Salida del IMU |
| **P0.06** | SDA/MOSI | Verde | Datos SPI (MCU→IMU) | Compartido con I2C SDA |
| **P0.08** | SCL/SCK | Amarillo | Reloj SPI | Compartido con I2C SCL |
| **P0.22** | CS/SS | Blanco | Chip Select | Active Low, único para SPI |
| **P0.17** | INT1/INT | Púrpura | Interrupción | Opcional, para motion detection |
| **3.3V** | VCC/VDD | Rojo | Alimentación | 3.3V ±5% |
| **GND** | GND | Negro | Tierra | Referencia común |

### Diagrama de Conexión

```
SuperMini nRF52840          IMU (ejemplo BMI270)
┌─────────────────┐        ┌─────────────────┐
│                 │        │                 │
│ P0.24 (MISO) ───┼────────┼─── SDO          │
│ P0.06 (MOSI) ───┼────────┼─── SDA          │
│ P0.08 (SCK) ────┼────────┼─── SCL          │
│ P0.22 (CS) ─────┼────────┼─── CS           │
│ P0.17 ──────────┼────────┼─── INT1         │
│ 3.3V ───────────┼────────┼─── VCC          │
│ GND ────────────┼────────┼─── GND          │
│                 │        │                 │
└─────────────────┘        └─────────────────┘
```

## Configuración DTS (Device Tree)

### Archivo: supermini_uf2_spi.dts

```dts
// Configuración SPI para SuperMini
&spi3 {
    status = "okay";
    
    pinctrl-0 = <&spi3_default>;
    pinctrl-1 = <&spi3_sleep>;
    pinctrl-names = "default", "sleep";
    
    cs-gpios = <&gpio0 22 GPIO_ACTIVE_LOW>;
};

// Configuración de pines
&pinctrl {
    spi3_default: spi3_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 6)>,
                    <NRF_PSEL(SPIM_MISO, 0, 24)>;
            nordic,drive-mode = <NRF_DRIVE_H0H1>;
        };
    };
    
    spi3_sleep: spi3_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 6)>,
                    <NRF_PSEL(SPIM_MISO, 0, 24)>;
            low-power-enable;
        };
    };
};
```

## Modos SPI Soportados

### Configuración de Modos

| Modo | CPOL | CPHA | Descripción | Compatible |
|------|------|------|-------------|------------|
| **Mode 0** | 0 | 0 | Reloj bajo en idle, datos en flanco ascendente | BMI160, LSM6DS3 |
| **Mode 1** | 0 | 1 | Reloj bajo en idle, datos en flanco descendente | Menos común |
| **Mode 2** | 1 | 0 | Reloj alto en idle, datos en flanco descendente | Menos común |
| **Mode 3** | 1 | 1 | Reloj alto en idle, datos en flanco ascendente | **BMI270, ICM20948** |

### Configuración para Diferentes IMUs

```dts
// Para BMI270 (Mode 3)
&spi3 {
    status = "okay";
    clock-frequency = <8000000>;  // 8MHz
    
    // Mode 3: CPOL=1, CPHA=1
    spi-cpol;
    spi-cpha;
};

// Para BMI160 (Mode 0)
&spi3 {
    status = "okay";
    clock-frequency = <8000000>;  // 8MHz
    
    // Mode 0: CPOL=0, CPHA=0 (por defecto)
};
```

## Velocidades SPI Recomendadas

### Por Tipo de IMU

| IMU | Velocidad Máxima | Velocidad Recomendada | Notas |
|-----|------------------|----------------------|--------|
| **BMI160** | 10MHz | 8MHz | Clásico, bien soportado |
| **BMI270** | 10MHz | 8MHz | Successor del BMI160 |
| **ICM20948** | 7MHz | 6MHz | Magnetómetro incluido |
| **ICM42688** | 24MHz | 16MHz | Alto rendimiento |
| **LSM6DS3** | 10MHz | 8MHz | ST Microelectronics |
| **LSM6DSO** | 10MHz | 8MHz | Successor del LSM6DS3 |
| **LSM6DSV** | 10MHz | 8MHz | Última generación ST |

### Configuración de Velocidad

```dts
// Configuración conservadora (8MHz)
&spi3 {
    clock-frequency = <8000000>;
};

// Configuración de alto rendimiento (16MHz)
&spi3 {
    clock-frequency = <16000000>;
};

// Configuración máxima (24MHz - solo para ICM42688)
&spi3 {
    clock-frequency = <24000000>;
};
```

## Compilación y Uso

### Comando de Compilación

```bash
west build -b supermini_uf2_spi
```

### Verificación con Logs

```bash
west build -b supermini_uf2_spi -- -DCONFIG_LOG_LEVEL_DBG=y -DCONFIG_SPI_LOG_LEVEL_DBG=y
```

## Troubleshooting SPI

### Problemas Comunes

| Problema | Síntoma | Solución Probable |
|----------|---------|-------------------|
| **IMU no detectado** | No response en comunicación | Verificar conexiones CS y alimentación |
| **Datos erróneos** | Valores incorrectos | Verificar modo SPI (CPOL/CPHA) |
| **Comunicación inestable** | Fallos intermitentes | Reducir velocidad SPI |
| **CS no funciona** | Dispositivo siempre activo | Verificar configuración GPIO CS |

### Pasos de Diagnóstico

1. **Verificar Conexiones SPI**
   ```
   SuperMini P0.24 ↔ IMU SDO/MISO
   SuperMini P0.06 ↔ IMU SDA/MOSI  
   SuperMini P0.08 ↔ IMU SCL/SCK
   SuperMini P0.22 ↔ IMU CS/SS
   ```

2. **Verificar Voltajes**
   - Todas las líneas en idle deben estar a 3.3V o 0V según configuración
   - CS debe estar en HIGH (3.3V) cuando no se usa

3. **Verificar Modo SPI**
   ```dts
   // Para debug, usar velocidad baja
   &spi3 {
       clock-frequency = <1000000>;  // 1MHz para debug
   };
   ```

4. **Test de Comunicación**
   ```bash
   # Habilitar logs SPI detallados
   CONFIG_SPI_LOG_LEVEL_DBG=y
   CONFIG_GPIO_LOG_LEVEL_DBG=y
   ```

### Herramientas de Debug

```dts
// Configuración de debug con timeouts largos
&spi3 {
    status = "okay";
    clock-frequency = <1000000>;  // Velocidad lenta para debug
    
    // Habilitar timeouts largos para debug
    timeout-ms = <5000>;
};
```

## Configuración Avanzada

### Optimización de Rendimiento

```dts
&spi3 {
    clock-frequency = <16000000>;  // 16MHz para alto rendimiento
    
    // Configuración optimizada para DMA
    nordic,drive-mode = <NRF_DRIVE_H0H1>;
    
    // Buffer grande para transferencias múltiples
    dma-maxcnt = <65535>;
};
```

### Configuración de Bajo Consumo

```dts
&spi3 {
    // Configuración para bajo consumo
    pinctrl-names = "default", "sleep";
    
    // Velocidad reducida para menor consumo
    clock-frequency = <4000000>;  // 4MHz
};
```

### Configuración Multi-dispositivo

```dts
&spi3 {
    status = "okay";
    
    // Múltiples CS para varios IMUs
    cs-gpios = <&gpio0 22 GPIO_ACTIVE_LOW>,  // IMU 1
               <&gpio0 23 GPIO_ACTIVE_LOW>;  // IMU 2
};
```

## Comparación I2C vs SPI

### Rendimiento

| Aspecto | I2C | SPI | Ganador |
|---------|-----|-----|---------|
| **Velocidad** | 400kHz | 8-24MHz | **SPI** |
| **Latencia** | Mayor overhead | Menor overhead | **SPI** |
| **Ancho de banda** | ~50kB/s | ~1-3MB/s | **SPI** |
| **Jitter** | Mayor | Menor | **SPI** |

### Complejidad

| Aspecto | I2C | SPI | Mejor para |
|---------|-----|-----|------------|
| **Conexiones** | 2 líneas | 4 líneas | **I2C** (simple) |
| **Pull-ups** | Requeridos | No requeridos | **SPI** |
| **Configuración** | Simple | Moderada | **I2C** (principiantes) |
| **Debug** | Más fácil | Más complejo | **I2C** |

### Cuándo Usar SPI

✅ **Usar SPI cuando:**
- Necesitas alta velocidad (>400kHz)
- Latencia baja es crítica
- Tienes problemas de ruido en I2C
- Usas sampling rates altos (>100Hz)
- Necesitas timing preciso

❌ **Usar I2C cuando:**
- Eres principiante
- Quieres conexiones simples
- No necesitas alta velocidad
- Tienes múltiples dispositivos
- Prefieres configuración simple

## Referencias

- **[README_SUPERMINI_I2C.md](README_SUPERMINI_I2C.md)**: Configuración I2C alternativa
- **[README_SUPERMINI_nRF52840.md](README_SUPERMINI_nRF52840.md)**: Documentación principal
- **nRF52840 SPI Specifications**: Documentación técnica Nordic
- **Zephyr SPI API**: Documentación de la API SPI de Zephyr
