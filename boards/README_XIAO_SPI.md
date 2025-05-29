# XIAO BLE nRF52840 - Configuración SPI

Este documento describe la configuración específica del protocolo SPI para la tarjeta XIAO BLE nRF52840 en el proyecto SlimeVR-Tracker-nRF.

## Configuración de Pines SPI

### Mapeo de Pines Principal

#### Tabla de Pines Disponibles XIAO BLE nRF52840

| Pin | Pin nRF52840 | GPIO | Función SPI | Estado | Descripción |
|-----|--------------|------|-------------|--------|-------------|
| **Pin 0** | P0.02 | GPIO 2 | Botón SW0 | Usado | Botón de control |
| **Pin 1** | P0.03 | GPIO 3 | **INT** | Usado | Interrupción del IMU |
| **Pin 2** | P0.28 | GPIO 28 | - | Disponible | GPIO general libre |
| **Pin 3** | P0.29 | GPIO 29 | - | Disponible | GPIO general libre |
| **Pin 4** | P0.04 | GPIO 4 | - | Disponible | GPIO general libre |
| **Pin 5** | P0.05 | GPIO 5 | **CS** | Usado | Chip Select SPI |
| **Pin 6** | P1.11 | GPIO 43 | - | Disponible | GPIO general libre |
| **Pin 7** | P1.14 | GPIO 46 | **MISO** | Usado | Master In, Slave Out |
| **Pin 8** | P1.13 | GPIO 45 | **SCK** | Usado | Serial Clock |
| **Pin 9** | P1.12 | GPIO 44 | - | Disponible | GPIO general libre |
| **Pin 10** | P1.15 | GPIO 47 | **MOSI** | Usado | Master Out, Slave In |
| **3V3** | - | - | Alimentación | Usado | Salida 3.3V regulada |
| **GND** | - | - | Tierra | Usado | Tierra común |
| **VBUS** | - | - | USB 5V | Disponible | Entrada 5V desde USB |

#### Resumen de Configuración SPI

| Función SPI | Pin | Pin nRF52840 | GPIO | Descripción |
|-------------|-----|--------------|------|-------------|
| **MOSI** | Pin 10 | P1.15 | GPIO 47 | Master Out, Slave In (datos MCU → IMU) |
| **MISO** | Pin 7 | P1.14 | GPIO 46 | Master In, Slave Out (datos IMU → MCU) |
| **SCK** | Pin 8 | P1.13 | GPIO 45 | Serial Clock (señal de reloj) |
| **CS** | Pin 5 | P0.05 | GPIO 5 | Chip Select (selección de dispositivo) |
| **INT** | Pin 1 | P0.03 | GPIO 3 | Interrupción del IMU (opcional) |

### Especificaciones Técnicas SPI

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| **Bus** | SPI1 | Bus SPI utilizado |
| **Velocidad** | 8MHz | Velocidad máxima recomendada |
| **Modo** | Mode 0 | CPOL=0, CPHA=0 |
| **Bits por palabra** | 8 bits | Tamaño de datos estándar |
| **Chip Select** | Active Low | CS bajo para activar dispositivo |
| **Endianness** | Big Endian | Orden de bytes para IMUs |

### Pines de Control Adicionales

| Función | Pin | Pin GPIO | Configuración | Descripción |
|---------|-----|----------|---------------|-------------|
| **Botón SW0** | **Pin 0** | **P0.02** | Input, Pull-up, Active Low | Botón de control habilitado |

**📌 Información del Botón SW0:**
- **Ubicación**: Pin 0 (P0.02 - GPIO 2) 
- **Función**: Control de sistema y reset
- **Configuración**: Activo bajo con pull-up interno
- **Acciones**: 1 presión = reinicio, 2 presiones = calibración, 3 presiones = reset pairing, 5+ segundos = factory reset

### Pines Disponibles para Expansión

Los siguientes pines están disponibles para funciones adicionales:

| Pin | Pin nRF52840 | Uso Sugerido |
|-----|--------------|--------------|
| **Pin 2** | P0.28 | GPIO general, sensor adicional |
| **Pin 3** | P0.29 | Control de LED, botón adicional |
| **Pin 4** | P0.04 | GPIO general, señal de control |
| **Pin 6** | P1.11 | GPIO general, señal de control |
| **Pin 9** | P1.12 | GPIO general, señal de control |

⚠️ **Nota**: Pin 4 (P0.04) también puede usarse como SCL para I2C secundario si se necesita.

## Conexiones del IMU - SPI

### Tabla de Conexiones

| Pin | Función | Pin IMU | Notas |
|-----|---------|---------|-------|
| Pin 10 (P1.15) | MOSI | SDA/MOSI | Datos MCU → IMU |
| Pin 7 (P1.14) | MISO | SDO/MISO | Datos IMU → MCU |
| Pin 8 (P1.13) | SCK | SCL/SCK | Reloj de comunicación |
| Pin 5 (P0.05) | CS | CS/SS | Chip Select (activo bajo) |
| Pin 1 (P0.03) | INT | INT1/INT | Interrupción (opcional) |
| 3V3 | VCC | VCC/VDD | Alimentación 3.3V |
| GND | GND | GND | Tierra común |

### Resistencias Pull-up/Pull-down

⚠️ **IMPORTANTE**: A diferencia de I2C, SPI no requiere resistencias pull-up externas. Sin embargo:

- **CS (Chip Select)**: Puede beneficiarse de una resistencia pull-up de 10kΩ a 3.3V para mantener el estado alto cuando no está activo
- **MISO**: Algunos IMUs pueden requerir pull-up de 10kΩ para operación confiable

## Configuración de IMUs SPI

### IMUs Soportados

| IMU | Pin de Dirección | Configuración CS | Notas |
|-----|------------------|------------------|-------|
| **BMI160** | SDO | CS bajo para SPI | SDO conectado a GND para dirección I2C 0x68 |
| **BMI270** | SDO | CS bajo para SPI | SDO conectado a GND para dirección I2C 0x68 |
| **ICM20948** | AD0 | CS bajo para SPI | AD0 conectado a GND para dirección I2C 0x68 |
| **ICM42688** | AP_AD0 | CS bajo para SPI | AP_AD0 conectado a GND para dirección I2C 0x68 |
| **ICM45686** | AP_AD0 | CS bajo para SPI | AP_AD0 conectado a GND para dirección I2C 0x68 |
| **LSM6DS3** | SDO/SA0 | CS bajo para SPI | SDO conectado a GND para dirección I2C 0x6A |
| **LSM6DSO** | SDO/SA0 | CS bajo para SPI | SDO conectado a GND para dirección I2C 0x6A |
| **LSM6DSV** | SDO/SA0 | CS bajo para SPI | SDO conectado a GND para dirección I2C 0x6A |

### Notas sobre Configuración de IMUs

1. **Pin de Dirección**: Cuando se usa SPI, el pin de dirección (SDO/AD0/SA0) debe conectarse a GND o VCC según el IMU
2. **Chip Select**: CS debe mantenerse alto (3.3V) cuando no se comunica con el IMU
3. **Velocidad**: Iniciar con velocidades bajas (1-2MHz) y aumentar gradualmente hasta encontrar la máxima estable

## Esquema de Conexión SPI

```
XIAO BLE nRF52840            IMU (ejemplo BMI270)
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│ Pin 10 (MOSI) ──┼─────────┼─── SDA/MOSI     │
│ Pin 7 (MISO) ───┼─────────┼─── SDO/MISO     │
│ Pin 8 (SCK) ────┼─────────┼─── SCL/SCK      │
│ Pin 5 (CS) ─────┼─────────┼─── CS/SS        │
│ Pin 1 (INT) ────┼─────────┼─── INT1         │
│ 3V3 ────────────┼─────────┼─── VCC          │
│ GND ────────────┼─────────┼─── GND          │
│                 │         │ GND ────────────┼─── SDO (para addr I2C)
│                 │         │                 │
└─────────────────┘         └─────────────────┘
    Opcional: Pull-up 10kΩ en CS y MISO
```

## Configuración del Archivo DTS

### Archivo SPI: `xiao_ble_spi.overlay`

⚠️ **IMPORTANTE**: Usa el overlay específico para SPI (`xiao_ble_spi.overlay`) que incluye la configuración completa para comunicación SPI, no el overlay base `xiao_ble.overlay` que está configurado para I2C.

#### Configuración Principal SPI

```dts
&pinctrl {
    spi1_default: spi1_default {
        group1 {
            psels = <NRF_PSEL(SPIM_MISO, 1, 14)>,    // Pin 7 (P1.14) - MISO
                    <NRF_PSEL(SPIM_MOSI, 1, 15)>,    // Pin 10 (P1.15) - MOSI
                    <NRF_PSEL(SPIM_SCK, 1, 13)>;     // Pin 8 (P1.13) - SCK
        };
    };

    spi1_sleep: spi1_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_MISO, 1, 14)>,
                    <NRF_PSEL(SPIM_MOSI, 1, 15)>,
                    <NRF_PSEL(SPIM_SCK, 1, 13)>;
            low-power-enable;
        };
    };
};

&spi1 {
    compatible = "nordic,nrf-spim";
    status = "okay";
    max-frequency = <DT_FREQ_M(8)>;
    pinctrl-0 = <&spi1_default>;
    pinctrl-1 = <&spi1_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 5 GPIO_ACTIVE_LOW>;    // Pin 5 (P0.05) - CS

    imu_spi: imu_spi@0 {
        compatible = "vnd,spi-device";
        spi-max-frequency = <DT_FREQ_M(8)>;
        label = "imu-spi";
        reg = <0>;
    };
};
```

#### Diferencias con el Overlay I2C

El archivo `xiao_ble_spi.overlay` incluye:

- ✅ **SPI habilitado**: Configuración completa de SPI1 con pines correctos
- ❌ **I2C deshabilitado**: I2C0 e I2C1 marcados como `status = "disabled"`
- ✅ **Pines SPI asignados**: MISO/MOSI/SCK/CS configurados apropiadamente
- ✅ **Configuración de energía**: Modos sleep para SPI

### Configuración de Interrupciones

```dts
/ {
    zephyr,user {
        int0-gpios = <&gpio0 3 0>;    // Pin 1 (P0.03) para interrupción del IMU
        // Nota: P0.17 referenciado en overlay I2C no está disponible externamente
    };
};
```

⚠️ **Nota sobre P0.17**: El pin P0.17 (GPIO 17) aparece en el overlay I2C base pero NO está disponible externamente en la XIAO BLE. El overlay SPI corrige esto usando solo pines accesibles.

## Comandos de Compilación

### Para configuración SPI (RECOMENDADO):
```bash
west build -b xiao_ble -- -DEXTRA_DTC_OVERLAY_FILE=xiao_ble_spi.overlay
```

### Método alternativo con overlay completo:
```bash
west build -b xiao_ble -- -DOVERLAY_CONFIG=boards/xiao_ble_spi.overlay
```

### Flash del firmware:
```bash
west flash
```

### Limpieza de build:
```bash
west build -t clean
```

⚠️ **IMPORTANTE**: 
- **NO uses** `xiao_ble.overlay` para SPI (está configurado para I2C)
- **USA siempre** `xiao_ble_spi.overlay` para comunicación SPI
- El overlay SPI deshabilita I2C automáticamente para evitar conflictos

## Troubleshooting SPI

### Problemas Comunes

1. **IMU no detectado**:
   - Verificar conexión CS (debe estar conectado y activo bajo)
   - Confirmar conexiones MOSI, MISO, SCK
   - Verificar alimentación 3.3V
   - Comprobar que el pin de dirección del IMU esté configurado correctamente

2. **Comunicación intermitente**:
   - Reducir velocidad SPI (de 8MHz a 1-2MHz)
   - Verificar integridad de las conexiones
   - Añadir pull-up de 10kΩ en CS y MISO
   - Revisar longitud de cables (máximo 15cm recomendado para SPI)

3. **Datos corruptos**:
   - Verificar modo SPI (CPOL/CPHA)
   - Comprobar orden de bits (MSB first)
   - Revisar timing de CS
   - Verificar niveles de voltaje (3.3V)

### Verificación de Conexiones

| Test | Método | Resultado Esperado |
|------|--------|--------------------|
| **Alimentación** | Multímetro VCC-GND | 3.3V ±0.1V |
| **CS Idle** | Multímetro CS-GND | 3.3V (estado alto) |
| **Continuidad** | Multímetro continuidad | Beep en todas las conexiones |
| **Resistencias** | Óhmetro | 10kΩ en pull-ups (si instaladas) |

### Análisis con Osciloscopio

1. **Señales a verificar**:
   - **SCK**: Pulsos de reloj limpios y estables
   - **CS**: Flancos limpios, activo bajo durante transferencias
   - **MOSI/MISO**: Datos válidos sincronizados con SCK

2. **Parámetros importantes**:
   - **Frecuencia**: Máximo 8MHz para operación estable
   - **Duty Cycle**: ~50% en SCK
   - **Setup/Hold Time**: Cumplir especificaciones del IMU

## Configuración Avanzada

### Configuración de Velocidad Personalizada

```dts
&spi1 {
    max-frequency = <DT_FREQ_M(1)>;     // 1MHz para pruebas
    // max-frequency = <DT_FREQ_M(4)>;  // 4MHz estándar
    // max-frequency = <DT_FREQ_M(8)>;  // 8MHz máximo
};
```

### Configuración de Modo SPI

```dts
imu_spi: imu_spi@0 {
    compatible = "vnd,spi-device";
    spi-max-frequency = <DT_FREQ_M(8)>;
    spi-cpha;       // CPHA=1 (Mode 1 o 3)
    spi-cpol;       // CPOL=1 (Mode 2 o 3)
    // Sin flags = Mode 0 (CPOL=0, CPHA=0)
    label = "imu-spi";
    reg = <0>;
};
```

### Buffer Size Personalizado

```dts
&spi1 {
    // Para transferencias grandes de FIFO
    zephyr,spi-buf-size = <1024>;
};
```

## Notas Técnicas

1. **Bus SPI1**: La XIAO BLE utiliza el bus SPI1 para evitar conflictos con otras interfaces
2. **Velocidad**: 8MHz es el máximo recomendado para operación estable con cables de prototipo
3. **Modos**: La mayoría de IMUs soportan Mode 0 (CPOL=0, CPHA=0)
4. **CS Control**: El CS es controlado automáticamente por el driver SPI
5. **DMA**: El controlador SPI utiliza DMA para transferencias eficientes
6. **Power Management**: SPI entra en modo de bajo consumo automáticamente

## Comparación I2C vs SPI

| Aspecto | I2C | SPI |
|---------|-----|-----|
| **Pines requeridos** | 2 + INT | 4 + INT |
| **Velocidad** | 400kHz | 8MHz |
| **Complejidad** | Simple | Moderada |
| **Pull-ups** | Requeridas (4.7kΩ) | Opcionales (10kΩ) |
| **Múltiples dispositivos** | Sí (direcciones) | Sí (múltiples CS) |
| **Distancia** | Hasta 30cm | Hasta 15cm |
| **Consumo** | Menor | Ligeramente mayor |
| **Throughput** | ~40KB/s | ~800KB/s |

### Recomendaciones de Uso

- **Usar I2C cuando**:
  - Se necesita simplicidad
  - Se conectan múltiples sensores
  - La velocidad no es crítica
  - Se prefieren menos cables

- **Usar SPI cuando**:
  - Se necesita alta velocidad
  - Se requiere máximo throughput
  - Se usa un solo IMU por bus
  - Se necesita comunicación determinista
