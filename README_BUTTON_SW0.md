# 🔘 Botón SW0 - Guía Completa de Acciones

Este documento detalla todas las acciones y funcionalidades del botón SW0 en el SlimeVR Tracker nRF.

## 📋 Configuración del Botón

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| **Pin GPIO** | P0.11 (SuperMini) / P0.02 (XIAO BLE) | Pin físico del botón |
| **Configuración** | Input, Pull-up, Active Low | Configuración eléctrica |
| **Debounce** | 50ms automático | Filtro anti-rebote |
| **Deep Sleep** | Wake-up habilitado | Despierta desde modo deep sleep |

## 🎯 Tabla de Acciones del Botón SW0

### Acciones por Número de Presiones

| Presiones | Acción | Descripción Técnica | Estado LED | Uso Típico |
|-----------|--------|-------------------|------------|------------|
| **1** | `sys_request_system_reboot()` | Reinicia completamente el sistema | 🔄 Parpadeo | Reinicio rápido del tracker |
| **2** | `sensor_request_calibration()` | Inicia calibración del sensor IMU | 🟡 Amarillo | Calibrar orientación del IMU |
| **3** | `esb_reset_pair()` | Resetea emparejamiento ESB | 🔵 Azul | Olvidar dispositivo emparejado |
| **4** | Modo DFU | Entra en modo Device Firmware Update | 🟣 Púrpura | Actualización de firmware |
| **5** | Modo DFU | Entra en modo Device Firmware Update | 🟣 Púrpura | Actualización de firmware |

### Acciones por Tiempo de Presión

| Duración | Acción | Descripción Técnica | Estado LED | Uso Típico |
|----------|--------|-------------------|------------|------------|
| **1 segundo** | Activación de función | Confirma la acción de presiones múltiples | ✅ Verde | Confirmación de comando |
| **5+ segundos** | `esb_reset_pair()` | Reset completo de emparejamiento | 🔴 Rojo intermitente | Factory reset de conexión |

### Acciones Especiales durante el Encendido

| Contexto | Acción | Descripción Técnica | Resultado | Uso Típico |
|----------|--------|-------------------|-----------|------------|
| **Boot + Presión** | Apagado inmediato | Cancela el encendido y apaga | ⚫ LED apagado | Apagado de emergencia |
| **Deep Sleep** | Wake-up | Despierta desde modo de bajo consumo | 🟢 LED encendido | Activación desde sleep |

## 🔧 Detalles Técnicos de Implementación

### Configuración GPIO

```c
// Configuración del pin del botón
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

// Configuración de entrada con pull-up
gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);

// Configuración para wake-up desde deep sleep
gpio_pin_configure(button.port, button.pin, 
                   GPIO_INPUT | GPIO_PULL_UP | NRF_GPIO_PIN_SENSE_LOW);
```

### Lógica de Detección

```c
// Detección de presiones múltiples con timeout de 1 segundo
button_count++;
if (button_count >= 1 && button_count <= 5) {
    k_work_schedule(&button_timeout_work, K_SECONDS(1));
}

// Detección de presión larga (>5 segundos)
if (pressed_time >= 5000) {
    esb_reset_pair();  // Reset de emparejamiento
}
```

### Estados del Sistema

| Estado del Sistema | Comportamiento del Botón | Notas |
|-------------------|--------------------------|--------|
| **Normal Operation** | Todas las funciones activas | Funcionamiento completo |
| **Calibración** | Solo cancelación | Presión cancela calibración |
| **DFU Mode** | Sin respuesta | Botón inactivo en modo DFU |
| **Deep Sleep** | Solo wake-up | GPIO sense activado |
| **Boot Sequence** | Apagado inmediato | Solo durante primeros segundos |

## 📱 Guía de Uso Práctica

### 🔄 **Reinicio Rápido**
1. Presiona el botón **1 vez**
2. Espera 1 segundo
3. El tracker se reinicia automáticamente

### 🎯 **Calibrar IMU**
1. Presiona el botón **2 veces** rápidamente
2. Espera 1 segundo para confirmación
3. Coloca el tracker en posición de calibración
4. Sigue las instrucciones LED

### 🔗 **Resetear Emparejamiento**
**Método Rápido:**
1. Presiona el botón **3 veces** rápidamente
2. Espera confirmación LED azul

**Método Seguro:**
1. Mantén presionado el botón por **5+ segundos**
2. Observa LED rojo intermitente
3. Suelta cuando confirme

### 🔧 **Actualizar Firmware**
1. Presiona el botón **4 o 5 veces** rápidamente
2. Espera LED púrpura
3. El tracker entra en modo DFU
4. Conecta USB y actualiza firmware

### 🛌 **Despertar desde Sleep**
1. El tracker en deep sleep se ve apagado
2. Presiona el botón **1 vez**
3. El tracker despierta inmediatamente

### ⚡ **Apagado de Emergencia**
1. **Durante el encendido** (primeros 3 segundos)
2. Presiona y mantén el botón
3. El tracker se apaga inmediatamente

## 🚨 Troubleshooting

### Problemas Comunes

| Problema | Posible Causa | Solución |
|----------|---------------|----------|
| Botón no responde | Pin no configurado | Verificar conexión física |
| Múltiples acciones | Debounce insuficiente | Presionar más despacio |
| No wake-up | GPIO sense deshabilitado | Verificar configuración DTS |
| Apagado accidental | Presión durante boot | Esperar a que termine el boot |

### Verificación de Funcionamiento

```bash
# Compilar con logs de debug del botón
west build -b supermini_uf2_i2c -- -DCONFIG_LOG_LEVEL_DBG=y

# Ver logs del botón en tiempo real
west debug
# En GDB: monitor rtt start
```

### Configuración Alternativa

Si necesitas deshabilitar el botón:

```dts
// En tu archivo .overlay
&gpio0 {
    status = "disabled";
};

/ {
    aliases {
        /delete-property/ sw0;
    };
};
```

## 📊 Resumen de Funciones

### Por Frecuencia de Uso

| Función | Frecuencia | Importancia | Acceso |
|---------|------------|-------------|--------|
| **Reinicio** | Alta | ⭐⭐⭐ | 1 presión |
| **Calibración** | Media | ⭐⭐⭐ | 2 presiones |
| **Wake-up** | Alta | ⭐⭐⭐ | 1 presión en sleep |
| **Reset Pair** | Baja | ⭐⭐ | 3 presiones / 5s |
| **DFU Mode** | Muy Baja | ⭐ | 4-5 presiones |
| **Apagado** | Baja | ⭐⭐ | Presión en boot |

### Códigos de Estado LED

| Color LED | Significado | Acción Relacionada |
|-----------|-------------|-------------------|
| 🔄 **Parpadeo rápido** | Reiniciando | 1 presión |
| 🟡 **Amarillo fijo** | Calibrando | 2 presiones |
| 🔵 **Azul fijo** | Reseteando pair | 3 presiones |
| 🟣 **Púrpura fijo** | Modo DFU | 4-5 presiones |
| 🔴 **Rojo intermitente** | Reset largo | 5+ segundos |
| 🟢 **Verde** | Confirmación | Acción exitosa |
| ⚫ **Apagado** | Sistema off | Apagado/sleep |

## 🔗 Referencias

- **Código fuente**: `/src/system/system.c` (líneas 235-310)
- **Configuración DTS**: `/boards/*/supermini_uf2_i2c.dts`
- **API GPIO**: Zephyr GPIO API Documentation
- **Power Management**: `/src/system/power.c`

---

**📝 Nota**: Este documento se basa en el análisis del código fuente del SlimeVR-Tracker-nRF. Las funcionalidades pueden variar según la versión del firmware.
