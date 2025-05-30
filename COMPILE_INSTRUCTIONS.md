# Instrucciones de Compilación para SlimeVR-Tracker-nRF

> **Nota:** En este documento, `<PROJECT_PATH>` se refiere al directorio donde clonaste este proyecto, y `<WORKSPACE_PATH>` se refiere al directorio padre donde se encuentra el workspace de Zephyr (normalmente el directorio padre del proyecto).

## Dependencias Instaladas

### ✅ Herramientas del Sistema
- ✅ Homebrew
- ✅ CMake (4.0.2)
- ✅ Ninja (1.12.1)
- ✅ gperf (3.3)
- ✅ ccache (4.11.3_1)
- ✅ Device Tree Compiler (dtc 1.7.2)
- ✅ wget (1.25.0)

### ✅ Python y Entorno Virtual
- ✅ Python 3.13.3
- ✅ Entorno virtual en: `<PROJECT_PATH>/.venv`
- ✅ West (1.4.0) - herramienta de gestión de Zephyr
- ✅ Ninja para Python

### ✅ Zephyr SDK
- ✅ Zephyr SDK 0.17.0 instalado en: `~/zephyr-sdk-0.17.0`
- ✅ Toolchain ARM configurado
- ✅ CMake package registrado

## ✅ Entorno de Desarrollo Completamente Configurado

### Zephyr SDK Instalado
- ✅ Zephyr SDK 0.16.8 instalado en: `~/zephyr-sdk-0.16.8`
- ✅ Todas las dependencias de sistema instaladas
- ✅ Variables de entorno configuradas
- ✅ VS Code configurado con extensiones para desarrollo embedded

### Script de Compilación Automático
Se ha creado un script de conveniencia para compilar el proyecto:

```bash
# Hacer el script ejecutable (solo la primera vez)
chmod +x build.sh

# Compilar con placa por defecto (nrf52840dk_nrf52840)
./build.sh

# Compilar para una placa específica
./build.sh xiao_ble_nrf52840

# Limpiar, compilar y programar
./build.sh -c -f nrf52840dongle_nrf52840

# Ver todas las opciones disponibles
./build.sh --help
```

### Configuración de VS Code
Se han configurado las siguientes características en VS Code:

1. **Tasks (Ctrl+Shift+P > Tasks: Run Task)**:
   - Build SlimeVR Tracker
   - Build for specific board
   - Clean Build
   - Flash to Device
   - Update West Dependencies

2. **Launch Configurations** para debugging:
   - Debug SlimeVR (J-Link)
   - Debug SlimeVR (OpenOCD)
   - Attach to Running Target

3. **IntelliSense** configurado para:
   - Autocompletado de código C/C++
   - Navegación de símbolos
   - Detección de errores en tiempo real

### Problemas Conocidos y Soluciones

#### Error de Compilación: "retained_mem_nrf_ram_ctrl.c"
Si encuentras errores relacionados con `retained_mem_nrf_ram_ctrl.c`, esto puede deberse a incompatibilidades entre versiones de Zephyr. Soluciones:

1. **Verificar versión de Zephyr**: `west --version`
2. **Limpiar build**: `rm -rf build` o usar `./build.sh -c`
3. **Actualizar dependencias**: `west update`

### Extensiones de VS Code Instaladas
- ✅ C/C++ IntelliSense
- ✅ CMake Tools
- ✅ CMake Language Support
- ✅ Cortex-Debug (para debugging ARM)
- ✅ Serial Monitor

## Pasos para Compilar

### 1. Activar el Entorno Virtual
```bash
cd <PROJECT_PATH>
source .venv/bin/activate
```

### 2. Configurar el Workspace de Zephyr
```bash
cd <WORKSPACE_PATH>
source <PROJECT_PATH>/.venv/bin/activate
west zephyr-export
```

### 3. Instalar Dependencias de Python de Zephyr
```bash
cd <WORKSPACE_PATH>
pip install -r zephyr/scripts/requirements.txt
```

### 4. Compilar el Proyecto
Para compilar para una placa específica (ejemplo SuperMini I2C):

```bash
cd <WORKSPACE_PATH>
west build \
  --board supermini_uf2/nrf52840/i2c \
  --pristine=always SlimeVR-Tracker-nRF \
  --build-dir SlimeVR-Tracker-nRF/build \
  -- \
  -DNCS_TOOLCHAIN_VERSION=NONE \
  -DBOARD_ROOT=SlimeVR-Tracker-nRF
```

### Placas Disponibles
- `supermini_uf2/nrf52840/i2c` - SuperMini I2C Tracker
- `supermini_uf2/nrf52840/spi` - SuperMini SPI Tracker
- `xiao_ble/nrf52840` - XIAO BLE Tracker
- `xiao_ble/nrf52840/sense` - XIAO BLE Sense Tracker
- `slimenrf_r3/nrf52840/uf2` - SlimeNRF R3 Tracker
- `slimevrmini_p1_uf2/nrf52833` - SlimeVR Mini P1 Tracker
- Y muchas más...

### 5. Buscar el Firmware Compilado
El archivo resultante estará en:
```
SlimeVR-Tracker-nRF/build/SlimeVR-Tracker-nRF/zephyr/zephyr.uf2
```
(o `.hex` dependiendo de la placa)

## Resolución de Problemas

### Si hay errores de CMake:
```bash
# Limpiar caché
rm -rf SlimeVR-Tracker-nRF/build
```

### Si faltan dependencias:
```bash
# Reinstalar dependencias de Python
pip install -r zephyr/scripts/requirements.txt
```

### Actualizar el SDK:
```bash
cd <WORKSPACE_PATH>
west update
```

## Estructura del Workspace
```
<WORKSPACE_PATH>/
├── SlimeVR-Tracker-nRF/          # Tu proyecto
├── zephyr/                       # Zephyr RTOS
├── nrf/                          # nRF Connect SDK
├── modules/                      # Módulos adicionales
└── .west/                        # Configuración de West
```
