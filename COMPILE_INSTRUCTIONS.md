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
