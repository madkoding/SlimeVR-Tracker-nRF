#!/bin/bash

# Script de compilación para SlimeVR-Tracker-nRF
# Autor: Configuración automática de Zephyr RTOS

set -e # Salir si hay algún error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 SlimeVR-Tracker-nRF Build Script${NC}"
echo -e "${BLUE}=====================================${NC}"

# Función para mostrar ayuda
show_help() {
    echo "Uso: $0 [OPCIONES] [PLACA]"
    echo ""
    echo "OPCIONES:"
    echo "  -h, --help              Mostrar esta ayuda"
    echo "  -c, --clean             Limpiar build antes de compilar"
    echo "  -f, --flash             Programar el dispositivo después de compilar"
    echo "  -q, --qemu              Ejecutar en emulador QEMU (solo para placas compatibles)"
    echo ""
    echo "PLACAS disponibles:"
    echo "  nrf52840dk_nrf52840     (por defecto)"
    echo "  nrf52840dongle_nrf52840"
    echo "  nrf52dk_nrf52832"
    echo "  xiao_ble_nrf52840"
    echo "  xiao_ble_nrf52840_sense"
    echo "  supermini_uf2_nrf52840"
    echo "  slimenrf_r3_nrf52840_uf2"
    echo "  slimevrmini_p1_uf2_nrf52833"
    echo "  slimevrmini_p2_uf2_nrf52833"
    echo "  slimevrmini_p3r6_uf2_nrf52833"
    echo "  slimevrmini_p3r7_uf2_nrf52833"
    echo ""
    echo "Ejemplos:"
    echo "  $0                                    # Compilar con placa por defecto"
    echo "  $0 nrf52840dongle_nrf52840           # Compilar para dongle nRF52840"
    echo "  $0 -c -f xiao_ble_nrf52840           # Limpiar, compilar y programar XIAO BLE"
}

# Variables por defecto
BOARD="nrf52840dk_nrf52840"
CLEAN_BUILD=false
FLASH_DEVICE=false
RUN_QEMU=false

# Parsear argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
    -h | --help)
        show_help
        exit 0
        ;;
    -c | --clean)
        CLEAN_BUILD=true
        shift
        ;;
    -f | --flash)
        FLASH_DEVICE=true
        shift
        ;;
    -q | --qemu)
        RUN_QEMU=true
        BOARD="qemu_x86"
        shift
        ;;
    -*)
        echo -e "${RED}❌ Opción desconocida: $1${NC}"
        show_help
        exit 1
        ;;
    *)
        BOARD=$1
        shift
        ;;
    esac
done

echo -e "${YELLOW}📋 Configuración:${NC}"
echo -e "   Placa objetivo: ${GREEN}$BOARD${NC}"
echo -e "   Limpiar build: ${GREEN}$CLEAN_BUILD${NC}"
echo -e "   Programar dispositivo: ${GREEN}$FLASH_DEVICE${NC}"
echo -e "   Ejecutar en QEMU: ${GREEN}$RUN_QEMU${NC}"
echo ""

# Verificar que estamos en el directorio correcto
if [[ ! -f "west.yml" ]]; then
    echo -e "${RED}❌ Error: No se encontró west.yml. Asegúrate de estar en el directorio del proyecto.${NC}"
    exit 1
fi

# Activar entorno virtual si existe
if [[ -d ".venv" ]]; then
    echo -e "${YELLOW}🐍 Activando entorno virtual...${NC}"
    source .venv/bin/activate
fi

# Verificar que west esté disponible
if ! command -v west &>/dev/null; then
    echo -e "${RED}❌ Error: west no está instalado o no está en el PATH.${NC}"
    exit 1
fi

# Verificar variables de entorno de Zephyr
if [[ -z "$ZEPHYR_SDK_INSTALL_DIR" ]]; then
    export ZEPHYR_SDK_INSTALL_DIR="~/zephyr-sdk-0.16.8"
    export ZEPHYR_TOOLCHAIN_VARIANT="zephyr"
    echo -e "${YELLOW}⚙️  Configurando variables de entorno de Zephyr...${NC}"
fi

# Limpiar build si se solicita
if [[ "$CLEAN_BUILD" == true ]]; then
    echo -e "${YELLOW}🧹 Limpiando build anterior...${NC}"
    if [[ -d "build" ]]; then
        rm -rf build
    fi
fi

# Compilar el proyecto
echo -e "${YELLOW}🔨 Compilando para $BOARD...${NC}"
if west build -p auto -b "$BOARD"; then
    echo -e "${GREEN}✅ Compilación exitosa!${NC}"

    # Mostrar información del binario
    if [[ -f "build/zephyr/zephyr.elf" ]]; then
        echo -e "${BLUE}📊 Información del binario:${NC}"
        size build/zephyr/zephyr.elf
    fi

    # Programar el dispositivo si se solicita
    if [[ "$FLASH_DEVICE" == true && "$RUN_QEMU" == false ]]; then
        echo -e "${YELLOW}📱 Programando dispositivo...${NC}"
        if west flash; then
            echo -e "${GREEN}✅ Programación exitosa!${NC}"
        else
            echo -e "${RED}❌ Error al programar el dispositivo${NC}"
            exit 1
        fi
    fi

    # Ejecutar en QEMU si se solicita
    if [[ "$RUN_QEMU" == true ]]; then
        echo -e "${YELLOW}🖥️  Ejecutando en QEMU...${NC}"
        echo -e "${BLUE}💡 Para salir de QEMU presiona: Ctrl+A, luego X${NC}"
        cd build && ninja run
    fi

else
    echo -e "${RED}❌ Error en la compilación${NC}"
    exit 1
fi

echo -e "${GREEN}🎉 ¡Proceso completado!${NC}"
