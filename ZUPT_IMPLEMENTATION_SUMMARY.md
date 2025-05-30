# ZUPT (Zero Velocity Update) and Kalman Filter Implementation

## Resumen de la Implementación

Se ha implementado exitosamente el algoritmo ZUPT (Zero Velocity Update) y la optimización del filtro de Kalman en el framework MotionSense del SlimeVR Tracker, basándose en el análisis previo de la implementación NXP-ISSDK.

## Características Implementadas

### 1. **Detección de Velocidad Cero (ZUPT)**
- **Umbrales de Detección:**
  - Giroscopio: 1.5°/s (configurable)
  - Acelerómetro: 0.15g desviación de la gravedad nominal
  - Tiempo mínimo de reposo: 2.0s

- **Algoritmo de Detección:**
  - Monitoreo continuo de la norma de velocidad angular
  - Verificación de estabilidad del acelerómetro
  - Timer de confirmación para evitar falsos positivos

### 2. **Estimación de Bias del Giroscopio**
- **Parámetros Basados en NXP:**
  - Incertidumbre inicial: 1.0°/s
  - Incertidumbre durante reposo: 0.1°/s
  - Máximo bias permitido: ±3.0°/s
  - Tiempo de olvido: 200s

- **Algoritmo de Estimación:**
  - Actualización conservadora durante períodos de reposo
  - Factor de olvido gradual durante movimiento
  - Clipping automático para evitar valores extremos

### 3. **Filtro de Kalman Optimizado**
- **Estado del Filtro (9D):**
  - Orientación: Quaternion (4 elementos → 3 grados de libertad)
  - Bias del giroscopio: 3 componentes
  - Aceleración lineal: 3 componentes

- **Parámetros de Ruido:**
  - Varianza del proceso (giroscopio): 2E2 (deg/s)²
  - Varianza del proceso (bias): 2E-2 (deg/s)²
  - Varianza de medición (acelerómetro): 1.2E-3 g²

### 4. **Funciones de Interfaz Implementadas**

```c
// Inicialización y configuración
void sensorfusion_init(float g_time, float a_time, float m_time);
void sensorfusion_load(const void* data);
void sensorfusion_save(void* data);

// Actualización de sensores
void sensorfusion_update_gyro(float* g, float time);
void sensorfusion_update_accel(float* a, float time);
void sensorfusion_update_mag(float* m, float time);
void sensorfusion_update(float* g, float* a, float* m, float time);

// Gestión de bias
void sensorfusion_get_gyro_bias(float* g_off);
void sensorfusion_set_gyro_bias(float* g_off);

// Verificación de integridad
void sensorfusion_update_gyro_sanity(float* g, float* m);
int sensorfusion_get_gyro_sanity(void);

// Salidas de fusión
void sensorfusion_get_lin_a(float* lin_a);
void sensorfusion_get_quat(float* q);
```

## Algoritmos Implementados

### Predicción del Filtro de Kalman
1. **Corrección de Bias:** Resta el bias estimado de las mediciones del giroscopio
2. **Integración de Quaternion:** Actualiza la orientación usando velocidad angular corregida
3. **Actualización de Covarianza:** Agrega ruido del proceso según el tiempo transcurrido

### Actualización con Acelerómetro
1. **Vector de Gravedad Esperado:** Calcula la gravedad esperada en el marco del cuerpo
2. **Innovación:** Diferencia entre aceleración medida y gravedad esperada
3. **Corrección de Orientación:** Aplica corrección basada en la innovación
4. **Aceleración Lineal:** Calcula aceleración lineal removiendo la gravedad

### Detección ZUPT
1. **Verificación de Umbrales:** Comprueba si las mediciones están dentro de los límites de reposo
2. **Timer de Confirmación:** Requiere tiempo mínimo de estabilidad
3. **Actualización de Bias:** Durante el reposo, actualiza conservadoramente el bias

## Parámetros de Configuración

### Constantes ZUPT
```c
#define ZUPT_REST_TH_GYR_DEG          1.5f      // Umbral de reposo giroscopio
#define ZUPT_REST_TH_ACC_G            0.15f     // Umbral de reposo acelerómetro  
#define ZUPT_REST_MIN_TIME_S          2.0f      // Tiempo mínimo de reposo
#define ZUPT_BIAS_SIGMA_INIT_DEG      1.0f      // Incertidumbre inicial de bias
#define ZUPT_BIAS_SIGMA_REST_DEG      0.1f      // Incertidumbre durante reposo
#define ZUPT_BIAS_CLIP_DEG            3.0f      // Límite máximo de bias
#define ZUPT_BIAS_FORGETTING_TIME_S   200.0f    // Tiempo de olvido de bias
```

### Parámetros del Filtro de Kalman
```c
#define KALMAN_Q_GYRO_VAR            (2E2f)     // Varianza de ruido del giroscopio
#define KALMAN_Q_BIAS_VAR            (2E-2f)    // Varianza de ruido del bias
#define KALMAN_R_ACCEL_VAR           (1.2E-3f)  // Varianza de ruido del acelerómetro
```

## Ventajas de la Implementación

1. **Estabilidad Mejorada:** ZUPT reduce significativamente la deriva del giroscopio
2. **Calibración Automática:** Estimación continua y adaptativa del bias
3. **Robustez:** Verificaciones de cordura y clipping de valores extremos
4. **Eficiencia:** Algoritmos optimizados basados en matemáticas de quaternions
5. **Compatibilidad:** Interfaz completa compatible con el framework SlimeVR

## Siguientes Pasos

1. **Pruebas de Integración:** Verificar funcionamiento con hardware real
2. **Optimización de Parámetros:** Ajustar umbrales según tipo específico de IMU
3. **Integración Magnetómetro:** Añadir corrección de rumbo usando magnetómetro
4. **Análisis de Rendimiento:** Medir latencia y uso de CPU

## Referencias

- Implementación basada en análisis de NXP-ISSDK SensorFusion
- Parámetros calibrados según especificaciones de VQF y frameworks existentes
- Algoritmos de quaternion optimizados para sistemas embebidos

---

**Estado:** ✅ Implementación completa y lista para pruebas
**Fecha:** 29 de Mayo de 2025
**Framework:** SlimeVR MotionSense con ZUPT y Kalman optimizado
