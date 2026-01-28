# RESUMEN EJECUTIVO - ADAPTACIÓN DE EVOLUCIÓN DIFERENCIAL PARA PÉNDULO INVERTIDO

## 📦 Contenido del Paquete

He adaptado tu trabajo anterior de optimización PID con Evolución Diferencial para el problema del **péndulo invertido en un carro**. Este paquete incluye:

### Archivos Principales:
1. **`setup_and_diagnostic.m`** ⭐ **EMPIEZA AQUÍ**
   - Script de diagnóstico que verifica tu modelo
   - Te ayuda a identificar qué bloques ajustar
   - Ejecuta una simulación de prueba

2. **`pendulum_DE_optimization.m`** 
   - Script principal de optimización
   - Configura parámetros DE y ejecuta la búsqueda
   - Genera gráficas de resultados

3. **Funciones de Coste** (elige la que mejor se adapte):
   - `pendulum_cost.m` - Versión completa con múltiples objetivos
   - `pendulum_cost_simple.m` - Versión simplificada y fácil de adaptar
   - `pendulum_cost_dual_loop.m` - Para estructura de doble lazo

4. **`devec3.m`**
   - Algoritmo de Evolución Diferencial (el que ya conoces)

5. **`README_PENDULUM_DE.md`**
   - Documentación completa con instrucciones detalladas

---

## 🚀 Quick Start (Inicio Rápido)

### Paso 1: Ejecuta el Diagnóstico
```matlab
setup_and_diagnostic
```
Este script te dirá:
- Si tu modelo está bien configurado
- Qué variables necesitas exportar
- Si puede simular correctamente

### Paso 2: Adapta la Función de Coste
Abre `pendulum_cost_simple.m` y:
- Cambia los nombres de las variables según tu modelo
- Ajusta cómo se configuran los parámetros
- Verifica los pesos de la función de coste

### Paso 3: Ejecuta la Optimización
```matlab
pendulum_DE_optimization
```

---

## 🎯 Diferencias Clave vs Tu Trabajo Anterior

| Aspecto | PID Simple (anterior) | Péndulo Invertido (nuevo) |
|---------|----------------------|---------------------------|
| **Parámetros** | 3 (Kp, Ki, Kd) | 2-6 (depende del controlador) |
| **Controladores** | 1 PID simple | 2 en cascada (PD + Estado) |
| **Sistema** | Estable | Inestable (péndulo) |
| **Objetivos** | 1 (seguimiento) | Múltiples (seguimiento + estabilidad) |
| **Función de coste** | Error simple | Ponderación múltiple |
| **Complejidad** | Baja | Media-Alta |

---

## ⚙️ Configuración Recomendada

### Para Pruebas Iniciales:
```matlab
D = 2;              % Solo Kp y Kd del PD de posición
XVmin = [0, 0];
XVmax = [15, 8];
NP = 40;            % Población pequeña
itermax = 20;       % Pocas iteraciones
```

### Para Optimización Final:
```matlab
D = 4;              % Incluir controlador de estado
XVmin = [0, 0, 0, 0];
XVmax = [15, 8, 100, 100];
NP = 80;            % Población grande
itermax = 100;      # Muchas iteraciones
```

---

## 📊 Función de Coste

La clave del éxito está en los pesos:

```matlab
F = w1 * error_posicion +      % w1 = 1.0
    w2 * error_angulo +        % w2 = 50.0 ⭐ MUY IMPORTANTE
    w3 * esfuerzo_control +    % w3 = 0.01
    w4 * oscilaciones +        % w4 = 5.0
    w5 * settling_time +       % w5 = 10.0
    w6 * sobrepaso;            % w6 = 15.0
```

**Principio clave**: El peso del ángulo (w2) debe ser ~50x mayor que el de posición porque **mantener el péndulo vertical es crítico**.

---

## 🔧 Adaptación Necesaria

Debes adaptar **principalmente 2 cosas**:

### 1. Cómo se configuran los parámetros
En `pendulum_cost_simple.m`, línea ~30:

```matlab
% Opción A: Variables del workspace (RECOMENDADO)
assignin('base', 'Kp_pos', Kp_pos);
assignin('base', 'Kd_pos', Kd_pos);

% Opción B: set_param directo
% set_param('rct_pendulum/Position_Controller', 'P', num2str(Kp_pos));
```

### 2. Qué señales se leen
En `pendulum_cost_simple.m`, línea ~40:

```matlab
% Lee las variables que exportaste con "To Workspace"
pos_ref = evalin('base', 'TU_NOMBRE_VARIABLE_REF');
pos_actual = evalin('base', 'TU_NOMBRE_VARIABLE_POS');
angulo = evalin('base', 'TU_NOMBRE_VARIABLE_ANGULO');
```

---

## ⚠️ Problemas Comunes

### "Model not found"
- Verifica que `rct_pendulum.slx` está en el path de MATLAB
- Usa `addpath('ruta/al/modelo')`

### "Undefined variable"
- Añade bloques "To Workspace" en tu modelo Simulink
- Verifica que los nombres coinciden con el código

### El péndulo se cae siempre
- **Aumenta el peso w2** (error angular) a 100 o más
- Reduce el rango de búsqueda para forzar ganancias más altas
- Verifica que el ángulo inicial es 0° (vertical)

### La optimización no mejora
- Reduce el rango de búsqueda (hazlo más estrecho)
- Aumenta la población (NP = 80-100)
- Cambia la estrategia (prueba strategy = 2 o 6)

---

## 📈 Resultados Esperados

Según el ejemplo de MATLAB con `systune`, los valores óptimos son aproximadamente:
- **Kp_pos** ≈ 6.11
- **Kd_pos** ≈ 2.19
- **Controlador de ángulo**: Polos en ±14.52, ganancia ~1639

Tu optimización con DE debería encontrar valores similares o mejores.

---

## 🎓 Contexto del Problema

El péndulo invertido es un **sistema de control no lineal clásico**:
- Estado de equilibrio **inestable** (como balancear un palo en la mano)
- Requiere **control activo continuo** para mantener estabilidad
- Problema de **múltiples objetivos**:
  1. Mover el carro a una posición deseada
  2. Mantener el péndulo vertical
  3. Minimizar oscilaciones y sobrepaso
  4. Usar fuerza de control razonable

La estructura de **doble lazo** es típica:
- **Lazo interno rápido**: Estabiliza el péndulo (crítico)
- **Lazo externo lento**: Controla la posición del carro

---

## 💡 Tips Finales

1. **Empieza simple**: Optimiza primero solo 2 parámetros
2. **Verifica manualmente**: Prueba valores antes de optimizar
3. **Usa systune como referencia**: Los valores del ejemplo son buenos
4. **Itera gradualmente**: Haz varias optimizaciones cortas
5. **Guarda resultados**: Cada run guarda un .mat automáticamente
6. **Ajusta pesos**: Si algo no funciona, revisa los pesos de la función de coste

---

## 📞 Siguiente Acción

1. ✅ Ejecuta `setup_and_diagnostic.m`
2. ✅ Lee el output y verifica que todo está OK
3. ✅ Adapta `pendulum_cost_simple.m` según tu modelo
4. ✅ Ejecuta `pendulum_DE_optimization.m` con itermax=10 (prueba)
5. ✅ Si funciona, aumenta población e iteraciones
6. ✅ Compara resultados con systune

---

## 📚 Recursos Adicionales

- **README_PENDULUM_DE.md**: Documentación completa y detallada
- **Ejemplo de MATLAB**: Control de péndulo invertido con systune
- **Tu trabajo anterior**: Estructura similar, aplicada a PID simple

---

¡Buena suerte con la optimización! Si tienes dudas sobre cómo adaptar algo específico, pregúntame. 🚀
