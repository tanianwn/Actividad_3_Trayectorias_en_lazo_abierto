# Actividad 3.1 (Trayectorias en lazo abierto)
## Descripción de la Actividad
El objetivo de esta actividad consiste en implementar código en MATLAB para generar diversas trayectorias en un plano 2D utilizando control en lazo abierto. A partir de ejemplos dados en la actividad que se muestran, así mismo se deben deducir las secuencias exactas de velocidades lineales ($u$) y angulares ($w$) a lo largo del tiempo necesarias para que un robot móvil reproduzca dichas rutas de manera autónoma.
<p align="center">
  <img src="https://github.com/user-attachments/assets/a5830a92-4fa3-4727-9a6d-4d92e88dcc9b" width="600" alt="Robot Cartesiano 3GDL"/>
</p>

## ¿Qué es el control en Lazo Abierto?
El control en **lazo abierto** u open-loop significa que el sistema ejecuta una secuencia de comandos pre-programados, como velocidades y tiempos, sin recibir retroalimentación (feedback) de su entorno. 

A diferencia del lazo cerrado, aquí el robot no usa sensores para verificar su posición real. Simplemente se le ordenan las indicaciones de velocidad linear y angular, y el sistema asume que la instrucción se cumplió perfectamente. Es un método sencillo, pero susceptible a errores de precisión en el mundo real.

---
## Explicación de la Implementación en MATLAB

La simulación de las trayectorias se fundamenta en la asignación de velocidad para cada instante de tiempo, eludiendo el uso de coordenadas absolutas de destino. La implementación se basa en los siguientes puntos clave:

### 1. El Manejo del Tiempo (`tf` y `ts`)
El movimiento se discretiza en pequeños pasos. 
* **`tf`:** Es el tiempo final o total de la simulación.
* **`ts` :** Es el tiempo de muestreo. Esto significa que en el codigo se evalúa y actualiza la posición del robot cada 0.1 segundos. 
> NOTA: Si actualizamos cada 0.1 segundos, significa que **1 segundo de simulación equivale a 10 muestras** (1 / 0.1 = 10).

### 2. Uso de `ones` y las velocidades
Para construir el path, definimos un vector de velocidad lineal **`u`** (que es avanzar) y un vector de velocidad angular **`w`** (que es girar). 

```matlab
% Ejemplo
u = [2 * ones(1,10)';   % Avanza a 2 m/s durante 1 segundo (10 muestras)
     0 * ones(1,10)'];  % Se detiene durante 1 segundo (10 muestras)

w = [0 * ones(1,10)';   % No gira mientras avanza
    -pi/2 * ones(1,10)']; % Gira a -90 grados/seg durante 1 segundo
```

Usamos `ones(1, 10)` para mantener una velocidad constante exactamente durante 1 segundo. Intercalando bloques donde el robot solo avanza (`u > 0`, `w = 0`) y bloques donde el robot solo gira en su propio eje (`u = 0`, `w > 0`), logramos trazar cualquier trayectoria paso a paso.

### 3. Modelo Cinemático: Integración de Euler

Una vez que se tienen los vectores de velocidades, el bucle `for` recorre cada instante de tiempo. Utilizando el método de Euler, se calcula la nueva posición y orientación del robot sumando la posición anterior más el desplazamiento generado por las velocidades actuales:

$$
\phi_{k+1} = \phi_k + w_k \cdot t_s
$$

$$
x_{k+1} = x_k + (u_k \cdot \cos(\phi_{k+1})) \cdot t_s
$$

$$
y_{k+1} = y_k + (u_k \cdot \sin(\phi_{k+1})) \cdot t_s
$$
