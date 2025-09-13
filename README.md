PID Balance con IMU, ESCs y LCD

Este proyecto implementa un sistema de control PID discreto para estabilizar un sistema con dos motores controlados mediante ESCs (Electronic Speed Controllers). El control se basa en la lectura de una IMU BNO080 y la sintonización dinámica de parámetros PID a través de potenciómetros. Además, se incluye un botón de emergencia (E-Stop) y una pantalla LCD para visualización en tiempo real.

🛠️ Hardware utilizado

Arduino (compatible con Servo, Wire e interrupciones)

IMU BNO080 (sensor de orientación y movimiento)

2 ESCs con motores eléctricos

Pantalla LCD 16x2 I²C (dirección 0x27 o 0x3F)

Potenciómetros (x4) para:

Kp (ganancia proporcional)

Ki (ganancia integral)

Kd (ganancia derivativa)

Referencia de ángulo (θ_ref)

Botón de emergencia (E-Stop) conectado a interrupción externa

⚙️ Funcionamiento

Inicialización de ESCs

Secuencia de armado prolongada para garantizar seguridad antes de habilitar los motores.

Lectura de entradas

Los potenciómetros ajustan dinámicamente Kp, Ki, Kd y el ángulo de referencia (θ_ref).

La IMU entrega el ángulo de roll (θ) en grados.

Control PID discreto

Calcula el error e = θ_ref - θ.

Obtiene los términos up, ui, ud.

Genera la señal de control u.

Accionamiento de motores

Se genera la señal PWM directamente en microsegundos (1000–2000 μs).

Los dos motores reciben señales diferentes para balancear el sistema.

Visualización en tiempo real

En el LCD: valores de Kp, Ki, Kd y error.

En el Serial Monitor: debug completo con referencia, ángulo, error, ganancias PID, señal u y señales enviadas a ESCs.

Botón de emergencia

Si se activa, los motores se apagan inmediatamente (1000 μs) y el sistema muestra en LCD y Serial la alerta de emergencia.

📊 Variables principales

Kp, Ki, Kd → Ganancias PID (ajustables con potenciómetros).

theta → Ángulo actual del sistema (grados).

theta_ref → Ángulo de referencia deseado.

e → Error entre referencia y medición.

u → Señal de control.

🚨 Seguridad

El E-Stop es prioritario: detiene inmediatamente los motores.

Límites de seguridad en señales ESC (constrain(1000, 2000 μs)).

▶️ Uso

Conectar todos los componentes según el esquema.

Encender el sistema sin mover los ESCs durante el armado inicial.

Ajustar Kp, Ki, Kd con los potenciómetros hasta obtener estabilidad.

Observar resultados en LCD o Serial.

Usar el botón de emergencia en caso de comportamiento inesperado.

🔮 Posibles mejoras

Implementar anti-windup para la acción integral.

Guardar los valores de PID ajustados en EEPROM.

Incorporar ajuste de referencia en grados (actualmente fijo en 0).

Mostrar en LCD también los valores de los motores (esc1, esc2).
