# Adaptación al chasis Ackermann

Estado (11 nov 2026): **en desarrollo**. El firmware del P4 compila (ESP-IDF 5.5.4) y el simulador ejecuta la misión
completa con este chasis, pero **nada está probado en el coche real**. No se debe usar para mover el robot hasta completar
y comprobar las tareas pendientes.

Se ha elegido un [perfil provisional de simulación](simulator/PERFIL_PROVISIONAL.md)
de 22 × 14 cm y batalla 14 cm. Son suposiciones editables, no medidas del robot.

## Hardware indicado por el equipo

- Un motor trasero JGB37-520 de 12 V y 600 rpm (el que hay hoy; el definitivo será de 1000 rpm), con encoder integrado A/B en
  cuadratura y conectado a un eje común para las ruedas traseras.
- Puente H L298N; servo MG90S para las ruedas directrices delanteras.
- Batería Gens Ace 3S, 11,1 V nominales, 2200 mAh, 25C; 12,6 V cargada.
- Placas y sensores a través de un convertidor a 5 V. IMU MPU-6050; cámara
  S3 existente. LiDAR WitMotion D6 o D500: confirmar modelo y protocolo.

## Cambios y límites

- El P4 manda velocidad a un solo canal del L298N y el ángulo efectivo de
  dirección al servo. El signo positivo corresponde a girar a la derecha.
- El servo utiliza pulsos medidos de izquierda, centro y derecha y ángulos
  efectivos máximos separados. La tracción no funciona si falta su calibración.
- `WRO_ACKERMANN_HW_CONFIGURED` queda apagado por defecto y bloquea los GPIO de
  tracción y servo incluso si un `sdkconfig` anterior tenía motores habilitados.
  Los pines y la calibración deben verificarse antes de activar el bloqueo.
- El P4 dispone de lectura A/B x4 por PCNT y odometría de un solo eje más IMU.
  Valor inicial propuesto por el equipo: 440 cuentas por vuelta de salida
  (11 pulsos por vuelta de motor por canal × 4 flancos × 10 de reducción).
  Se supone conexión 1:1 a las ruedas; confirmar la reducción exacta y los
  pulsos en la unidad real. La escala se cambia con
  `WRO_AXLE_ENCODER_COUNTS_PER_REV` (por vuelta del motorreductor), la pareja de
  engranajes con `WRO_MOTOR_GEAR_TEETH` y `WRO_DIFF_GEAR_TEETH`, los pines A/B con
  `WRO_AXLE_ENCODER_A_GPIO` y `WRO_AXLE_ENCODER_B_GPIO`, y el signo con
  `WRO_AXLE_ENCODER_INVERT`. La odometría solo se activa con
  `WRO_AXLE_ENCODER_CALIBRATED`, tras confirmar escala, signo y diámetro.
  Sin esa confirmación usa velocidad ordenada × tiempo, no calibrada. Un
  encoder de eje común no permite detectar patinaje comparando ruedas.
- Se desactivan pivote, desplazamiento lateral y giro de observación. Existe un
  planificador Ackermann experimental para entrar y salir mediante arcos de
  avance y reversa. La geometría ideal se comprobó con pruebas pequeñas; la
  integración con sensores y su comportamiento real aún no están validados.
  En el firmware físico, el parqueo sigue desactivado por defecto.
- El simulador selecciona el modelo bicicleta y desactiva esas mismas opciones.
  Su geometría y su modelo de encoder aún son provisionales. El WASM versionado
  y los resultados históricos no validan esta adaptación.
- Parámetros del servo y pines en `LIDAR System/p4_firmware/main/Kconfig.projbuild`.
  Los `sdkconfig` previos pueden conservar valores del vehículo anterior: revisar
  todos antes de usarlos. El bloqueo nuevo permanece desactivado por defecto.

## Datos pendientes

1. Cableado completo y GPIO, incluidos servo y motor; alimentación y masa común.
2. Geometría: dimensiones, batalla entre ejes, ancho, diámetro de rueda, posiciones
   de sensores y ángulos efectivos de dirección en ambos sentidos.
3. Pulsos del servo con ruedas rectas y en los límites mecánicos seguros, y sentido.
4. Encoder: confirmar pines A/B, sentido y número real de cuentas por vuelta de
   las ruedas; comprobar que no haya transmisión adicional entre salida y eje.
5. Velocidad real frente a PWM, frenado y respuesta de dirección, medidos en pista.
6. Adaptar parqueo, odometría, simulador y pruebas al chasis; luego compilar y
   validar antes de probar gradualmente en hardware.

## Referencia de pista

El [reglamento oficial Future Engineers 2026](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
establece un área interior de pista de 3000 × 3000 mm y bloques de 50 × 50 ×
100 mm. En el reto de obstáculos, el rojo indica pasar por la derecha y el verde
por la izquierda; el cajón tiene 20 cm de ancho y longitud de 1,5 veces la del
vehículo. Las reglas de la competencia interna pueden ser distintas.

La condición oficial de aparcamiento paralelo compara las distancias a la pared
de dos ruedas del mismo lado: la diferencia no debe superar 2 cm. No significa
que ambas ruedas deban estar a menos de 2 cm de la pared. El juez aproxima esa
diferencia con la batalla y el ángulo; queda por validar el modelo completo.
