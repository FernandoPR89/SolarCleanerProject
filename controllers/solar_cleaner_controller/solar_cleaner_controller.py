from controller import Robot
import serial

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# 1. Configuración de motores
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
brush_motor = robot.getDevice('brush_motor')

for motor in [left_motor, right_motor, brush_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# 2. Configuración de sensores infrarrojos
ds_left = robot.getDevice('ds_front_left')
ds_right = robot.getDevice('ds_front_right')
ds_left.enable(TIME_STEP)
ds_right.enable(TIME_STEP)

# 3. Configuración de Odometría
ps_left = robot.getDevice('left_sensor')
ps_right = robot.getDevice('right_sensor')
ps_left.enable(TIME_STEP)
ps_right.enable(TIME_STEP)

# Parámetros de Navegación
CRUISE_SPEED = 3.5
BRUSH_SPEED = 10.0
EDGE_THRESHOLD = 500.0

TARGET_90_DEG = 5.95  
TARGET_SHIFT = 6.0    

# 4. Parámetros de Batería (NUEVO)
MAX_BATTERY = 270.0
current_battery = MAX_BATTERY

# Consumo de energía por segundo
BRUSH_CONSUMPTION_RATE = 1.0  # El cepillo gasta mucha energía
MOTOR_CONSUMPTION_RATE = 0.5  # Motores de tracción (cada uno)
IDLE_CONSUMPTION_RATE = 0.1   # Consumo de la electrónica cuando está quieto

# --- 2. CONFIGURACIÓN DEL GEMELO DIGITAL ---
# Conectar con el ESP32-S3 físico
try:
    esp32 = serial.Serial('COM8', 115200, timeout=0.01)
    conexion_hardware = True
    print("[SISTEMA] Enlace HIL establecido en COM8.")
except:
    conexion_hardware = False
    print("[ADVERTENCIA] ESP32 no detectado en COM8. Simulando a ciegas.")

factor_suciedad = 0.0
UMBRAL_PERDIDA_KW = 1500.0 # Tolerancia antes de limpiar
prediccion_ideal_kw = 11426.35 # Valor por defecto si falla el ESP32

state = "IDLE"
turn_direction = 1
turn_counter = 0
initial_wheel_pos = 0.0
print_timer = 0

brush_motor.setVelocity(BRUSH_SPEED)

# Variable para imprimir el porcentaje de batería sin saturar la consola
print_timer = 0
print("--- INICIANDO SISTEMA CIBERFÍSICO ---")

while robot.step(TIME_STEP) != -1:
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()
    current_wheel_pos = ps_left.getValue()
    
    # --- 3. LECTURA SERIAL Y GEMELO DIGITAL ---
    if conexion_hardware and esp32.in_waiting > 0:
        linea = esp32.readline().decode('utf-8').strip()
        # Parsear la línea del ESP32 buscando la predicción (Ej: "=> Predicción: 11426.35 kW")
        if "Predicción:" in linea:
            try:
                # Extraer el valor numérico (ajustar índice según tu print en C++)
                partes = linea.split("Predicción: ")
                prediccion_ideal_kw = float(partes[1].split(" ")[0])
            except:
                pass 

    # Simulación física: El polvo cae constantemente
    factor_suciedad += 0.00005 

    # Matemática del desgaste
    energia_real = prediccion_ideal_kw * (1.0 - factor_suciedad)
    delta_P = prediccion_ideal_kw - energia_real

    # --- SIMULACIÓN DE BATERÍA ---
    # Convertimos los milisegundos del TIME_STEP a segundos (dt)
    dt = TIME_STEP / 1000.0 
    
    if state in ["IDLE", "STOP", "OUT_OF_BATTERY"]:
        current_battery -= IDLE_CONSUMPTION_RATE * dt
    else:
        # Consume energía del cepillo + 2 motores + electrónica base
        total_consumption = BRUSH_CONSUMPTION_RATE + (MOTOR_CONSUMPTION_RATE * 2) + IDLE_CONSUMPTION_RATE
        current_battery -= total_consumption * dt
        # Si el cepillo gira, limpiamos el polvo virtual
        factor_suciedad -= 0.002 
        if factor_suciedad < 0: factor_suciedad = 0.0

    if current_battery <= 0 and state != "OUT_OF_BATTERY":
        current_battery = 0
        state = "OUT_OF_BATTERY"
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        brush_motor.setVelocity(0.0)
        print("\n[ALERTA] ¡Batería agotada! El robot se ha apagado a mitad del trabajo.")
    
    # --- 4. MÁQUINA DE ESTADOS MODIFICADA ---
    if state == "IDLE":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        # El Disparador Ciberfísico
        if delta_P > UMBRAL_PERDIDA_KW and current_battery > 50.0:
            print(f"\n[ALERTA] Delta P crítico detectado: {delta_P:.2f} kW. Iniciando rutina Zig-Zag.")
            state = "FORWARD"
    
    # Imprimir telemetría cada 1 segundo para no saturar la consola
    print_timer += dt
    if print_timer >= 1.0 and state != "OUT_OF_BATTERY":
        if state == "IDLE":
            print(f"Monitor -> Ideal: {prediccion_ideal_kw:.1f} kW | Real: {energia_real:.1f} kW | Pérdida: {delta_P:.1f} kW (Umbral: {UMBRAL_PERDIDA_KW})")
        else:
            print(f"Limpiando... Batería restante: {current_battery:.1f}%")
        print_timer = 0
    # -----------------------------

    # --- MÁQUINA DE ESTADOS ---
    if state == "OUT_OF_BATTERY":
        pass # El robot se queda inerte

    elif state == "STOP":
        pass # Trabajo terminado, esperando recolección

    elif state == "FORWARD":
        left_motor.setVelocity(-CRUISE_SPEED)
        right_motor.setVelocity(-CRUISE_SPEED)
        
        if (left_val > EDGE_THRESHOLD) or (right_val > EDGE_THRESHOLD):
            state = "BACKWARD"
            turn_counter = int(1.0 / dt)

    elif state == "BACKWARD":
        left_motor.setVelocity(CRUISE_SPEED * 0.6)
        right_motor.setVelocity(CRUISE_SPEED * 0.6)
        turn_counter -= 1
        if turn_counter <= 0:
            state = "TURN_1"
            initial_wheel_pos = current_wheel_pos

    elif state == "TURN_1":
        left_motor.setVelocity(-CRUISE_SPEED * 0.8 * turn_direction)
        right_motor.setVelocity(CRUISE_SPEED * 0.8 * turn_direction)
        
        if abs(current_wheel_pos - initial_wheel_pos) >= TARGET_90_DEG:
            state = "SHIFT"
            initial_wheel_pos = current_wheel_pos

    elif state == "SHIFT":
        left_motor.setVelocity(-CRUISE_SPEED)
        right_motor.setVelocity(-CRUISE_SPEED)
        
        # Corrección: Detectar vacío durante el cambio de carril (Fin del panel)
        if (left_val > EDGE_THRESHOLD) or (right_val > EDGE_THRESHOLD):
            state = "IDLE"
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            brush_motor.setVelocity(0.0)
            print("\n[ÉXITO] Limpieza terminada. Retornando a monitoreo de energía.")
            
        elif abs(current_wheel_pos - initial_wheel_pos) >= TARGET_SHIFT:
            state = "TURN_2"
            initial_wheel_pos = current_wheel_pos

    elif state == "TURN_2":
        left_motor.setVelocity(-CRUISE_SPEED * 0.8 * turn_direction)
        right_motor.setVelocity(CRUISE_SPEED * 0.8 * turn_direction)
        
        if abs(current_wheel_pos - initial_wheel_pos) >= TARGET_90_DEG:
            state = "FORWARD"
            turn_direction *= -1