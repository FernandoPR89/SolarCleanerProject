from controller import Robot

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

state = "FORWARD"
turn_direction = 1
turn_counter = 0
initial_wheel_pos = 0.0

brush_motor.setVelocity(BRUSH_SPEED)

# Variable para imprimir el porcentaje de batería sin saturar la consola
print_timer = 0

while robot.step(TIME_STEP) != -1:
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()
    current_wheel_pos = ps_left.getValue()

    # --- SIMULACIÓN DE BATERÍA ---
    # Convertimos los milisegundos del TIME_STEP a segundos (dt)
    dt = TIME_STEP / 1000.0 
    
    if state in ["STOP", "OUT_OF_BATTERY"]:
        current_battery -= IDLE_CONSUMPTION_RATE * dt
    else:
        # Consume energía del cepillo + 2 motores + electrónica base
        total_consumption = BRUSH_CONSUMPTION_RATE + (MOTOR_CONSUMPTION_RATE * 2) + IDLE_CONSUMPTION_RATE
        current_battery -= total_consumption * dt

    if current_battery <= 0 and state != "OUT_OF_BATTERY":
        current_battery = 0
        state = "OUT_OF_BATTERY"
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        brush_motor.setVelocity(0.0)
        print("\n[ALERTA] ¡Batería agotada! El robot se ha apagado a mitad del trabajo.")

    # Imprimir estado de la batería cada ~1 segundo
    print_timer += dt
    if print_timer >= 1.0 and state != "OUT_OF_BATTERY":
        print(f"Batería restante: {current_battery:.1f}%")
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
            state = "STOP"
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            brush_motor.setVelocity(0.0)
            print("\n[ÉXITO] ¡Limpieza terminada exitosamente!")
            
        elif abs(current_wheel_pos - initial_wheel_pos) >= TARGET_SHIFT:
            state = "TURN_2"
            initial_wheel_pos = current_wheel_pos

    elif state == "TURN_2":
        left_motor.setVelocity(-CRUISE_SPEED * 0.8 * turn_direction)
        right_motor.setVelocity(CRUISE_SPEED * 0.8 * turn_direction)
        
        if abs(current_wheel_pos - initial_wheel_pos) >= TARGET_90_DEG:
            state = "FORWARD"
            turn_direction *= -1