"""Autonomous Arm Controller for Lunar Rover"""

from controller import Robot, Camera
import math
import numpy as np

# Constantes de color para detección (en espacio RGB)
COLOR_YELLOW = [200, 200, 50]
COLOR_GREEN = [50, 200, 50]
COLOR_TOLERANCE = 50  # Tolerancia para la detección de color

# Estados del robot
STATE_SEARCHING = 0
STATE_APPROACHING = 1
STATE_PICKING = 2
STATE_TURNING = 3
STATE_COMPLETED = 4

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inicialización del estado
current_state = STATE_SEARCHING
target_found = False
turn_angle = 0
turn_target = 0

# Inicializar motores base
wheel_motors = []
for name in ['motor_FR', 'motor_FL', 'motor_RR', 'motor_RL']:
    motor = robot.getDevice(name)
    motor.setPosition(0)
    wheel_motors.append(motor)

drive_motors = []
for name in ['wheel_FR', 'wheel_FL', 'wheel_RR', 'wheel_RL']:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
    drive_motors.append(motor)

# Inicializar motores del brazo
arm_motors = {
    'waist': robot.getDevice('waist_motor'),
    'shoulder': robot.getDevice('shoulder_motor'),
    'elbow': robot.getDevice('elbow_motor'),
    'wrist': robot.getDevice('wrist_motor'),
    'pitch': robot.getDevice('pitch_motor'),
    'claw': robot.getDevice('phalanx_motor::right')
}

for motor in arm_motors.values():
    motor.setPosition(float('inf'))
    motor.setVelocity(0)

# Inicializar sensores del brazo
arm_sensors = {
    'waist': robot.getDevice('waist_sensor'),
    'shoulder': robot.getDevice('shoulder_sensor'),
    'elbow': robot.getDevice('elbow_sensor'),
    'wrist': robot.getDevice('wrist_sensor'),
    'pitch': robot.getDevice('pitch_sensor'),
    'fingers': robot.getDevice('phalanx_sensor'),
    'finger': robot.getDevice('finger_sensor')
}

for sensor in arm_sensors.values():
    sensor.enable(timestep)

# Inicializar cámara
camera = robot.getDevice('camera')
camera.enable(timestep * 2)
width = camera.getWidth()
height = camera.getHeight()

# Función para detectar colores objetivo
def detect_target_color(image):
    targets = []
    for y in range(height):
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            
            # Detectar amarillo
            if (abs(r - COLOR_YELLOW[0]) < COLOR_TOLERANCE and
                abs(g - COLOR_YELLOW[1]) < COLOR_TOLERANCE and
                abs(b - COLOR_YELLOW[2]) < COLOR_TOLERANCE):
                targets.append((x, y, 'yellow'))
            
            # Detectar verde
            elif (abs(r - COLOR_GREEN[0]) < COLOR_TOLERANCE and
                  abs(g - COLOR_GREEN[1]) < COLOR_TOLERANCE and
                  abs(b - COLOR_GREEN[2]) < COLOR_TOLERANCE):
                targets.append((x, y, 'green'))
    
    return targets

# Función para mover el brazo a posición de recolección
def move_arm_to_pick():
    # Posiciones predefinidas para recoger objetos
    arm_motors['shoulder'].setVelocity(1.0)
    arm_motors['shoulder'].setPosition(-1.0)
    
    arm_motors['elbow'].setVelocity(1.0)
    arm_motors['elbow'].setPosition(0.5)
    
    arm_motors['wrist'].setVelocity(1.0)
    arm_motors['wrist'].setPosition(0)
    
    arm_motors['pitch'].setVelocity(1.0)
    arm_motors['pitch'].setPosition(0)
    
    arm_motors['claw'].setVelocity(0.5)
    arm_motors['claw'].setPosition(0.8)  # Abrir garra

# Función para recoger objeto
def pick_object():
    arm_motors['claw'].setVelocity(0.3)
    arm_motors['claw'].setPosition(0)  # Cerrar garra

# Función para mover el brazo a posición de transporte
def move_arm_to_carry():
    arm_motors['shoulder'].setVelocity(0.5)
    arm_motors['shoulder'].setPosition(0)
    
    arm_motors['elbow'].setVelocity(0.5)
    arm_motors['elbow'].setPosition(0)
    
    arm_motors['wrist'].setVelocity(0.5)
    arm_motors['wrist'].setPosition(0)

# Función para girar el rover
def turn_robot(angle):
    global turn_angle, turn_target
    
    # Configurar motores para giro
    wheel_motors[0].setPosition(0.785398)  # FR
    wheel_motors[1].setPosition(-0.785398) # FL
    wheel_motors[2].setPosition(-0.785398)  # RR
    wheel_motors[3].setPosition(0.785398)  # RL
    
    # Calcular velocidad y dirección
    turn_target = angle
    turn_angle = 0
    for motor in drive_motors:
        motor.setVelocity(2.0 if angle > 0 else -2.0)

# Bucle principal
while robot.step(timestep) != -1:
    # Obtener imagen de la cámara
    image = camera.getImage()
    
    # Leer sensores del brazo
    sensor_values = {name: sensor.getValue() for name, sensor in arm_sensors.items()}
    
    # Máquina de estados
    if current_state == STATE_SEARCHING:
        # Buscar objetivos en la imagen
        targets = detect_target_color(image)
        
        if targets:
            print(f"Objetivo detectado: {targets[0][2]} en posición {targets[0][0]}, {targets[0][1]}")
            target_found = True
            current_state = STATE_APPROACHING
            
            # Mover brazo a posición de recolección
            move_arm_to_pick()
            
            # Calcular movimiento de aproximación basado en posición del objetivo
            target_x = targets[0][0]
            if target_x < width/3:
                turn_robot(math.pi/6)  # Girar 30° izquierda
            elif target_x > 2*width/3:
                turn_robot(-math.pi/6)  # Girar 30° derecha
            else:
                # Avanzar directamente
                for motor in drive_motors:
                    motor.setVelocity(3.0)
    
    elif current_state == STATE_APPROACHING:
        # Verificar si estamos lo suficientemente cerca (simulado)
        if target_found and robot.getTime() > 5:  # Después de 5 segundos de aproximación
            current_state = STATE_PICKING
            for motor in drive_motors:
                motor.setVelocity(0)
            
            # Cerrar garra para recoger
            pick_object()
    
    elif current_state == STATE_PICKING:
        # Esperar a que la garra se cierre completamente
        if sensor_values['fingers'] < 0.1:
            current_state = STATE_TURNING
            move_arm_to_carry()
            turn_robot(math.pi/2)  # Girar 90°
    
    elif current_state == STATE_TURNING:
        # Simular seguimiento del ángulo de giro
        turn_angle += 0.05 * (2.0 if turn_target > 0 else -2.0)
        
        if abs(turn_angle) >= abs(turn_target):
            current_state = STATE_COMPLETED
            for motor in drive_motors:
                motor.setVelocity(0)
    
    elif current_state == STATE_COMPLETED:
        print("Recolección completada. Volviendo a modo de búsqueda.")
        current_state = STATE_SEARCHING
        target_found = False