from controller import Robot

TIME_STEP = 64
robot = Robot()

# Sensores de distancia
ds = []
dsNames = ['ds_right', 'ds_left']  # Asegúrate de que estos nombres coincidan con el modelo
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))  # Corrige "root" por "robot"
    ds[i].enable(TIME_STEP)

# Configuración de ruedas (ajusta según tu robot)
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))  # Corrige "wheelSkNames" por "wheelsNames"
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)  # Corrige "setVector(v@.0)" por "setVelocity(0.0)"

avoidObstacleCounter = 0

while robot.step(TIME_STEP) != -1:  # Corrige "-l" por "-1"
    leftSpeed = 1.0
    rightSpeed = 1.0

    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0  # Corrige "-=" por "=" (giro en su lugar)
    else:
        # Lectura de sensores
        for i in range(2):
            if ds[i].getValue() < 950.0:  # Ajusta este umbral según tus pruebas
                avoidObstacleCounter = 100

    # Asigna velocidades (ajusta según la configuración real de tu robot)
    wheels[0].setVelocity(leftSpeed)   # Del. izq
    wheels[1].setVelocity(rightSpeed)  # Del. der
    wheels[2].setVelocity(leftSpeed)   # Tras. izq
    wheels[3].setVelocity(rightSpeed)  # Tras. der