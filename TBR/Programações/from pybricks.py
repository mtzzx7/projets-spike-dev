from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

# Inicializando o hub e os motores
hub = PrimeHub()
motor_esquerdo = Motor(Port.A)
motor_direito = Motor(Port.B)
motor_garra = Motor(Port.E)

def GyroMove(target_degrees_motor, speed):
    # Resetando o giroscópio e os motores
    hub.imu.reset_heading()
    motor_esquerdo.reset_angle(0)
    motor_direito.reset_angle(0)
    Kp = 2
    Ki = 0.1
    Kd = 0.1
    integral = 0
    previous_error = 0
    
    while motor_esquerdo.angle() < target_degrees_motor:
        # Corrigindo com base no erro de ângulo
        error = hub.imu.heading() - 0
        integral = integral + error
        derivative = error - previous_error
        correction = error * Kp + integral * Ki + derivative * Kd  # Controle proporcional (P)

        # Controlando os motores com correção
        motor_esquerdo.run(speed - correction)
        motor_direito.run(speed + correction)
        
    # Parar os motores ao atingir o alvo
    motor_esquerdo.stop()
    motor_direito.stop()

def GyroTurn(target_angle, speed):
    # Resetando o giroscópio
    hub.imu.reset_heading()
    
    while abs(hub.imu.heading()) < abs(target_angle):
        error = target_angle - hub.imu.heading()
        correction = error * 1.2  # Fator P (proporcional) ajustável
        
        # Ajustando os motores para girar em direção ao alvo
        motor_esquerdo.run(-speed - correction)
        motor_direito.run(speed + correction)
    
    # Parando os motores
    motor_esquerdo.stop()
    motor_direito.stop()


def abrir_garra():
    # Define o movimento de abertura da garra
    motor_garra.run_until_stalled(-500, duty_limit=50)
    motor_garra.reset_angle(0)  # Reseta o ângulo da garra quando completamente aberta

def fechar_garra():
    # Define o movimento de fechamento da garra
    motor_garra.run_until_stalled(500, duty_limit=50)

# Exemplo de uso: GyroMove que anda 720 graus nos motores corrigindo o giroscópio
GyroMove(720, 80)
