# LEGO type:standard slot:1 autostart

import math
from spike import PrimeHub, Motor, MotorPair, ColorSensor
from spike.control import wait_for_seconds, Timer
from hub import battery
hub = PrimeHub()


import hub as hub2

import sys

        #Preparação para execução paralela de código
accelerate = True
run_generator = True
runSmall = True

lastAngle = 0
oldAngle = 0

gyroValue = 0

        #Crie seus objetos aqui.
hub = PrimeHub()

        #PID
pRegler = 0.0
iRegler = 0.0
dRegler = 0.0

pReglerLight = 0.0
iReglerLight = 0.0
dReglerLight = 0.0

"""
        Inicializar sensores de cores
        sensor esquerdo: porta F
        sensor direito: porta E
"""
colorE = ColorSensor('C') #Ajuste as portas do sensor ate que correspondam a sua configuracao, recomendamos atribuir suas portas a's do programa para facilitar o uso
colorF = ColorSensor('D')
smallMotorA = Motor('A')
smallMotorD = Motor('E')

        #Base do Robô
circumference = 17.6 #Circunferência da roda movida pelo robo em cm
sensordistance = 7 #Distancia entre os dois sensores de luz em cm. Usado no alinhamento tangente 6,4 em pinos

cancel = False
inMain = True

class DriveBase:

            def __init__(self, hub, smallMotorA, smallMotorD):
                self.hub = hub
                self.leftMotor = Motor(smallMotorA)
                self.rightMotor = Motor(smallMotorD)
                self.movement_motors = MotorPair(smallMotorA, smallMotorD) 

            def lineFollower(self, distance, startspeed, maxspeed, endspeed, sensorPort, side, addspeed = 0.2, brakeStart = 0.7 , stopMethod=None, generator = None, stop = True):
                """
                    Esta é a função usada para permitir que o robô siga uma linha até que a distância inserida seja alcançada ou o outro sensor do robô detecte uma linha.
                    Como todas as funções que acionam o robô, esta função possui aceleração e frenagem lineares. Ele também usa valores PID que são definidos automaticamente dependendo do
                    velocidade atual do robô (Ver função PIDCalculationLight)
                    Parâmetros
                    -------------
                    distância: O valor informa ao programa a distância que o robô deve percorrer. Tipo: Inteiro. Padrão: nenhum valor padrão
                    velocidade: A velocidade na qual o robô deve iniciar. Tipo: Inteiro. Padrão: nenhum valor padrão
                    maxspeed: A velocidade mais alta na qual o robô se move. Tipo: Inteiro. Padrão: nenhum valor padrão
                    endspeed: A velocidade que o robô atinge no final da função. Tipo: Inteiro. Padrão: nenhum valor padrão
                    addspeed: A porcentagem após a qual o robô atinge sua velocidade máxima. Tipo: Flutuador. Padrão: nenhum valor padrão
                    BrakeStart: O valor que usamos para informar ao robô após qual porcentagem da distância precisamos desacelerar. Tipo: Flutuador. Padrão: nenhum valor padrão
                    stopMethod: o Stopmethod que o robô usa para parar. Se nenhum stopMethod for passado, stopDistance será usado. Padrão: stopDistance
                    gerador: o gerador que faz algo paralelo durante a condução. Padrão: nenhum valor padrão
                    stop: o booleano que determina se o robô deve parar os motores após dirigir ou não. Padrão: Verdadeiro
                """
                
                if cancel:
                    return

                global run_generator, runSmall

                if generator == None:
                    run_generator = False

                #Definir a velocidade em que o robô começa
                speed = startspeed
                #Redefinir os valores PID para eliminar bugs
                change = 0
                old_change = 0
                integral = 0
                #Redefinir a distância percorrida do robô para eliminar bugs

                #Especifica o sensor de cor
                colorsensor = ColorSensor('C', 'D' )
                #Obtenha os graus dos motores girados antes que o robô se mova, permitindo o cálculo da distância sem reiniciar os motores
                loop = True
                #Retroceder não é suportado em nosso robô devido aos motores estarem na frente dos sensores de cores e o programa não funcionar
                if distance < 0:
                    print('ERR: distance < 0')
                    distance = abs(distance)
                #Calcule os valores alvo para os motores girarem
                finalDistance = (distance / 17.6) * 360
                #Calcule após que distância o robô tem para atingir a velocidade máxima
                accelerateDistance = finalDistance * addspeed
                deccelerateDistance = finalDistance * (1 - brakeStart)

                invert = 1

                #Cálculo do fator de direção, dependendo de qual lado da linha estamos
                if side == "left":
                    invert = 1
                elif side == "right":
                    invert = -1
                
                #Cálculo do início da desaceleração do robô
                
                self.left_Startvalue = self.leftMotor.get_degrees_counted()
                self.right_Startvalue = self.rightMotor.get_degrees_counted()
                drivenDistance = getDrivenDistance(self)

                brakeStartValue = brakeStart * finalDistance
                while loop:
                    if cancel:
                        print("cancel")
                        break
                
                    if run_generator: #run parallel code execution
                        next(generator)

                    #Verifica a distância percorrida como uma média de ambos os motores para maior precisão
                    oldDrivenDistance = drivenDistance
                    drivenDistance = getDrivenDistance(self)
                    #Calcula o valor alvo do Robô como a borda das linhas pretas e brancas
                    old_change = change

                    change = colorsensor.get_reflected_light() - 50


                    #Cálculo do fator de direção usando PID, define novo valor I

                
                    steering = (((change * pReglerLight) + (integral * iReglerLight) + (dReglerLight * (change - old_change)))) * invert
                    integral = change + integral
                    #Cálculo da velocidade atual do robô, usada para aceleração, frenagem, etc.
                    speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)

                    pidCalculationLight(speed)
                    #PID atualizar valores
                    steering = max(-100, min(steering, 100))

                    #Condução usando valores de velocidade calculados com PID e aceleração para direção, uso de verificação de distância

                    self.movement_motors.start_at_power(int(speed), int(steering))

                    if stopMethod != None:
                        if stopMethod.loop():
                            loop = False
                    else:   
                        if finalDistance < drivenDistance:
                            break

                if stop:
                    self.movement_motors.stop()
                    
                run_generator = True
                runSmall = True
                generator = 0
                return

            def gyroRotation(self, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, rotate_mode = 0, stopMethod = None, generator = None, stop = True):
                """
                    Esta é a função que usamos para fazer o robô girar o comprimento de um ângulo específico ou para o robô girar até sentir uma linha. Mesmo nesta função o robô
                    pode acelerar e desacelerar. Ele também possui calibrações de Gyrosensor baseadas em nossa experiência experimental.
                    Parâmetros
                    -------------
                    ângulo: O ângulo que o robô deve girar. Use números negativos para girar no sentido anti-horário. Tipo: Inteiro. Valor padrão: Nenhum valor padrão
                    startspeed: A velocidade na qual o robô deve iniciar. Tipo: Inteiro. Padrão: nenhum valor padrão
                    maxspeed: A velocidade mais alta na qual o robô se move. Tipo: Inteiro. Padrão: nenhum valor padrão
                    endspeed: A velocidade que o robô atinge no final da função. Tipo: Inteiro. Padrão: nenhum valor padrão
                    addspeed: A porcentagem após a qual o robô atinge a velocidade máxima. Tipo: Flutuador. Padrão: nenhum valor padrão
                    BrakeStart: A porcentagem após a qual o robô começa a desacelerar até atingir a velocidade final. Tipo: Flutuador. Padrão: nenhum valor padrão
                    rotate_mode: Diferentes tipos de torneamento. 0: Ambos os motores giram, o robô gira no mesmo lugar. 1: Somente o motor externo gira, resultando em um canto. Tipo: Inteiro. Padrão: 0
                    stopMethod: o Stopmethod que o robô usa para parar. Se nenhum stopMethod for passado, stopDistance será usado. Padrão: stopDistance
                    gerador: o gerador que faz algo paralelo durante a condução. Padrão: nenhum valor padrão
                    stop: o booleano que determina se o robô deve parar os motores após dirigir ou não. Padrão: Verdadeiro
                """

                if cancel:
                    return

                global run_generator, runSmall

                if generator == None:
                    run_generator = False

                if rotate_mode == 0:
                    startspeed = abs(startspeed)
                    maxspeed = abs(maxspeed)
                    endspeed = abs(endspeed)

                speed = startspeed

                #Definir variáveis ​​padrão
                rotatedDistance = 0
                steering = 1

                accelerateDistance = abs(angle * addspeed) 
                deccelerateDistance = abs(angle * (1 - brakeStart))

                #Calibração do sensor giroscópio
                angle = angle * (2400/2443) #Valor experimental baseado em 20 rotações do robô

                #Setting variables based on inputs
                loop = True
                gyroStartValue = getGyroValue() #Ângulo de guinada usado devido à orientação do self.hub. Talvez seja necessário alterar isso
                brakeStartValue = (angle + gyroStartValue) * brakeStart

                #Inversão do valor de direção para girar no sentido anti-horário
                if angle < 0:
                    steering = -1

                #Testando para ver se é necessário turining, gira até loop = False

                while loop:
                    if cancel:
                        break

                    if run_generator: #Executar execução paralela de código
                        next(generator)

                    oldRotatedDistance = rotatedDistance
                    rotatedDistance = getGyroValue() #Yaw angle used due to orientation of the self.hub. This might need to be changed
                    speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, abs(1), abs(0))
                    
                    
                    #Checking for variants
                    #Both Motors turn, robot moves on the spot
                    if rotate_mode == 0:
                        self.movement_motors.start_tank_at_power(int(speed) * steering, -int(speed) * steering)
                    #Only outer motor turns, robot has a wide turning radius
                    
                    elif rotate_mode == 1:

                        if angle * speed > 0:
                            self.leftMotor.start_at_power(- int(speed))
                        else:
                            self.rightMotor.start_at_power(+ int(speed))

                    if stopMethod != None:
                        if stopMethod.loop():
                            loop = False
                            break
                    elif abs(angle) <= abs(rotatedDistance - gyroStartValue):                   
                            loop = False
                            break



                #Stops movement motors for increased accuracy while stopping
                if stop:
                    self.movement_motors.stop()

                run_generator = True
                runSmall = True

                return # End of gyroStraightDrive

            def gyroStraightDrive(self, distance, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, stopMethod=None, offset = 0, generator = None, stop = True):
                """
                    Esta é a função que usamos para fazer o robô avançar ou retroceder sem derrapar. Pode acelerar, pode desacelerar e também tem PID. Você pode definir os valores
                    de uma forma que você possa dirigir até que a distância inserida seja alcançada ou até que o robô detecte uma linha.
                    Parâmetros
                    -------------
                    distância: a distância que o robô deve percorrer. Tipo: Inteiro. Padrão: nenhum valor padrão
                    velocidade: A velocidade na qual o robô deve iniciar. Tipo: Inteiro. Padrão: nenhum valor padrão
                    maxspeed: A velocidade mais alta na qual o robô se move. Tipo: Inteiro. Padrão: nenhum valor padrão
                    endspeed: A velocidade que o robô atinge no final da função. Tipo: Inteiro. Padrão: nenhum valor padrão
                    addspeed: A velocidade que o robô adiciona para acelerar. Tipo: Flutuador. Padrão: 0,2
                    BrakeStart: O valor que usamos para informar ao robô após qual porcentagem da distância precisamos desacelerar. Tipo: Flutuador. Padrão: 0,8
                    porta: Este valor informa ao programa se o robô deve verificar se há uma linha preta com o sensor de luz especificado. Tipo: String. Padrão: 0
                    lightValue: Este valor informa ao programa o valor em que o robô deve parar se a porta o vir. Tipo: Inteiro. Padrão: 0
                    alinhar_variant: Diz ao robô para se alinhar a uma linha se vir uma. 0: Sem alinhamento. 1: alinhamento padrão. 2: alinhamento baseado em tangente Tipo: Inteiro. Padrão: 0
                    detectLineStart: O valor que usamos para informar ao robô após qual porcentagem da distância precisamos procurar a linha para onde dirigir. Tipo: Flutuador. Padrão: 0
                    offset: O valor envia o robô em uma direção indicada pelo valor inserido. Tipo: Inteiro. Padrão: 0
                    gerador: Função executada enquanto o robô está executando o gyroStraightDrive. Escreva aqui a função desejada e seus parâmetros. Tipo: . Padrão: 0
                    stopMethod: o Stopmethod que o robô usa para parar. Se nenhum stopMethod for passado, stopDistance será usado. Padrão: stopDistance
                    gerador: o gerador que faz algo paralelo durante a condução. Padrão: nenhum valor padrão
                    stop: o booleano que determina se o robô deve parar os motores após dirigir ou não. Padrão: Verdadeiro          
                """

                if cancel:
                    return

                global run_generator, runSmall
                global pRegler, iRegler, dRegler
                
                if generator == None:
                    run_generator = False

                #Set starting speed of robot
                speed = startspeed
                #Sets PID values

                change = 0
                old_change = 0
                integral = 0
                steeringSum = 0

                invert = -1

                #Sets values based on user inputs
                loop = True


                gyroStartValue = getGyroValue()

                #Error check for distance
                if distance < 0:
                    print('ERR: distance < 0')
                    distance = abs(distance)

                #Calulation of degrees the motors should turn to
                #17.6 is wheel circumference in cm. You might need to adapt it
                rotateDistance = (distance / 17.6) * 360
                accelerateDistance = rotateDistance * addspeed
                deccelerateDistance = rotateDistance * (1 - brakeStart)

                #Inversion of target rotation value for negative values
                if speed < 0:
                    invert = 1

                #Calculation of braking point
                self.left_Startvalue = self.leftMotor.get_degrees_counted()
                self.right_Startvalue = self.rightMotor.get_degrees_counted()
                brakeStartValue = brakeStart * rotateDistance
                drivenDistance = getDrivenDistance(self)

                while loop:
                    if cancel:
                        break
                    if run_generator: #run parallel code execution
                        next(generator)

                    #Calculation of driven distance and PID values
                    oldDrivenDistance = drivenDistance
                    drivenDistance = getDrivenDistance(self)

                    pidCalculation(speed)
                    change = getGyroValue() - gyroStartValue #yaw angle used due to orientation of the self.hub


                    currenSteering = (change * pRegler + integral * iRegler + dRegler * (change - old_change)) + offset + steeringSum*0.02

                    currenSteering = max(-100, min(currenSteering, 100))
                    #print("steering: " + str(currenSteering) + " gyro: " + str(change) + " integral: " + str(integral))

                    steeringSum += change
                    integral += change - old_change
                    old_change = change

                    #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight
                    speed = speedCalculation(speed, startspeed,maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)
                    self.movement_motors.start_at_power(int(speed), invert * int(currenSteering))


                    if stopMethod != None:
                        if stopMethod.loop():
                            loop = False
                    elif rotateDistance < drivenDistance:                   
                            loop = False


                if stop:
                    self.movement_motors.stop()
            
                run_generator = True
                runSmall = True

                return #End of gyroStraightDrive

            def arcRotation(self, radius, angle, startspeed, maxspeed, endspeed, addspeed = 0.3, brakeStart = 0.7, stopMethod=None, generator = None, stop = True):  
                """
                    This is the function that we use to make the robot drive a curve with a specified radius and to a given angle
                    Parameters
                    -------------
                    radius: the radius of the curve the robot is supposed to drive; measured from the outside edge of the casing. Type: Integer. Default: 0
                    angle: the angle that the robot is supposed to rotate on the curve. Type: Integer. Default: 0
                    speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value
                    maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value
                    endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value
                    addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: 0.2
                    brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: 0.8
                    stopMethod: the Stopmethod the robot uses to stop. If no stopMethod is passed stopDistance is used instead. Default: stopDistance
                    generator:  the generator that runs something parallel while driving. Default: No default value
                    stop: the boolean that determines whether the robot should stop the motors after driving or not. Default: True
                """

                if cancel:
                    print("cancel")
                    return

                global run_generator, runSmall

                if generator == None:
                    run_generator = False

                angle = angle * (2400/2443) #gyro calibration
                
                gyroStartValue = getGyroValue()
                finalGyroValue = gyroStartValue + angle
                currentAngle = gyroStartValue

                accelerateDistance = abs(angle * addspeed)
                deccelerateDistance = abs(angle * (1 - brakeStart))
                brakeStartValue = abs(angle * brakeStart)

                loop = True

                #Calculating the speed ratios based on the given radius
                if angle * startspeed > 0:
                    speed_ratio_left = (radius+14) / (radius+2) #calculate speed ratios for motors. These will need to be adapted based on your robot design
                    speed_ratio_right = 1
                else:
                    speed_ratio_left = 1
                    speed_ratio_right = (radius+14) / (radius+2)
                
                #Calculating the first speed to drive with
                left_speed = speedCalculation(startspeed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)
                right_speed = speedCalculation(startspeed, startspeed , maxspeed , endspeed , accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)
                while loop:
                    #when the cancel button is pressed stop the gyrostraight drive directly
                    if cancel:
                        break

                    if run_generator: #run parallel code execution
                        next(generator)

                    currentAngle = getGyroValue() #Yaw angle used due to orientation of the self.hub. This might need to be changed
                
                    #Calculating the current speed the robot should drive
                    left_speed = speedCalculation(left_speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)
                    right_speed = speedCalculation(right_speed, startspeed , maxspeed , endspeed , accelerateDistance, deccelerateDistance, brakeStartValue, 1, 0)


                    self.movement_motors.start_tank_at_power(int(left_speed* speed_ratio_left), int(right_speed* speed_ratio_right))
                
                    #if there is a stopMethod passed use it and stop the loop if it returns true otherwise check if the robot has rotated to the given angle
                    if stopMethod != None:
                        #print("stoMeth")
                        if stopMethod.loop():
                            loop = False
                            break

                    (angle / abs(angle))
                    if finalGyroValue * (angle / abs(angle)) < currentAngle * (angle / abs(angle)):
                        #print("finalGyroValue: " + str(finalGyroValue) + " rotatedDistance: " + str(currentAngle))                  
                        loop = False
                        break


                    

                #if stop is true then stop the motors otherwise don't stop the motor
                if stop:
                    self.movement_motors.stop()

                run_generator = True
                runSmall = True
                return #End of arcRotation

def resetGyroValue():
            global gyroValue
            hub2.motion.yaw_pitch_roll(0)

            gyroValue = 0

def getGyroValue():

            #this method is used to return the absolute gyro Angle and the angle returned by this method doesn't reset at 180 degree
            global lastAngle
            global oldAngle
            global gyroValue

            #gets the angle returned by the spike prime program. The problem is the default get_yaw_angle resets at 180 and -179 back to 0
            angle = hub.motion_sensor.get_yaw_angle()

            if angle != lastAngle:
                oldAngle = lastAngle
                
            lastAngle = angle

            if angle == 179 and oldAngle == 178:
                hub2.motion.yaw_pitch_roll(0)#reset
                gyroValue += 179
                angle = 0
            
            if angle == -180 and oldAngle == -179:
                hub2.motion.yaw_pitch_roll(0) #reset
                gyroValue -= 180   
                angle = 0

            return gyroValue + angle

def getDrivenDistance(data):


            #print(str(abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue)) + " .:. " + str(abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)))

            drivenDistance = (
                            abs(data.leftMotor.get_degrees_counted() - data.left_Startvalue) + 
                            abs(data.rightMotor.get_degrees_counted() - data.right_Startvalue)) / 2

            return drivenDistance

def defaultClass(object, db):
            object.db = db
            object.leftMotor = db.leftMotor
            object.rightMotor = db.rightMotor

            object.left_Startvalue = abs(db.leftMotor.get_degrees_counted())
            object.right_Startvalue = abs(db.rightMotor.get_degrees_counted())
            return object

class stopMethods(): #This class has all our stopmethods for easier coding and less redundancy
            
            class stopLine():
                """
                    Dirija até que uma linha seja detectada
                     Parâmetros
                     -------------
                     db: a base de transmissão do robô
                     porta: Porta para detectar linha ligada
                     lightvalue: Valor da luz a ser detectada
                     detectLineDistance: Distância até começar a detectar uma linha
                    """
                def __init__(self, db, port, lightvalue, detectLineDistance):
                    self = defaultClass(self, db)            

                    self.port = port
                    self.detectLineDistance = (detectLineDistance / 17.6) * 360

                    #if lightvalue bigger 50 stop when lightvalue is higher
                    self.lightvalue = lightvalue


                def loop(self):


                    drivenDistance = getDrivenDistance(self)

                    if abs(self.detectLineDistance) < abs(drivenDistance):
                        if self.lightvalue > 50:
                            if ColorSensor(self.port).get_reflected_light() > self.lightvalue:
                                return True
                        else:
                            if ColorSensor(self.port).get_reflected_light() < self.lightvalue:
                                return True

                    return False
            
            class stopAlign():
                """
                    Dirija até que uma linha seja detectada
                     Parâmetros
                     -------------
                     db: a base de transmissão do robô
                     porta: Porta para detectar linha ligada
                     lightvalue: Valor da luz a ser detectada
                     velocidade: velocidade na qual o robô procura outra linha

                    """
                def __init__(self, db, lightvalue, speed):
                    self = defaultClass(self, db)    
                    self.speed = speed


                    #if lightvalue bigger 50 stop when lightvalue is higher
                    self.lightValue = lightvalue


                def loop(self):

                    if colorE.get_reflected_light() < self.lightValue:
                        self.rightMotor.stop()
                        #Turning robot so that other colour sensor is over line
                        while True:

                            self.leftMotor.start_at_power(-int(self.speed))

                            #Line detection and stopping
                            if colorF.get_reflected_light() < self.lightValue or cancel:
                                self.leftMotor.stop()
                                return True
                        

                    #O sensor de cor F vê a linha primeiro
                    elif colorF.get_reflected_light() < self.lightValue:

                        self.leftMotor.stop()

                        #Girar o robô para que o outro sensor de cor fique acima da linha
                        while True:
                            self.rightMotor.start_at_power(int(self.speed))

                            #Detecção e parada de linha
                            if colorE.get_reflected_light() < self.lightValue or cancel:
                                self.rightMotor.stop()
                                return True
                    

                    return False

            class stopTangens():
                """
                    Dirija até que uma linha seja detectada
                     Parâmetros
                     -------------
                     db: a base de transmissão do robô
                     porta: Porta para detectar linha ligada
                     lightvalue: Valor da luz a ser detectada
                     velocidade: Distância até começar a detectar uma linha

                    """
                def __init__(self, db, lightvalue, speed):
                    self.count = 0
                    self = defaultClass(self, db)    
                    self.speed = speed
                    #se o valor da luz for maior 50 para quando o valor da luz for maior
                    self.lightValue = lightvalue
                    self.detectedLineDistance = 0

                    self.invert = 1
                    if speed < 0:
                        self.invert = -1
                    
                def loop(self):
                    drivenDistance = getDrivenDistance(self)
                    if colorE.get_reflected_light() < self.lightValue:
                            #medição da distância que o robô percorreu desde que viu a linha
                            if(self.detectedLineDistance == 0):
                                self.detectedLineDistance = getDrivenDistance(self)
                                self.detectedPort = 'E'

                            elif self.detectedPort == 'F':
                                db.movement_motors.stop() #Para o robô com sensor F na linha
                                angle = math.degrees(math.atan(((drivenDistance - self.detectedLineDistance) / 360 * circumference) / sensordistance)) #Calculating angle that needs to be turned using tangent
                                #print("angle: " + str(angle))
                                db.gyroRotation(-angle, self.invert * self.speed, self.invert * self.speed, self.invert * self.speed, rotate_mode=1) #Standard gyrorotation for alignment, but inverting speed values if necessary

                                db.movement_motors.stop() #Parando o robô para maior confiabilidade
                                return True

                        #O sensor de cor F vê a linha primeiro
                    elif colorF.get_reflected_light() < self.lightValue:
                        #medição da distância que o robô percorreu desde que viu a linha
                        if(self.detectedLineDistance == 0):
                            self.detectedLineDistance = drivenDistance
                            self.detectedPort = 'F'

                        elif self.detectedPort == 'E':
                            db.movement_motors.stop() #Para o robô com sensor E na linha
                            angle = math.degrees(math.atan(((drivenDistance - self.detectedLineDistance) / 360 * circumference) / sensordistance)) #Calculation angle that needs to be turned using tangent
                            db.gyroRotation(angle, self.invert * self.speed, self.invert * self.speed, self.invert * self.speed, rotate_mode=1) #Standard gyrorotation for alignment, but inverting speed values if necessary
                            db.movement_motors.stop() #Parando o robô para maior confiabilidade
                            return True

                    return False
            class stopDegree():
                """
                    Girar até que um certo grau seja alcançado
                     Parâmetros
                     -------------
                     db: a base de transmissão do robô
                     ângulo: o ângulo para girar

                """
                def __init__(self, db, angle):
                    self.angle = angle * (336/360)
                    
                    self.gyroStartValue = getGyroValue() #Ângulo de guinada usado devido à orientação do self.hub.
                    

                def loop(self):
                    rotatedDistance = getGyroValue() #Ângulo de guinada usado devido à orientação do self.hub.

                    if abs(self.angle) <= abs(rotatedDistance - self.gyroStartValue):
                        return True
                    else:
                        return False

            class stopTime():

                """
                    Dirija até atingir um determinado tempo
                     Parâmetros
                     -------------
                     db: a base de transmissão do robô
                     hora: a hora de dirigir
                """

                def __init__(self, db, time) -> None:
                    self = defaultClass(self, db)
                    self.time = time
                    self.timer = Timer()
                    self.startTime = self.timer.now()

                def loop(self):
                    if self.timer.now() > self.startTime + self.time:
                        return True
                    else:
                        return False
            
            class stopResistance():

                """
                    Dirija até que o robô não se mova mais
                     Parâmetros
                     -------------
                     db: a base de transmissão do robô
                     resistência: o valor que a resistência deve estar abaixo para parar
              
                """
                def __init__(self, db, resistance):
                    self = defaultClass(self, db)
                    self.resistance = resistance
                    self.timer = Timer()
                    
                    self.startTime = 0
                    self.lower = False
                    self.runs = 0

                def loop(self):

                    self.runs += 1
                    motion = abs(hub2.motion.accelerometer(True)[2])

                    if motion < self.resistance:
                        self.lower = True

                    if self.runs > 15:
                        if self.lower:
                            if self.startTime == 0:
                                self.startTime = self.timer.now()

                            if self.timer.now() > self.startTime:
                                return True

                        else:
                            self.lower = False
                            return False
                        
def motorResistance(speed, port, resistancevalue):
            """
           deixa o motor parar quando atinge um obstáculo
            """
            if abs(resistancevalue) > abs(speed):
                return

                
            if cancel:
                return

            if port == "A":
                smallMotorA.start_at_power(speed)
                while True:
                    old_position = smallMotorA.get_position()
                    wait_for_seconds(0.4)
                    if abs(old_position - smallMotorA.get_position())<resistancevalue or cancel:
                        smallMotorA.stop()
                        print("detected stalling")
                        return

            elif port == "D":
                smallMotorD.start_at_power(speed)
                while True:
                    old_position = smallMotorD.get_position()
                    wait_for_seconds(0.4)
                    if abs(old_position - smallMotorD.get_position())<resistancevalue or cancel:
                        smallMotorD.stop()
                        print("detected stalling")
                        return
            else:
                print("wrong port selected. Select A or D")
                return

def speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, deccelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance):
            """
                Usado para calcular todas as velocidades em nossos programas. Feito separadamente para reduzir a redundância. Freia e acelera
                 Parâmetros
                 -------------
                 velocidade: A velocidade atual que o robô tem
                 startspeed: Velocidade em que o robô inicia. Tipo: Inteiro. Padrão: Nenhum valor padrão.
                 maxspeed: A velocidade máxima que o robô atinge. Tipo: Inteiro. Padrão: Nenhum valor padrão.
                 endspeed: Velocidade que o robô almeja durante a frenagem, velocidade mínima no final do programa. Tipo: Inteiro. Padrão: Nenhum valor padrão.
                 addspeed: Porcentagem da distância após a qual o robô atinge a velocidade máxima. Tipo: Inteiro. Padrão: Nenhum valor padrão.
                 BrakeStartValue: Porcentagem da distância percorrida após a qual o robô começa a frear. Tipo: Inteiro. Padrão: Nenhum valor padrão.
                 DriveDistance: Cálculo da distância percorrida em graus. Tipo: Inteiro. Padrão: Nenhum valor padrão.
             """

            addSpeedPerDegree = (maxspeed - startspeed) / accelerateDistance 
            subSpeedPerDegree = (maxspeed - endspeed) / deccelerateDistance
            

            subtraction = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * subSpeedPerDegree
            addition = (abs(drivenDistance) - abs(oldDrivenDistance) if abs(drivenDistance) - abs(oldDrivenDistance) >= 1 else 1) * addSpeedPerDegree

            if abs(drivenDistance) > abs(brakeStartValue):

                if abs(speed) > abs(endspeed):
                    speed = speed - subtraction
                    
            elif abs(speed) < abs(maxspeed):

                speed = speed + addition

            return speed

def breakFunction(args):
            """
            Permite que você pare manualmente a execução do round, mas ainda permanece no main.
             Isso é muito mais rápido e confiável do que pressionar o botão central.
            """
            global cancel, inMain
            if not inMain:
                cancel = True

def pidCalculation(speed):
            #golbally define valores PID com base na velocidade atual do robô, permitindo uma condução rápida e precisa
            global pRegler
            global iRegler
            global dRegler
            #Nota importante: Esses valores PID são experimentais e baseados em nosso projeto para o robô. Você precisará ajustá-los manualmente. Você também pode configurá-los estaticamente, como pode ver abaixo
            if speed > 0:
                pRegler = -0.17 * speed + 12.83
                iRegler = 12
                dRegler = 1.94 * speed - 51.9
                if pRegler < 3.2:
                    pRegler = 3.2
            else:
                pRegler = (11.1 * abs(speed))/(0.5 * abs(speed) -7) - 20
                iRegler = 10
                #iRegler = 0.02
                dRegler = 1.15**(- abs(speed)+49) + 88
            
def pidCalculationLight(speed):
            #Define os valores PID para o lineFollower com base na velocidade atual. Permite uma condução precisa e rápida
             #Nota importante: esses valores PID são experimentais e baseados em nosso projeto para o robô. Você precisará ajustá-los. Veja acima como fazer isso
            global pReglerLight
            global dReglerLight

            pReglerLight = -0.04 * speed + 4.11
            dReglerLight = 0.98 * speed - 34.2
            #set hard bottom para o valor d, caso contrário os valores não funcionam
            if dReglerLight < 5:
                dReglerLight = 5

def driveMotor(rotations, speed, port):
            """
           Permite que você acione um pequeno motor em paralelo com o gyroStraightDrive
             Parâmetros
             -------------
             rotações: as rotações que o motor gira
             velocidade: a velocidade na qual o motor gira
             porta: o motor usado. Nota: não podem ser os mesmos motores configurados no motor Drivebase
            """
                
            global runSmall
            global run_generator

            if cancel:
                runSmall = False
                run_generator = False

            while runSmall:
                smallMotor = Motor(port)
                smallMotor.set_degrees_counted(0)

                loop_small = True
                while loop_small:
                    drivenDistance = smallMotor.get_degrees_counted()
                    smallMotor.start_at_power(speed)
                    if (abs(drivenDistance) > abs(rotations) * 360):
                        loop_small = False
                    if cancel:
                        loop_small = False
                    yield

                smallMotor.stop()
                runSmall = False
                run_generator = False
            yield

hub2.motion.yaw_pitch_roll(0)

db = DriveBase(hub, 'B', 'C') #isso nos permite entregar nossos motores de maneira conveniente (B: driver esquerdo; C: driver direito). Isso é necessário para a função cancelar

def exampleOne():
            #Este exemplo tem como objetivo mostrar todas as opções para seguir uma linha. Consulte a documentação específica da função para maiores informações.
             db.lineFollower(15, 25, 35, 25, 'E', 'left') #segue o lado esquerdo de uma linha no sensor E por 15 cm. Acelera da velocidade 25 para 35 e termina na 25 novamente
             hub.left_button.wait_until_pressed()
             db.lineFollower(15, 25, 35, 25, 'E', 'left', 0.4, 0.6) #mesmo seguidor de linha de antes, mas com um período de aceleração e quebra mais longo
             hub.left_button.wait_until_pressed()
             db.lineFollower(15, 25, 35, 25, 'E', 'left', stopMethod=stopMethods.stopLine(db, 'F', 0.7)) #mesmo linefollower do primeiro, mas desta vez parando, quando o outro o sensor vê uma linha preta após pelo menos 70% da distância percorrida
             hub.left_button.wait_until_pressed()
             db.lineFollower(15, 25, 35, 25, 'E', 'left', stopMethod=stopMethods.stopResistance(db, 20)) #igual ao primeiro seguidor de linha, mas para quando a resistência desejada é atingida. Teste o valor da resistência com base no seu robô
             hub.left_button.wait_until_pressed()
             generator = driveMotor(5, 100, 'A')
             db.lineFollower(15, 25, 35, 25, 'E', 'left', Generator=generator) #mesmo que o primeiro seguidor de linha, mas aciona enquanto gira o Motor A por 5 rotações
             hub.left_button.wait_until_pressed()
             db.lineFollower(15, 25, 35, 25, 'E', 'left', stop=False) #mesmo que o primeiro seguidor de linha, mas não freia ativamente os motores. A transição desta ação para a próxima é então mais suave
             return

def exampleTwo():
            #Este exemplo tem como objetivo mostrar todas as opções de giro do robô. Consulte a documentação específica da função para maiores informações.
             db.gyroRotation(90, 25, 35, 25) #gira o robô 90° no sentido horário enquanto acelera da velocidade 25 para 35 e volta para 25
             hub.left_button.wait_until_pressed()
             db.gyroRotation(90, 25, 35, 25, 0.4, 0.5) #mesmo giro da primeira rotação, mas com fase de aceleração/frenagem mais longa
             hub.left_button.wait_until_pressed()
             db.gyroRotation(90, 25, 35, 25, rotate_mode=1) #mesma curva da primeira rotação, mas desta vez girando usando apenas uma roda em vez de girar no local. Suas velocidades podem precisar ser maiores para isso
             hub.left_button.wait_until_pressed()
             db.gyroRotation(90, 25, 35, 25, stopMethod=stopMethods.stopAlign(db, 25, 25)) #alinha o robô com uma linha no caminho de giro
             hub.left_button.wait_until_pressed()
             db.gyroRotation(90, 25, 35, 25, stopMethod=stopMethods.stopLine(db, 'E', 25, 0.7)) #gira até que o robô veja uma linha no sensor E após pelo menos 70% de giro
             hub.left_button.wait_until_pressed()
             db.gyroRotation(90, 25, 35, 25, stopMethod=stopMethods.stopTangens(db, 25, 25)) #alinha o robô como stopAlign, mas é um pouco mais preciso
             hub.left_button.wait_until_pressed()
            #remaining são iguais aos do linefollower. Consulte exampleOne ou a documentação das funções individuais
             return

def exampleThree():
            #Este exemplo tem como objetivo mostrar todas as opções para dirigir em linha reta. Consulte a documentação específica da função para maiores informações.
             db.gyroStraightDrive(30, 25, 35, 25) #dirige em linha reta por 30 cm
             hub.left_button.wait_until_pressed()
             db.gyroStraightDrive(30, 25, 55, 25, 0.1, 0.9) #igual ao primeiro drive, mas mais rápido e com aceleração/frenagem mais fortes
             hub.left_button.wait_until_pressed()
             db.gyroStraightDrive(30, 25, 35, 25, offset=15) #igual ao primeiro drive, mas aponta 15° no sentido horário como orientação do alvo
             #remaining recursos do código são explicados em exemplos anteriores. Consulte exampleOne, exampleTwo e documentação adicional dentro de funções individuais
             return


def exampleFour():
    #Este exemplo tem como objetivo mostrar todas as opções para virar em uma curva grande. Consulte a documentação específica da função para maiores informações.
     db.arcRotation(5, 35, 25, 30, 25) #robot dirige 35° em um círculo com um raio de 5 cm medido a partir da borda interna do robô
     #remaining recursos do código são explicados em exemplos anteriores. Consulte exampleOne, exampleTwo e documentação adicional dentro de funções individuais
     return
            
def exampleFive():
            #adicione seu próprio código aqui
            
            
            return

def exampleSix():
            #adicione seu próprio código aqui
            
            
            return

class bcolors:
            BATTERY = '\033[32m'
            BATTERY_LOW = '\033[31m'

            ENDC = '\033[0m'

pReglerLight = 1.6
iReglerLight = 0.009
dReglerLight = 16

accelerate = True

hub2.button.right.callback(breakFunction)
gyroValue = 0


        #Battery voltage printout in console for monitoring charge
if battery.voltage() < 8000: #set threshold for battery level
            print(bcolors.BATTERY_LOW + "battery voltage is too low: " + str(battery.voltage()) + " \n ----------------------------- \n >>>> please charge robot <<<< \n ----------------------------- \n"+ bcolors.ENDC)
else:
            print(bcolors.BATTERY + "battery voltage: " + str(battery.voltage()) + bcolors.ENDC)


        #User Interface in Program for competition and instant program loading
main = True


programselect = 1 #Set the attachment the selection program starts on
hub.light_matrix.write(programselect)
db.movement_motors.set_stop_action("hold") #hold motors on wait for increased reliability



while main:
            cancel = False
            inMain = True

            #Program selection
            
            if hub.right_button.is_pressed(): #press right button to cycle through programs. cycling back isn't supported yet, but we are working on reallocating the buttons in the file system
                wait_for_seconds(0.15) #waiting prevents a single button press to be registered as multiple clicks
                programselect = programselect + 1
                hub.light_matrix.write(programselect) #show current selcted program
                hub.speaker.beep(85, 0.1) #give audio feedback for user

                if programselect == 1:
                    hub.status_light.on('blue')
                elif programselect == 2:
                    hub.status_light.on('black')
                elif programselect == 3:
                    hub.status_light.on('white')
                elif programselect == 4:
                    hub.status_light.on('white')
                elif programselect == 5:
                    hub.status_light.on('red')        
                elif programselect == 6:
                    hub.status_light.on('orange')
                #cycle to start of stack
                if programselect == 7:
                    programselect = 1
                    hub.light_matrix.write(programselect)
                    hub.status_light.on('blue')

            #Program start
            if hub.left_button.is_pressed():
                inMain = False

                if programselect == 1:
                    hub.status_light.on("blue")
                    hub.light_matrix.show_image("DUCK")
                    exampleOne()
                    programselect = 2
                    hub.light_matrix.write(programselect)
                elif programselect == 2:
                    hub.status_light.on("black")
                    hub.light_matrix.show_image("DUCK")
                    exampleTwo()
                    programselect = 3
                    hub.light_matrix.write(programselect)
                elif programselect == 3:
                    hub.status_light.on("white")
                    hub.light_matrix.show_image("DUCK")
                    exampleThree()
                    programselect = 5
                    hub.light_matrix.write(programselect)
                elif programselect == 4:
                    hub.status_light.on('white')
                    hub.light_matrix.show_image('DUCK')
                    exampleFour()
                    programselect = 5
                    hub.light_matrix.write(programselect)
                elif programselect == 5:
                    hub.status_light.on("red")
                    hub.light_matrix.show_image("DUCK")
                    exampleFive()
                    programselect = 6
                    hub.light_matrix.write(programselect)
                elif programselect == 6:
                    hub.status_light.on("orange")
                    hub.light_matrix.show_image("DUCK")
                    exampleSix()
                    programselect = 1
                    hub.light_matrix.write(programselect)


sys.exit("finalizou essa porra")
