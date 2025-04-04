# Criação da função
def welcome():
    print("Seja bem vindo a minha conversora de unidades")

welcome()
x = 0
med1 = (input("Qual medida você está usando?"))
med2 = (input("Para qual você quer passa-la?")) 

if (med1 == "metro" and med2 == "cm"):
    resultado = float(input("Qual o valor?")) * 100
elif(med1 == "cm" and med2 == "metro"):
    resultado = float(input("Qual o valor?")) / 100
elif(med1 == "dm" and med2 == "cm"):
    resultado = float(input("Qual o valor?")) * 10
elif(med1 == "cm" and med2 == "dm"):
    resultado = float(input("Qual o valor?")) / 10
elif(med1 == "m" and med2 == "mm"):
    resultado = float(input("Qual o valor?")) * 1000
elif(med1 == "mm" and med2 == "m"):
    resultado = float(input("Qual o valor?")) / 1000
elif(med1 == "m" and med2 == "km"):
    resultado = float(input("Qual o valor?")) / 1000
elif(med1 == "km" and med2 == "m"):
    resultado = float(input("Qual o valor?")) * 1000
else:
    x = 1

if(x == 0):
    print(resultado,med2)

else:
    print("Conversão Inválida")