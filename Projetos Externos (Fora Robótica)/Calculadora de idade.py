# Função de boas-vindas
def welcome():
    print("Seja bem vindo ao meu sistema de idades")

#Chama a função para o console
welcome()

a = float(input("Em que ano você nasceu?"))
ano = 2024
idade = ano - a
mensagem = "nossa você tem " +  str (idade) +  " anos"

print(mensagem)