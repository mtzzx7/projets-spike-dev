import tkinter

cor = "white"
janela = tkinter.Tk()
janela.geometry ("800x400")
janela.title("THUGNINE")
janela.configure(bg=cor)

def entrar():
    janela3 = tkinter.Tk()
    janela3.geometry ("800x400")
    janela3.title("THUGNINE")
    janela3.configure(bg=cor)

    texto2 = tkinter.Label(janela3, text="Entrar", font=16)
    texto2.pack(padx=10, pady=10)

    email = tkinter.Entry(janela3)
    email.pack(padx=10, pady=10)

    senha = tkinter.Entry(janela3, show="*")
    senha.pack(padx=10, pady=10)

    button3 = tkinter.Button(janela3, text="Entrar", command=button)
    button3.pack(pady=5, padx=10)

def button():
    print("Seja Bem vindo")
    janela2 = tkinter.Tk()
    janela2.geometry ("800x400")
    janela2.title("THUGNINE")
    janela2.configure(bg=cor)

    texto1 = tkinter.Label(janela2, text="Crie sua conta", font=16)
    texto1.pack(padx=10, pady=10)

    email = tkinter.Entry(janela2)
    email.pack(padx=10, pady=10)

    senha = tkinter.Entry(janela2, show="*")
    senha.pack(padx=10, pady=10)

    button2 = tkinter.Button(janela2, text="Concluir", command=entrar)
    button2.pack(pady=5, padx=10)


def button3():
    if email.get() == "felippop007@gmail.com" or senha.get() == "123":
        print("Acessado com sucesso")




texto = tkinter.Label(janela, text="Entre na sua conta", font=16)
texto.pack(padx=10, pady=10)

email = tkinter.Entry(janela)
email.pack(padx=10, pady=10)

senha = tkinter.Entry(janela, show="*")
senha.pack(padx=10, pady=10)

button = tkinter.Button(janela, text="Criar conta", command=button)
button.pack(pady=5, padx=10)

button3 = tkinter.Button(janela, text="Entrar", command=button3)
button3.pack(pady=5, padx=10)










janela.mainloop()
janela2.mainloop()
janela3.mainloop()

