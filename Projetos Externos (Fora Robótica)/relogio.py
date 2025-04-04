from tkinter import*
import tkinter
from datetime import datetime

cor1 = "#3d3d3d"
cor2 = "#fafcff"


fundo = cor1
cor = cor2

janela = Tk()
janela.title("")
janela.geometry("440x180")
janela.resizable(width=FALSE, height=FALSE)
janela.configure(bg=cor1)


def relogio():
    tempo = datetime.now()

    hora = tempo.strftime("%H:%M")

    dia_semana = tempo.strftime("%A")
    dia = tempo.day

    l1.config(text=hora)
    l1.after(200, relogio)
    l2.config(text=dia_semana + str(dia))

l1 = Label(janela, text="", font=("Arial 80"), bg=fundo, fg=cor)
l1.grid(row=10, column=0, sticky=NW, padx=5)

l2 = Label(janela, text="10:50:22", font=("Arial 20"), bg=fundo, fg=cor)
l2.grid(row=0, column=0, sticky=NW, padx=5)



relogio()
janela.mainloop()
