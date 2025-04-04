from tkinter import *
import tkinter
from datetime import datetime

# Define colors
cor1 = "#3d3d3d"
cor2 = "#fafcff"

# Define window properties
fundo = cor1
cor = cor2
janela = Tk()
janela.title("Digital Clock")
janela.geometry("440x180")
janela.resizable(width=FALSE, height=FALSE)
janela.configure(bg=cor1)

# Define delay time for the after method
DELAY = 1000

# Define a function to create and configure the labels
def create_label(row, column, text, font_size, font_name):
    label = Label(janela, text=text, font=(font_name, font_size), bg=fundo, fg=cor)
    label.grid(row=row, column=column, sticky=NW, padx=5)
    return label

# Define the relogio function
def relogio():
    tempo = datetime.now()

    # Format the hour and minute
    hora = tempo.strftime("%H:%M")

    # Format the day of the week and day of the month
    dia_semana = tempo.strftime("%A")
    dia = tempo.day
    day_of_week = {
        "Monday": "Segunda-feira",
        "Tuesday": "Terça-feira",
        "Wednesday": "Quarta-feira",
        "Thursday": "Quinta-feira",
        "Friday": "Sexta-feira",
        "Saturday": "Sábado",
        "Sunday": "Domingo"
    }
    dia_semana = day_of_week[dia_semana]

    # Update the labels
    l1.config(text=hora)
    l2.config(text=f"{dia_semana}, {dia}")
    janela.after(DELAY, relogio)

# Create and configure the labels
l1 = create_label(10, 0, "", 80, "Arial")
l2 = create_label(0, 0, "10:50:22", 20, "Arial")

# Start the clock
relogio()
janela.mainloop()