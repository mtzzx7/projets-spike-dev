import customtkinter as ctk

# Create the main window
window = ctk.CTk()
window.title("Unit Converter")
window.geometry("600x400")


# Create a function to handle the conversion
def convert_units():
    value = float(entry_value.get())
    unit_from = combo_from.get()
    unit_to = combo_to.get()

    if unit_from == "Metrô" and unit_to == "Cm":
        result = value * 100
    elif unit_from == "Cm" and unit_to == "Metrô":
        result = value / 100
    elif unit_from == "Cm" and unit_to == "Dm":
        result = value / 10
    elif unit_from == "Dm" and unit_to == "Cm":
        result = value * 10
    elif unit_from == "Km" and unit_to == "Metrô":
        result = value * 1000
    elif unit_from == "Metrô" and unit_to == "Km":
        result = value / 1000
    elif unit_from == "Km" and unit_to == "Dm":
        result = value * 10000
    elif unit_from == "Dm" and unit_to == "Km":
        result = value / 10000
    elif unit_from == "Km" and unit_to == "Cm":
        result = value * 100000
    elif unit_from == "Cm" and unit_to == "Km":
        result = value / 100000
    elif unit_from == "Metrô" and unit_to == "Dm":
        result = value * 10
    elif unit_from == "Dm" and unit_to == "Metrô":
        result = value / 10
    elif unit_from == "Km" and unit_to == "Metrô":
        result = value * 1000
    elif unit_from == "Metrô" and unit_to == "Km":
        result = value / 1000
    else:
        result = "Invalid conversion"

    label_result.configure(text=str(result) + str(unit_to))

# Create the UI elements
entry_value = ctk.CTkEntry(window)
entry_value.pack(padx=10, pady=10)

combo_from = ctk.CTkComboBox(window, values=["Metrô", "Dm", "Km", "Cm"])
combo_from.pack(padx=10, pady=10)

combo_to = ctk.CTkComboBox(window, values=["Metrô", "Dm", "Km", "Cm"])
combo_to.pack(padx=10, pady=10)

button_convert = ctk.CTkButton(window, text="Convert", command=convert_units)
button_convert.pack(padx=10, pady=10)

label_result = ctk.CTkLabel(window, text="")
label_result.pack(padx=10, pady=10)

# Run the main loop
window.mainloop()