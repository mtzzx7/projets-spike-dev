import random
import string

def generate_password(length=8):
    """Gera uma senha aleatória de tamanho especificado."""
    # Define o intervalo de caracteres para escolher
    characters = string.ascii_letters + string.digits + string.punctuation
    
    # Gera a senha
    password = ''.join(random.choice(characters) for _ in range(length))
    
    return password

# Gera e imprime uma senha de 8 caracteres
print(generate_password())