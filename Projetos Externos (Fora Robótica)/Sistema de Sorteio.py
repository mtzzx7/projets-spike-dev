import random

# Define the number of lottery numbers to generate
num_numbers = 10

# Define the range of numbers to choose from
start = 1
end = 100

# Generate the lottery numbers
lottery_numbers = random.sample(range(start, end + 1), num_numbers)

# Print the lottery numbers
adivinha = str(input("Qual você acha que são?"))

if adivinha == str(lottery_numbers):
    print("Você acertou em cheio!")
    print("lottery numbers:")
    print(sorted(lottery_numbers))
else: 
    print("Você errou")
    print("lottery numbers:")
    print(sorted(lottery_numbers))