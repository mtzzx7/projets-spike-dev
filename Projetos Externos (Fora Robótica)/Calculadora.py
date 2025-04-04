# Welcome function to greet the user
def welcome():
    print('''
Welcome to Calculator
''')

# Function to perform calculations based on user input
def calculate():
    # Prompt for user input
    a = float(input("Enter the first number: "))
    b = float(input("Enter the second number: "))

    # Fetch the operation to be performed
    operation = input('''
Please type in the math operation you would like to complete:
+ for addition
- for subtraction
* for multiplication
/ for division
** for power
% for modulo
''')

    # Perform calculations based on user input
    if operation == '+':
        print('{} + {} = '.format(a, b))
        print(a + b)

    elif operation == '-':
        print('{} - {} = '.format(a, b))
        print(a - b)

    elif operation == '*':
        print('{} * {} = '.format(a, b))
        print(a * b)

    elif operation == '/':
        if b != 0:
            print('{} / {} = '.format(a, b))
            print(a / b)
        else:
            print("Error: Division by zero is not allowed.")

    elif operation == '**':
        print('{} ** {} = '.format(a, b))
        print(a ** b)

    elif operation == '%':
        if b != 0:
            print('{} % {} = '.format(a, b))
            print(a % b)
        else:
            print("Error: Division by zero is not allowed.")

    else:
        print('Invalid operation. Please try again.')

    # Function to ask user if they want to calculate again
    def again():
        calc_again = input('''
Do you want to calculate again?
Please type Y for YES or N for NO.
''')

        if calc_again.upper() == 'Y':
            calculate()
        elif calc_again.upper() == 'N':
            print('See you later.')
        else:
            again()

    again()

# Call the welcome function
welcome()
# Call the calculate function
calculate()