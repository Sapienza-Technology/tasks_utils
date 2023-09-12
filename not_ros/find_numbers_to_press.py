""""
Ask for input of 8 numbers and a desired sum and find a combination of 5 numbers that add up to the desired sum.
3 numbers have to be selected among the first 4 numbers and 2 numbers have to be selected among the last 4 numbers.
"""

import itertools

def find_numbers_to_press():
    numbers = []
    for i in range(8):
        numbers.append(int(input("Enter number: ")))
    desired_sum = int(input("Enter desired sum: "))

    #printing  a panel with the buttons received in input
    print("Panel:")
    print("  ---------------------")
    print("| ______ \t ______  |")
    print("| |  ▣  |\t |  |  | |")
    print("| ------ \t ------  |")
    print("|        \t         |")
    print("| ______ \t ______  |")
    print("| |{}|{}|\t |{}|{}| |".format(numbers[0], numbers[1], numbers[2], numbers[3]))
    print("| ------ \t ------- |")
    print("|        \t         |")
    print("| ______ \t ______  |")
    print("| |{}|{}|\t |{}|{}| |".format(numbers[4], numbers[5], numbers[6], numbers[7]))
    print("| ------ \t ------  |")
    print("|        \t         |")
    print("  ---------------------")


    combinations = list(itertools.combinations(numbers, 5))
    for combination in combinations:
        if sum(combination) == desired_sum:
            return combination
    return None

if __name__ == "__main__":
    to_press=find_numbers_to_press()
    if to_press is None:
        print("No combination of 5 numbers adds up to the desired sum")
    else:
        print("Press the following buttons: {}".format(to_press))

