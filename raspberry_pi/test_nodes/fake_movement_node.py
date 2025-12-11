from signal_state_machine import *

while True:
    print("1 = Greeting")
    print("2 = Move")
    print("3 = Start Move")
    print("4 = Stop Move")
    print("5 = Reverse")
    print("6 = Error Major")
    choice = input("> ")

    if choice == "1":
        greeting()
    if choice == "2":
        send_state(6)
    if choice == "3":
        send_state(7)
    if choice == "4":
        send_state(8)
    if choice == "5":
        reverse()
    if choice == "6":
        error_major()

