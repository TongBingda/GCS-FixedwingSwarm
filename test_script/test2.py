import PySimpleGUI as sg

if __name__ == "__main__":
    dict = {"1":True, "2":False, "3":True}
    print(all(value == True for value in dict.values()))
    a = print("!")