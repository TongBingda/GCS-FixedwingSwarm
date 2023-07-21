import PySimpleGUI as sg

if __name__ == "__main__":
    sg.theme('Dark Grey 13')

    layout = [[sg.Text('Filename')],
            [sg.Input(), sg.FileBrowse()],
            [sg.OK(), sg.Cancel()]]

    window = sg.Window('Get filename example', layout, use_custom_titlebar=True)

    event, values = window.read()
    window.close()

    text_input = values[0]    
    sg.popup('You entered', text_input)