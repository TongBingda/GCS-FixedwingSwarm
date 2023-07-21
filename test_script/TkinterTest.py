import PySimpleGUI as sg

def forget(self, child):
    self.Widget.forget(str(child.Widget))

def tab(index):
    layout = [
        [sg.VPush()],
        [sg.Push(), sg.Text(f"Here's Tab {index+1}", background_color='green'), sg.Push()],
        [sg.VPush()],
    ]
    return sg.Tab(f'Tab {index+1}', layout, key=f'Tab {index+1}', expand_x=True, expand_y=True)

sg.TabGroup.forget = forget

sg.theme("DarkBlue3")
sg.set_options(font=('Courier New', 16), dpi_awareness=True)
tabs = 6

tab_group = [[tab(i) for i in range(tabs)]]
layout = [
    [sg.TabGroup(tab_group, size=(600, 300), key='TABGROUP')],
    [sg.Push(), sg.Button('Remove TAB'), sg.Push()],
]

window = sg.Window("Title", layout, finalize=True)

while True:

    event, values = window.read()
    if event == sg.WINDOW_CLOSED:
        break
    elif event == 'Remove TAB' and tabs > 0:
        window['TABGROUP'].forget(window[f'Tab {tabs}'])
        tabs -= 1

window.close()