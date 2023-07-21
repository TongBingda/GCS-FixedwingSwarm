#!/usr/bin/env python
import PySimpleGUI as sg
from random import randint
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, FigureCanvasAgg
from matplotlib.figure import Figure

def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

def main():
    NUM_DATAPOINTS = 10000
    # define the form layout
    layout = [[sg.Text('Animated Matplotlib', size=(40, 1),
                justification='center', font='Helvetica 20')],
              [sg.Canvas(size=(640, 480), key='-CANVAS-')],
              [sg.Button('Exit', size=(10, 1), pad=((280, 0), 3), font='Helvetica 14'), sg.Button("Plot")]]

    # create the form and show it without the plot
    window = sg.Window('Demo Application - Embedding Matplotlib In PySimpleGUI',
                layout, finalize=True)
    return window

if __name__ == '__main__':
    window = main()
    canvas_elem = window['-CANVAS-']
    canvas = canvas_elem.TKCanvas

    # draw the initial plot in the window
    fig = Figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.grid()
    fig_agg = draw_figure(canvas, fig)

    vehicles_lat = {}
    vehicles_lon = {}

    vehicles_lat["vehicle1"] = []
    vehicles_lon["vehicle1"] = []

    while True:
        event, values = window.read(timeout=100)
        if event in ('Exit', None):
            break

        vehicles_lat["vehicle1"].append(randint(0,10))
        vehicles_lon["vehicle1"].append(randint(0,10))
        if len(vehicles_lat["vehicle1"]) > 50:
            vehicles_lat["vehicle1"].pop(0)
            vehicles_lon["vehicle1"].pop(0)
        ax.cla()
        ax.grid()
        ax.plot(vehicles_lon["vehicle1"], vehicles_lat["vehicle1"], color="purple")
        ax.plot(vehicles_lat["vehicle1"], vehicles_lon["vehicle1"], color="blue")
        fig_agg.draw()
