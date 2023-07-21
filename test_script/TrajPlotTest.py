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
              [sg.Text('Progress through the data')],
              [sg.Slider(range=(0, NUM_DATAPOINTS), size=(60, 10),
                orientation='h', key='-SLIDER-')],
              [sg.Text('Number of data points to display on screen')],
               [sg.Slider(range=(10, 500), default_value=40, size=(40, 10),
                    orientation='h', key='-SLIDER-DATAPOINTS-')],
              [sg.Button('Exit', size=(10, 1), pad=((280, 0), 3), font='Helvetica 14'), sg.Button("Plot")]]

    # create the form and show it without the plot
    window = sg.Window('Demo Application - Embedding Matplotlib In PySimpleGUI',
                layout, finalize=True)
    return window

if __name__ == '__main__':
    NUM_DATAPOINTS = 10000
    window = main()
    canvas_elem = window['-CANVAS-']
    slider_elem = window['-SLIDER-']
    canvas = canvas_elem.TKCanvas

    # draw the initial plot in the window
    fig = Figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.grid()
    fig_agg = draw_figure(canvas, fig)
    # make a bunch of random data points
    dpts = [randint(0, 10) for x in range(NUM_DATAPOINTS)]

    # while True:
    ax.plot([1,2,3], [4,5,6])
    print(randint(0,10))
    while True:
        ax.plot([randint(0,10),randint(0,10)], [randint(0,10),randint(0,10)])
        time.sleep(1)
    # while True:
    #     event, values = window.read()
    #     if event in ('Exit', None):
    #         exit(69)
    #     else:
    #         ax.plot([1,2,3],[6,5,4])
            # ax.plot([randint(0,10),randint(0,10),randint(0,10)], [randint(0,10),randint(0,10),randint(0,10)])
    # for i in range(len(dpts)):

    #     event, values = window.read(timeout=10)
    #     if event in ('Exit', None):
    #         exit(69)
    #     slider_elem.update(i)       # slider shows "progress" through the data points
    #     ax.cla()                    # clear the subplot
    #     ax.grid()                   # draw the grid
    #     data_points = int(values['-SLIDER-DATAPOINTS-']) # draw this many data points (on next line)
    #     ax.plot(range(data_points), dpts[i:i+data_points],  color='purple')
    #     fig_agg.draw()