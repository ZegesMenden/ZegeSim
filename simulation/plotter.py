from os import error
from simulation.dataManagement import dataVisualiser
import csv

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


class plotter:

    """Data visualization class"""

    def __init__(self) -> None:
        """initializes data visualizer"""

        self.data_descriptions: list = []
        self.header: str = ""
        self.viewer: dataVisualiser = dataVisualiser()

        self.n_plots: int = 0

    def read_header(self, file_name):

        f = open(file_name, "r")
        self.header = f.readline()
        self.data_descriptions = self.header.split(',')
        self.viewer.allDataDescriptions = self.data_descriptions

    def create_2d_graph(self, graph_points: list, x_desc: str = "", y_desc: str = "", annotate: bool = False):
        """Creates a 2-dimensional graph from the provided data descriptions.

        Params:

        graph_points - a list of all data point names to be graphed. First element of graph_points will be the x-axis datapoint.

        x_desc - label for the x axis of the graph.

        y_desc - label for the y axis of the graph.

        annotate - set true to show a color code for all lines on the graph."""

        self.n_plots += 1
        plt.figure(self.n_plots)

        plot_points = self.viewer.graph_from_csv(graph_points)

        for index, dataPoint in enumerate(plot_points):
            if index > 0:
                if annotate:
                    plt.plot(plot_points[0], dataPoint,
                             label=graph_points[index])
                else:
                    plt.plot(plot_points[0], dataPoint)
        plt.legend()
        plt.xlabel(x_desc)
        plt.ylabel(y_desc)

    def create_3d_graph(self, graph_points: list, size: float = 0.0, color: str = 'Blues'):
        """Creates a 3 dimensional graph from the provided data descriptions

        Params:

        graph_points - a list of all data point names to be graphed.

        size - the size of the graph in all axes"""

        if len(graph_points) != 3:
            raise error("graph_points must have 3 data points!")

        self.n_plots += 1
        plt.figure(self.n_plots)

        plot_points = self.viewer.graph_from_csv(graph_points)

        ax = plt.axes(projection='3d')

        if size != 0.0:

            ax.set_xlim3d(-size, size)
            ax.set_ylim3d(-size, size)
            ax.set_zlim3d(0, size)

        ax.scatter3D(plot_points[3], plot_points[2],
                     plot_points[1], c=plot_points[3], cmap=color)

    def show_all_graphs(self):
        """Displays all graphs."""
        plt.show()
