import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt


class PlotData(Node):
    def __init__(self):
        super().__init__('data_plot_node')
        print("data_plot_node create")



def main():
    rclpy.init()
    plot_node = PlotData()

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print('Interrupted by user, shutting down...')
    finally:
        rclpy.shutdown() #防止ctrl+c 失效


if __name__ == '__main__':
    main()
