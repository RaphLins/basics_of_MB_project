import time
import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from utils import get_pattern, get_ground_sensor_pos


class Interface:
    def __init__(self, robot_path, callbacks={}, display_rate=0.3, pattern_file=None):
        self.obstacle_map = get_pattern(pattern_file)
        self.robot_path = robot_path
        matplotlib.use("TkAgg")

        self.map_fig = plt.figure()
        self.map_ax = self.map_fig.add_subplot(111)
        plt.subplots_adjust(bottom=0.2)
        ctrl_ax = plt.axes([0.7, 0.05, 0.1, 0.075])
        self.pause_button = Button(ctrl_ax, "Stop")

        if "pause" in callbacks:
            self.pause_button.on_clicked(callbacks["pause"])

        self.map_ax.imshow(get_pattern(), cmap="Greys", origin="lower")
        self.map_ax.set_xlim(-50, 500)
        self.map_ax.set_ylim(-50, 400)

        self.display_rate = display_rate
        self.last_update = time.time()

        plt.draw()
        plt.show(block=False)

    def draw_particles(self, particle_pos_list):
        for particle in particle_pos_list:
            x, y, theta = particle
            self.map_ax.scatter(x, y, c="m")

    def draw_thymio(self, thymio_pos):
        x, y, theta = thymio_pos
        ground_left, ground_right = get_ground_sensor_pos(thymio_pos)
        length = 30
        self.map_ax.scatter(x, y, c="b")
        self.map_ax.scatter(ground_left[0], ground_left[1], c="c")
        self.map_ax.scatter(ground_right[0], ground_right[1], c="c")
        self.map_ax.plot((x, x + length * math.cos(theta)), (y, y + length * math.sin(theta)), "b")

    def update_viz(self, current_pos, target_pos, particle_pos_list):
        current_time = time.time()
        if current_time - self.last_update > self.display_rate:
            self.map_ax.clear()
            self.map_ax.imshow(self.obstacle_map, cmap="Greys", origin="lower")
            self.map_ax.set_xlim(-100, 900)
            self.map_ax.set_ylim(0, 700)
            for point in self.robot_path:
                x, y, theta = point
                self.map_ax.scatter(x, y, c="r")
            x, y, theta = target_pos
            self.map_ax.scatter(x, y, c="g")
            self.draw_particles(particle_pos_list)
            self.draw_thymio(current_pos)
            plt.draw()
            plt.pause(0.001)
