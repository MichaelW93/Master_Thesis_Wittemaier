import pygame
import numpy as np
import carla

from implementation.configuration_parameter import *


class PygameWindow(object):
    """Creates the Pygame Window, which is used as control panel for the user to change the simulation"""

    def __init__(self):

        self.pygame_display = None
        self.pygame_surface = None
        self.pygame_clock = None
        self.my_font = None

    def initialize_window(self) -> None:
        pygame.init()
        pygame.font.init()
        self.my_font = pygame.font.SysFont('FreeMono', 30, True)

        self.pygame_display = pygame.display.set_mode(
            (PYGAME_WINDOW_WIDTH, PYGAME_WINDOW_HEIGHT),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self.pygame_clock = pygame.time.Clock()

    def create_image(self, image: carla.Image) -> None:

        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.pygame_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):

        if self.pygame_surface is not None:
            display.blit(self.pygame_surface, (0, 0))

