import argparse
import pygame
from libs.robust_serial.robust_serial import read_order, write_order, Order, read_i32, write_i16, read_i8
from libs.robust_serial.utils import open_serial_port

import struct
import time

import collections

# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

NUM_AXES = 2
NUM_HATS = 2

# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


class ArduinoTester():
    def __init__(self, input_joy_name, fino_joy_name, fino_com_port):
        self.input_joy_name = input_joy_name
        self.fino_joy_name = fino_joy_name
        self.fino_com_port = fino_com_port
        self.pos = [0] * NUM_AXES
        self.hats = [0] * NUM_HATS
        self.force = (0, 0)
        self.semaphore = 10
        try:
            self.serial_file = open_serial_port(self.fino_com_port, baudrate=115200)
        except Exception as e:
            raise e

    def connect_fino(self):
        serial_file = self.serial_file
        is_connected = False
        # Initialize communication with Arduino
        while not is_connected:
            print("Waiting for Arduino...")
            self.write_order(Order.HELLO)
            bytes_array = bytearray(serial_file.read(1))
            if not bytes_array:
                time.sleep(2)
                continue
            byte = bytes_array[0]
            if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
                is_connected = True
        self.semaphore = 10
        print("Connected to Arduino")

    def write_order(self, order):
        if self.semaphore > 0:
            write_order(self.serial_file, order)
            self.semaphore -= 1

    # send a request to the Arduino to send us the forces soon
    def request_forces(self):
        self.write_order(Order.FORCES)

    # read the forces the Arduino is sending us right now
    def read_forces(self):
        force_x = read_i32(self.serial_file)
        force_y = read_i32(self.serial_file)
        self.force = (force_x, force_y, 0, 0)
        return self.force

    @staticmethod
    # accept a number in the range -1.0 to 1.0 and
    # return a number in the range -32767,32767
    # for axis position
    def translate_brunner_pos(pos):
        return int(pos * 32767)

    @staticmethod
    def translate_hat(a):
        th = {
            (0, 0): -1,
            (0, 1): 0,
            (1, 1): 45,
            (1, 0): 90,
            (1, -1): 135,
            (0, -1): 180,
            (-1, -1): 225,
            (-1, 0): 270,
            (-1, 1): 315
        }
        return th[(a[0], a[1])]

    def send_position(self):
        self.write_order(Order.POSITION)
        # print("orig: " + str(self.pos[0]) + "," + str(self.pos[1]))
        for i in range(NUM_AXES):
            write_i16(self.serial_file, self.pos[i])
        for i in range(NUM_HATS):
            write_i16(self.serial_file, self.hats[i])

    # read the log the Arduino is sending us right now
    def read_log(self):
        return self.serial_file.readline().decode('ascii')

    # do we have messages from the Arduino
    @property
    def in_waiting(self):
        return self.serial_file.in_waiting

    @staticmethod
    def list_joysticks():
        joystick_count = pygame.joystick.get_count()
        # For each joystick:
        for i in range(joystick_count):
          joystick = pygame.joystick.Joystick(i)
          print(joystick.get_name())

    @staticmethod
    def get_joystick(joystick_name):
        joystick_count = pygame.joystick.get_count()

        # For each joystick:
        for i in range(joystick_count):
          joystick = pygame.joystick.Joystick(i)
          if joystick.get_name() == joystick_name:
              return joystick


    def loop(self):
        pygame.init()
        input_joy = self.get_joystick(self.input_joy_name)
        fino_joy = self.get_joystick(self.fino_joy_name)
        hat_num = 0

        if input_joy is None:
            self.list_joysticks()
            return 0

        # Set the width and height of the screen (width, height).
        screen = pygame.display.set_mode((500, 200))
        # Loop until the user clicks the close button.
        done = False
        # Used to manage how fast the screen updates.
        clock = pygame.time.Clock()
        # Initialize the joysticks.
        pygame.joystick.init()

        # Get ready to print.
        textPrint = TextPrint()
        size = width, height = 320, 230
        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Read joystick positions")
        bg = (200, 200, 250, 100)
        red = (255, 0, 0, 100)
        white = (255, 255, 255, 100)
        rect = pygame.Rect(5, 75, 150, 150)

        last_pressed = False

        while not done:

            for event in pygame.event.get(): # User did something.
                if event.type == pygame.QUIT: # If user clicked close.
                    done = True # Flag that we are done so we exit this loop.

            while self.in_waiting:
                try:
                    order = read_order(self.serial_file)
                except ValueError:
                    order = None

                if order == Order.RECEIVED:
                    self.semaphore += 1
                elif order == Order.FORCES:
                    self.read_forces()
                elif order == Order.LOG:
                    log_line = self.read_log()
                    print(log_line)

            screen.fill(WHITE)
            textPrint.reset()

            input_joy.init()

            textPrint.tprint(screen, "Joystick")
            textPrint.indent()


            for i in range(NUM_AXES):
                a = input_joy.get_button(0)
                raw_pos = input_joy.get_axis(i % 4)
                fino_pos = fino_joy.get_axis(i)

                if (i < 4 and not a) or (i>=4 and a):
                    translated_pos = self.translate_brunner_pos(raw_pos)
                    self.pos[i] = translated_pos
                else:
                    translated_pos = self.pos[i]

                textPrint.tprint(screen, "Axis {} value: {:>6.3f} > {:>6.3f}".format(i, translated_pos, fino_pos))

            for i in range(NUM_HATS):
                fino_pos = fino_joy.get_hat(i)
                translated_fino_pos = self.translate_hat(fino_pos)
                textPrint.tprint(screen, "Hat {} value: {} > {}".format(i, self.hats[i], translated_fino_pos))

            l1 = input_joy.get_button(4)
            r1 = input_joy.get_button(5)

            if (l1 or r1):
                if not last_pressed:
                    last_pressed = True
                    hat_num = hat_num + (1 if r1 else -1)
                    hat_num = 0 if hat_num < 0 else hat_num % NUM_HATS
            else:
                last_pressed = False

            # print(str(l1) + str(r1) + str(self.translate_hat(input_joy.get_hat(0))))
            if hat_num <= len(self.hats) - 1:
                self.hats[hat_num] = self.translate_hat(input_joy.get_hat(0))

            # print(input_joy.get_hat(0))
            self.send_position()

            # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
            #
            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()
            # Limit to 20 frames per second.
            clock.tick(50)
        pygame.quit()


# This is a program for testing all hats and axes of a Fino device
# it accepts as input an XBOX controller
# Axes 1 to 4 are controlled by the left and right sticks
# Axes 5 to 8 require keeping pressed the a button

# The hats are controlled with the dpads
# To control the prev/next hat, change between them with l1/r1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Emulate CLS2Sim with a different joystick')
    parser.add_argument('joy_name', help='which joystick to use as input')
    parser.add_argument('fino_joy_name', help='which joystick to use as output')
    parser.add_argument('fino_com_port', help='which port to use as output')
    args = parser.parse_args()
    tester = ArduinoTester(args.joy_name, args.fino_joy_name, args.fino_com_port)
    tester.connect_fino()
    tester.loop()
