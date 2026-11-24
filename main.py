import serial, time, math, atexit, pygame

PORT = "COM15"
BAUDRATE = 460800
WHITE = (255, 255, 255)
BLACK = (0,0,0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

BACKGROUND_COLOR = BLACK
CIRCLE_COLOR = GREEN
CIRCLE_RAD = 2

ser: Serial | None = None
screen = None

window_size = 800

def main():
    global ser, screen

    ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=1)

    pygame.init()
    screen = pygame.display.set_mode((window_size, window_size), pygame.SCALED)
    pygame.display.set_caption("LIDAR C1 Stream Viewer")
    pygame.draw.circle(screen, RED, (window_size/2, window_size/2), 5)

    ser.write(b'\xA5\x40') # Reset Command
    time.sleep(.1)
    ser.reset_input_buffer()
    ser.write(b'\xA5\x20') # Start Command
    time.sleep(.1)

    bytes_window = bytearray(5)

    recent_points = []
    data_recieved = False
    fresh_packet = True
    while True:
        while ser.in_waiting >= 5:



