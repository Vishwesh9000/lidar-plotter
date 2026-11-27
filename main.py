import serial, time, math, atexit, signal
import multiprocessing as mp
from multiprocessing import shared_memory as mp_shared_memory
from multiprocessing.synchronize import Event, Lock
import numpy as np
# import pygame # Imported in main to prevent re-import in child process

PORT = "COM15"
BAUDRATE = 460800
WHITE = (255, 255, 255)
BLACK = (0,0,0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

BACKGROUND_COLOR = BLACK

SCALING_FACTOR = 0.1

SHM_NAME = "LIDAR_DATA"

BUFFER_SHAPE = (700, 3)
DTYPE = np.float32

screen = None

window_size = 800

grid_subdivisions = 100
grid_length_mm = 10_000 # mm


class RPLidar:
    # RPLIDAR Packet sytax:
    # 5 bytes
    # b[0]:
    #   b[0][0]: Start bit (1 if first reading after passing 0 degrees else 0
    #   b[0][1] != b[0][0]: Check bit
    #   b[0][2:]: Quality
    # b[1]:
    #   b[1][0] == 1 (Check bit)
    #   b[1][1:]: Angle lower 7 bits
    # b[2]:
    #   b[2][:]: Angle higher 8 bits
    # b[3]:
    #   b[3][:]: Distance lower 8 bits
    # b[4]:
    #   b[4][:]: Distance higher 8 bits
    #
    # Quality value: 0-63 (0 if  invalid reading/no return)
    # Angle value: transmitted as 1/64th degrees (max value: 360*64 - 1)
    # Distance value: transmitted as 1/4th mm's (0 if did not return/invalid reading) (Practical range: 50mm to 12,000mm)
    def __init__(self):
        self.BUFFER_SIZE = int(np.prod(BUFFER_SHAPE) * np.dtype(DTYPE).itemsize)

        self.shm: mp_shared_memory.SharedMemory | None = self.create_shared_memory(SHM_NAME, self.BUFFER_SIZE)

        self.data: np.ndarray[tuple[int, int], np.dtype[np.float32]] = np.ndarray(BUFFER_SHAPE, dtype=DTYPE, buffer=self.shm.buf)
        self.data.fill(0)

        self.lock = mp.Lock()
        self.shut_down_event = mp.Event()
        self.data_stream_active_event = mp.Event()

        self.proc = mp.Process(target=RPLidar.serial_reader, args=(self.shm.name, self.lock, self.shut_down_event, self.data_stream_active_event))
        self.proc.start()

        atexit.register(self.free_shared_memory) # Just in case, even though context manager handles it
    def __enter__(self):
        return self

    def __exit__(self, *args: object):
        self.stop_process()
        self.free_shared_memory()

    def create_shared_memory(self, name: str, size: int):
        try:
            shm = mp_shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shm = mp_shared_memory.SharedMemory(name=name)
            shm.close()
            shm.unlink()
            shm = mp_shared_memory.SharedMemory(name=name, create=True, size=size)
        return shm

    @staticmethod
    def serial_reader(name: str, lock: Lock, shut_down_event: Event, data_stream_active_event: Event):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        parent = mp.parent_process()
        shm = mp_shared_memory.SharedMemory(name=name, create=False)
        data: np.ndarray[tuple[int, int], np.dtype[np.float32]] = np.ndarray(BUFFER_SHAPE, DTYPE, buffer=shm.buf)
        ser: serial.Serial | None = None
        index: int = 0
        incoming_bytes = bytearray()
        try: # Ensures clean shutdown if error occurs
            while not shut_down_event.is_set() and (parent is None or parent.is_alive()): # continously read serial
                if ser is None: # if serial is not connected
                    data_stream_active_event.clear()
                    try: # Try to reconnect and catch SerialException if failed
                        ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=0)

                        ser.write(b"\xA5\x25")
                        time.sleep(.2)
                        ser.reset_input_buffer()
                        ser.write(b"\xA5\x20")
                        time.sleep(0.1)
                        # Catch 7-byte descriptor on startup: "A5 5A 05 00 00 40 81"
                        ser.read(7)

                    except serial.SerialException:
                        print(f"Unable to connect to {PORT}. Retrying...")
                        time.sleep(2)
                        ser = None
                        continue

                    data_stream_active_event.set()

                try:
                    b = ser.read(4096) # Read all data available (Chunking to improve speed)
                    # print(b)
                except serial.SerialException as e:
                    print("ERROR READING SERIAL:", e)
                    RPLidar.close_serial(ser)
                    ser = None
                    print("Serial port disconnected")
                    incoming_bytes = bytearray()
                    b = bytes()
                    print("Deleted stored bytes")
                    print("Attempting to reconnect...")
                    continue

                incoming_bytes += b
                batch: list[tuple[float, float, float]] = []
                byte_index = 0
                while byte_index + 5 <= len(incoming_bytes):
                    # Decode and validate packet
                    if (incoming_bytes[byte_index + 0] & 0x01) == ((incoming_bytes[byte_index + 0] >> 1) & 0x01):
                        byte_index += 1
                        print("Invalid Packet: Matching inverse start bits")
                        continue

                    if incoming_bytes[byte_index + 1] & 0x01 == 0:
                        byte_index += 1
                        print("Invalid Packet: Check bit invalid")
                        continue

                    quality = int(incoming_bytes[byte_index + 0] >> 2)

                    angle = float((incoming_bytes[byte_index + 1] >> 1) | (incoming_bytes[byte_index + 2] << 7)) / 64
                    if angle >= 360:
                        byte_index += 1
                        print(f"Invalid Packet: Angle ({angle:.3f}) >= 360 degrees")
                        continue

                    distance = float((incoming_bytes[byte_index + 3]) | (incoming_bytes[byte_index + 4] << 8))/4
                    if (distance < 30 or distance > 12_000) and distance != 0:
                        print(f"WARNING: Distance ({distance:.3f}) is out of expected range (50-12,000)")


                    # Chunk data in local buffer to avoid aquiring lock in a tight loop
                    batch.append((quality, angle, distance))
                    byte_index += 5

                if batch:
                    n = len(batch)
                    if n >= BUFFER_SHAPE[0]:
                        batch = batch[-BUFFER_SHAPE[0]:] # Don't need to consider the length of deleted points because alignment is arbitary
                        n = len(batch)
                    with lock:
                        if index + n > BUFFER_SHAPE[0]:
                            data[index:] = batch[ : BUFFER_SHAPE[0]-index]
                            data[:index+n-BUFFER_SHAPE[0]] = batch[BUFFER_SHAPE[0]-index : ]
                        else:
                            data[index:index+n] = batch
                    index = (index + n) % BUFFER_SHAPE[0]

                del incoming_bytes[:byte_index]
                time.sleep(0.01)
        except Exception as e:
            print("UNEXPECTED ERROR:", e)
        finally:
            print("Process shutting down...")
            RPLidar.close_serial(ser)
            shm.close()
            print("Process gracefully ended")

    @staticmethod
    def close_serial(ser: serial.Serial | None):
        if ser is not None and ser.is_open:
            ser.write(b"\xA5\x25")
            print("Stopped the LIDAR")
            time.sleep(0.1)
            ser.close()

    def is_active(self):
        return self.data_stream_active_event.is_set()

    def get_data(self):
        """Returns the 700 most recent points in circular buffer order (chronological except across the wrap-around seam; arbitrary start angle)."""
        with self.lock:
            return self.data.copy()

    def free_shared_memory(self):
        if self.shm is not None:
            self.shm.close()
            self.shm.unlink()
            self.shm = None
            print("Freed shared memory")

    def stop_process(self):
        print("Stopping lidar monitoring...")
        self.shut_down_event.set()

        print("Waiting for process to stop")
        self.proc.join(timeout=2)

        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join()
            print("Process didn't respond. Process forcefully shut down")

def main(lidar: RPLidar):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((window_size, window_size), pygame.SCALED)
    grid_surface = pygame.Surface((grid_subdivisions, grid_subdivisions), depth=24)
    pygame.display.set_caption("LIDAR C1 Stream Viewer")
    pygame.draw.circle(screen, RED, (window_size/2, window_size/2), 5)

    grid: np.ndarray[tuple[int, int], np.dtype[np.float32]] = np.zeros((grid_subdivisions, grid_subdivisions), np.float32) # [y, x] ordering; top left is [0,0]; values: 0-1
    rgba: np.ndarray[tuple[int, int, int], np.dtype[np.uint8]] = np.zeros((grid_subdivisions, grid_subdivisions, 4), np.uint8)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill(BACKGROUND_COLOR)
        pygame.draw.circle(screen, RED, (window_size/2, window_size/2), 5)
        data = lidar.get_data()
        for quality, angle, distance in data:
            quality, angle, distance = float(quality), float(angle), float(distance)
            if distance == 0 or quality == 0:
                continue

            x = math.cos(math.radians(90-angle))*distance
            y = -math.sin(math.radians(90-angle))*distance # y axis points downward

            i_x: int = math.floor(x*grid_subdivisions/grid_length_mm + grid_subdivisions/2)
            i_y: int = math.floor(y*grid_subdivisions/grid_length_mm + grid_subdivisions/2)
            if i_x in range(grid_subdivisions) and i_y in range(grid_subdivisions):
                grid[i_y, i_x] = 1

        # pygame.display.flip()
        # time.sleep(0.5)
        rgba[:,:,:3] = (grid[:,:,None] * 255).astype(np.uint8)
        rgba[:,:,3] = 255
        grid_surface = pygame.image.frombuffer(rgba.tobytes(), (grid_subdivisions, grid_subdivisions), "RGBA")
        pygame.transform.scale(grid_surface, (window_size, window_size), screen)
        pygame.draw.circle(screen, RED, (window_size/2, window_size/2), 5)
        grid *= .8
        grid[grid < 0.02] = 0


        pygame.display.flip()
        time.sleep(0.2)

if __name__ == "__main__":
    with RPLidar() as lidar:
        try:
            main(lidar)
        except KeyboardInterrupt:
            print("KeyboardInterrupt intercepted. Shutting down program")