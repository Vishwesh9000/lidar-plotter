import serial, time, math, atexit, signal
import multiprocessing as mp
import multiprocessing.shared_memory
import numpy as np
# import pygame # Imported in main to prevent re-import in child process
PORT = "COM15"
BAUDRATE = 460800
WHITE = (255, 255, 255)
BLACK = (0,0,0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

BACKGROUND_COLOR = BLACK
CIRCLE_COLOR = GREEN
CIRCLE_RAD = 2

SCALING_FACTOR = 0.1

SHM_NAME = "LIDAR_DATA"

BUFFER_SHAPE = (700, 3)
DTYPE = np.float32

screen = None

window_size = 800

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

        self.shm = self.create_shared_memory(SHM_NAME, self.BUFFER_SIZE)

        self.data = np.ndarray(BUFFER_SHAPE, dtype=DTYPE, buffer=self.shm.buf)

        self.lock = mp.Lock()
        self.shut_down_event = mp.Event()
        self.data_stream_active_event = mp.Event()

        self.proc = mp.Process(target=RPLidar.serial_reader, args=(self.shm.name, self.lock, self.shut_down_event, self.data_stream_active_event))
        self.proc.start()

        atexit.register(self.free_shared_memory)

    def create_shared_memory(self, name: str, size: int):
        try:
            shm = mp.shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shm = mp.shared_memory.SharedMemory(name=name)
            shm.close()
            shm.unlink()
            shm = mp.shared_memory.SharedMemory(name=name, create=True, size=size)
        return shm

    @staticmethod
    def serial_reader(name: str, lock, shut_down_event, data_stream_active_event):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        parent = mp.parent_process()
        shm = mp.shared_memory.SharedMemory(name=name, create=False)
        data = np.ndarray(BUFFER_SHAPE, DTYPE, buffer=shm.buf)
        ser = None
        index = 0
        incoming_bytes = bytearray()
        try: # Ensures clean shutdown if error occurs
            while not shut_down_event.is_set() and (parent is None or parent.is_alive()): # continously read serial
                if ser is None: # if serial is not connected
                    data_stream_active_event.clear()
                    try: # Try to reconnect and catch SerialException if failed
                        ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=0)

                        # TODO: Move reset commands out of try?
                        # ser.write(b"\xA5\x40")
                        # time.sleep(1)
                        ser.reset_input_buffer()
                        ser.write(b"\xA5\x20")
                        time.sleep(0.1)

                    except serial.SerialException:
                        print(f"Unable to connect to {PORT}. Retrying...")
                        time.sleep(2)
                        ser = None
                        continue

                    data_stream_active_event.set()

                if ser is not None:
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
                    
                    incoming_bytes += b
                    while len(incoming_bytes) >= 5:
                        # Decode and validate packet
                        # TODO: Optimize/correct order of checks
                        if (incoming_bytes[0] & 0x01) == ((incoming_bytes[0] >> 1) & 0x01):
                            del incoming_bytes[0]
                            print("Invalid Packet: Matching inverse start bits")
                            continue
                        
                        if incoming_bytes[1] & 0x01 == 0:
                            del incoming_bytes[0]
                            print("Invalid Packet: Check bit invalid")
                            continue

                        quality = int(incoming_bytes[0] >> 2)
                        # if quality == 0:
                        #     del incoming_bytes[:5]
                        #     print("Skipped over 0 quality reading")
                        #     continue
                        
                        angle = float((incoming_bytes[1] >> 1) | (incoming_bytes[2] << 7)) / 64
                        if angle >= 360:
                            del incoming_bytes[0]
                            print(f"Invalid Packet: Angle ({angle:.3f}) >= 360 degrees")
                            continue
                        
                        distance = float((incoming_bytes[3]) | (incoming_bytes[4] << 8))/4
                        if (distance < 30 or distance > 12_000) and distance != 0:
                            print(f"WARNING: Distance ({distance:.3f}) is out of expected range (50-2,000)")
                        

                        # Add data to buffer
                        with lock:
                            data[index] = (quality, angle, distance)
                        index += 1
                        index %= data.shape[0]
                        del incoming_bytes[:5]

                    time.sleep(0.01)
        except Exception as e:
            print("UNEXPECTED ERROR:", e)
        finally:
            RPLidar.close_serial(ser)
            shm.close()
    
    @staticmethod
    def close_serial(ser):
        if ser is not None and ser.is_open:
            ser.write(b"\xA5\x25")
            print("Stopped the LIDAR")
            time.sleep(0.1)
            ser.close()

    def is_active(self):
        return self.data_stream_active_event.is_set()

    def get_data(self):
        with self.lock:
            return self.data.copy()
    
    def free_shared_memory(self):
        if self.shm is not None:
            self.shm.close()
            self.shm.unlink()
            self.shm = None
            print("Freed shared memory")

    def stop(self):
        print("Stopping lidar monitoring...")
        self.shut_down_event.set()

        print("Waiting for process to stop")
        self.proc.join(timeout=2)

        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join()
            print("Process didn't respond. Process forcefully shut down")
        else:
            print("Process gracefully ended")

def main(lidar):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((window_size, window_size), pygame.SCALED)
    pygame.display.set_caption("LIDAR C1 Stream Viewer")
    pygame.draw.circle(screen, RED, (window_size/2, window_size/2), 5)
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
            x = math.cos(math.radians(90-angle))*distance*SCALING_FACTOR + window_size/2
            y = -math.sin(math.radians(90-angle))*distance*SCALING_FACTOR + window_size/2 # y axis points downward

            pygame.draw.circle(screen, CIRCLE_COLOR, (x, y), CIRCLE_RAD)
        pygame.display.flip()
        time.sleep(0.5)

if __name__ == "__main__":
    lidar = RPLidar()
    try:
        main(lidar)
    except KeyboardInterrupt:
        print("KeyboardInterrupt intercepted. Shutting down program")
    finally:
        lidar.stop()