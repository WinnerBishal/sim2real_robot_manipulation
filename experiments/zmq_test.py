import zmq
import numpy as np
import cv2

# --- Color Stream Configuration ---
COLOR_PORT = "5556"
COLOR_WIDTH = 1280
COLOR_HEIGHT = 720
COLOR_CHANNELS = 3
COLOR_DTYPE = np.uint8
COLOR_EXPECTED_BYTES = COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS * np.dtype(COLOR_DTYPE).itemsize

# --- Depth Stream Configuration ---
DEPTH_PORT = "5555"
DEPTH_WIDTH = 480
DEPTH_HEIGHT = 270
DEPTH_CHANNELS = 1
DEPTH_DTYPE = np.uint16
DEPTH_EXPECTED_BYTES = DEPTH_WIDTH * DEPTH_HEIGHT * DEPTH_CHANNELS * np.dtype(DEPTH_DTYPE).itemsize

# --- Set up ZMQ context and sockets ---
context = zmq.Context()

# Create color socket
color_socket = context.socket(zmq.SUB)
color_socket.connect(f"tcp://localhost:{COLOR_PORT}")
color_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Create depth socket
depth_socket = context.socket(zmq.SUB)
depth_socket.connect(f"tcp://localhost:{DEPTH_PORT}")
depth_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# --- Set up Poller ---
# A poller lets us check multiple sockets for messages without blocking
poller = zmq.Poller()
poller.register(color_socket, zmq.POLLIN)
poller.register(depth_socket, zmq.POLLIN)

print(f"Receiving COLOR on port {COLOR_PORT} and DEPTH on port {DEPTH_PORT}...")
print("Press 'q' in any window to quit.")

try:
    while True:
        # Poll all registered sockets for new messages (100ms timeout)
        socks = dict(poller.poll(timeout=100))

        # --- Check for Color Frame ---
        if color_socket in socks and socks[color_socket] == zmq.POLLIN:
            raw_bytes = color_socket.recv()

            if len(raw_bytes) == COLOR_EXPECTED_BYTES:
                frame = np.frombuffer(raw_bytes, dtype=COLOR_DTYPE)
                frame = frame.reshape((COLOR_HEIGHT, COLOR_WIDTH, COLOR_CHANNELS))
                
                # ZMQ sends RGB, but OpenCV's imshow expects BGR
                # If colors look weird, comment the line below and uncomment the next one
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # frame_bgr = frame 
                
                cv2.imshow("Color Frame", frame_bgr)
            else:
                print(f"COLOR ERROR: Received {len(raw_bytes)} bytes, expected {COLOR_EXPECTED_BYTES}")

        # --- Check for Depth Frame ---
        if depth_socket in socks and socks[depth_socket] == zmq.POLLIN:
            raw_bytes = depth_socket.recv()

            if len(raw_bytes) == DEPTH_EXPECTED_BYTES:
                frame = np.frombuffer(raw_bytes, dtype=DEPTH_DTYPE)
                frame = frame.reshape((DEPTH_HEIGHT, DEPTH_WIDTH))

                # --- Normalize depth for display ---
                # This converts the 16-bit depth data (e.g., 0-10000)
                # into an 8-bit grayscale image (0-255) so we can see it.
                depth_normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                
                cv2.imshow("Depth Frame", depth_normalized)
            else:
                print(f"DEPTH ERROR: Received {len(raw_bytes)} bytes, expected {DEPTH_EXPECTED_BYTES}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopping receiver...")
finally:
    print("Cleaning up resources.")
    cv2.destroyAllWindows()
    poller.unregister(color_socket)
    poller.unregister(depth_socket)
    color_socket.close()
    depth_socket.close()
    context.term()