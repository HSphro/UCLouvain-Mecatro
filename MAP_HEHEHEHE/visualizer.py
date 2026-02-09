import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# --- CONFIGURATION ---
UDP_IP = "127.0.0.1"
UDP_PORT = 8888

# Landscape Orientation
TABLE_WIDTH = 3.0   # Horizontal (Long side)
TABLE_HEIGHT = 2.0  # Vertical (Short side)

# --- SETUP UDP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 

# --- SETUP PLOT ---
# figsize=(10, 4) creates a wide landscape window
fig, ax = plt.subplots(figsize=(10, 4)) 

ax.set_xlim(0, TABLE_WIDTH)
ax.set_ylim(0, TABLE_HEIGHT)
ax.set_aspect('equal')
ax.grid(True, linestyle='--')
ax.set_title("Robot Tracker (Landscape 5x2m)")
ax.set_xlabel("X [meters]")
ax.set_ylabel("Y [meters]")

# Draw a rectangle to represent the table borders clearly
table_border = patches.Rectangle((0, 0), TABLE_WIDTH, TABLE_HEIGHT, 
                                 linewidth=2, edgecolor='black', facecolor='none')
ax.add_patch(table_border)

# Create plot elements
trail_line, = ax.plot([], [], 'b-', alpha=0.5, label='Trail') 
robot_dot, = ax.plot([], [], 'ro', markersize=12, label='Robot')

# Data storage
x_data, y_data = [], []
current_x, current_y = 0, 0

def update(frame):
    global current_x, current_y
    
    # Read ALL available packets
    try:
        while True:
            data, _ = sock.recvfrom(1024)
            decoded = data.decode('utf-8')
            parts = decoded.split(',')
            if len(parts) == 3:
                current_x = float(parts[0])
                current_y = float(parts[1])
                
                x_data.append(current_x)
                y_data.append(current_y)
                
                # Keep trail manageable
                if len(x_data) > 500:
                    x_data.pop(0)
                    y_data.pop(0)
    except BlockingIOError:
        pass 

    # Update the plot
    trail_line.set_data(x_data, y_data)
    robot_dot.set_data([current_x], [current_y])
    
    return trail_line, robot_dot

ani = animation.FuncAnimation(fig, update, interval=20, blit=True)

plt.show()