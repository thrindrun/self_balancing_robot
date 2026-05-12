import asyncio
import threading
import re
from collections import deque
from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Configuration ---
# Ensure this matches your ESP32's name in NimBLEDevice::init
DEVICE_NAME = "ESP32_SMC_Bot" 
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" 
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

MAX_POINTS = 200;

#shared data
times = deque(maxlen=MAX_POINTS)
thetas = deque(maxlen=MAX_POINTS)
pwms = deque(maxlen=MAX_POINTS)
positions = deque(maxlen=MAX_POINTS)
velocities = deque(maxlen=MAX_POINTS)
time_counter = 0

class ThreadSafeQueue:
    def __init__(self):
        self.queue = []
        self.lock = threading.Lock()
    def put(self, item):
        with self.lock: self.queue.append(item)
    def pop_all(self):
        with self.lock:
            items = list(self.queue)
            self.queue.clear()
            return items
        
send_queue = ThreadSafeQueue()

def notification_handler(sender, data):
    global time_counter
    try:
        text = data.decode('utf-8').strip()
        if text.startswith(">>"):
            print(f"\n[ROBOT] {text}")
            return
            
        # Matches the snprintf format: "The:%.2f, PWM:%.0f, Pos:%.2f, Vel:%.2f"
        pattern = r"The:([-\d.]+),\s*PWM:([-\d.]+),\s*Pos:([-\d.]+),\s*Vel:([-\d.]+)"
        match = re.search(pattern, text)
        if match:
            theta = float(match.group(1))
            pwm = float(match.group(2))
            pos = float(match.group(3))
            vel = float(match.group(4))

            times.append(time_counter)
            thetas.append(theta)
            pwms.append(pwm)
            positions.append(pos)
            velocities.append(vel)
            time_counter += 1
    except Exception as e:
        pass

async def ble_task():
    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if not device:
        print(f"Device {DEVICE_NAME} not found.")
        return

    async with BleakClient(device) as client:
        print(f"Connected to {DEVICE_NAME}")
        await client.start_notify(TX_CHAR_UUID, notification_handler)
        
        while True:
            messages = send_queue.pop_all()
            for msg in messages:
                if msg == "QUIT": return
                await client.write_gatt_char(RX_CHAR_UUID, msg.encode('utf-8'))
            await asyncio.sleep(0.05)

def input_thread():
    print("\n" + "="*30)
    print(" SMC TUNING INTERFACE")
    print(" S: Start | X: Stop")
    print(" M <1-3>: Mode (1:Cls, 2:Hyb, 3:Hie)")
    print(" <1-6> <value>: Set Param")
    print(" Example: 'M 2' then '1 45.0' (Sets K1 to 45 in Hybrid)")
    print("="*30 + "\n")
    
    while True:
        try:
            user_input = input("").strip().upper()
            if not user_input: continue
            
            # Direct commands (S, X)
            if user_input in ['S', 'X']:
                send_queue.put(user_input)
            
            # Mode or Parameter commands (M 2, 1 15.5)
            else:
                parts = user_input.split()
                if len(parts) == 2:
                    # e.g., "M 2" -> sends "M2" to ESP32
                    # e.g., "1 15.5" -> sends "115.5" to ESP32
                    cmd = f"{parts[0]}{parts[1]}"
                    send_queue.put(cmd)
        except EOFError: break

def main():
    threading.Thread(target=lambda: asyncio.run(ble_task()), daemon=True).start()
    threading.Thread(target=input_thread, daemon=True).start()

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    
    line_theta, = ax1.plot([], [], 'r-', label='Angle (deg)')
    line_pwm,   = ax2.plot([], [], 'b-', label='PWM Output')
    line_pos,   = ax3.plot([], [], 'g-', label='Pos (m)')

    for ax in [ax1, ax2, ax3]:
        ax.grid(True)
        ax.legend(loc='upper right')

    ax1.set_ylim(-30, 30)
    ax2.set_ylim(-1100, 1100)
    ax3.set_ylim(-1, 1)

    def animate(frame):
        if not times: return line_theta, line_pwm, line_pos
        
        curr_x = list(times)
        ax1.set_xlim(max(0, curr_x[-1]-MAX_POINTS), max(MAX_POINTS, curr_x[-1]))
        ax2.set_xlim(max(0, curr_x[-1]-MAX_POINTS), max(MAX_POINTS, curr_x[-1]))
        ax3.set_xlim(max(0, curr_x[-1]-MAX_POINTS), max(MAX_POINTS, curr_x[-1]))

        line_theta.set_data(curr_x, list(thetas))
        line_pwm.set_data(curr_x, list(pwms))
        line_pos.set_data(curr_x, list(positions))
        return line_theta, line_pwm, line_pos

    ani = animation.FuncAnimation(fig, animate, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()