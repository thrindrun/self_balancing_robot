import asyncio
import threading
import re
import sys
from collections import deque
from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# --- Configuration ---
DEVICE_NAME = "ESP32_SelfBalancingBot"
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" # ESP32 -> Phone (Notify)
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" # Phone -> ESP32 (Write)

MAX_POINTS = 200

# --- Shared Data ---
times = deque(maxlen=MAX_POINTS)
thetas = deque(maxlen=MAX_POINTS)
pwms = deque(maxlen=MAX_POINTS)
time_counter = 0

# Create a queue to safely pass messages from the input thread to the asyncio loop
class ThreadSafeQueue:
    def __init__(self):
        self.queue = []
        self.lock = threading.Lock()
    
    def put(self, item):
        with self.lock:
            self.queue.append(item)
            
    def pop_all(self):
        with self.lock:
            items = list(self.queue)
            self.queue.clear()
            return items

send_queue = ThreadSafeQueue()

def notification_handler(sender, data):
    """Callback for BLE notifications."""
    global time_counter
    try:
        text = data.decode('utf-8').strip()
        
        # Check if it's a confirmation message for PID tuning
        if text.startswith(">>"):
            print(f"\n[ROBOT] {text}")
            return
            
        # Parse telemetry data
        match = re.search(r"Theta:\s*([-\d.]+),\s*PWM:\s*([-\d.]+)", text)
        if match:
            theta = float(match.group(1))
            pwm = float(match.group(2))
            
            times.append(time_counter)
            thetas.append(theta)
            pwms.append(pwm)
            time_counter += 1
            
    except Exception as e:
        print(f"BLE Connection Error: {type(e).__name__} - {e}")
async def ble_task():
    """Main BLE connection and communication loop."""
    print(f"Scanning for {DEVICE_NAME}...")
    
    device = None
    try:
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    except Exception as e:
        print(f"Scanner error: {e}")
        return

    if not device:
        print(f"Could not find device '{DEVICE_NAME}'. Make sure it's powered on and advertising.")
        return

    print(f"Found device: {device.name}. Connecting...")
    
    try:
        async with BleakClient(device) as client:
            print("Connected to ESP32!")
            await client.start_notify(TX_CHAR_UUID, notification_handler)
            print("Started receiving telemetry. Close the plot window to exit.")
            print("-" * 50)
            
            while True:
                # Check for pending messages to send
                messages = send_queue.pop_all()
                for msg in messages:
                    if msg == "QUIT":
                        return
                    await client.write_gatt_char(RX_CHAR_UUID, msg.encode('utf-8'))
                    print(f"Successfully sent: {msg}")
                
                await asyncio.sleep(0.1) # Prevent CPU hogging
                
    except Exception as e:
        print(f"BLE Connection Error: {e}")

def input_thread():
    """Reads terminal input for PID tuning."""
    print("=" * 50)
    print("PID Tuning Interface Ready")
    print("Format: <P|I|D> <value>")
    print("Example: P 15.5")
    print("Example: I 0.5")
    print ("Commands: S (Start), X (Stop)")
    print("=" * 50)
    
    while True:
        try:
            cmd = input().strip().upper()
            if not cmd:
                continue

            if cmd in ['S', 'X']:
                send_queue.put(cmd)
                continue
                
            parts = cmd.split()
            if len(parts) == 2 and parts[0] in ['P', 'I', 'D']:
                try:
                    val = float(parts[1])
                    msg = f"{parts[0]}{val}"
                    send_queue.put(msg)
                except ValueError:
                    print("Invalid value. Example: P 15.5")
            else:
                print("Invalid command format. Use S/X to start/stop, P, I, or D followed by a space and a number.")
        except EOFError:
            break
        except Exception:
            pass

def run_ble_loop():
    """Runs the asyncio event loop for BLE."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_task())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()

def main():
    # 1. Start BLE thread
    ble_thread = threading.Thread(target=run_ble_loop, daemon=True)
    ble_thread.start()

    # 2. Start input thread for PID tuning
    inp_thread = threading.Thread(target=input_thread, daemon=True)
    inp_thread.start()

    # 3. Setup Matplotlib Plotting (Must run in main thread)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.canvas.manager.set_window_title('Self-Balancing Robot Telemetry')
    
    # Angle Plot
    line_theta, = ax1.plot([], [], 'r-', label='Theta (deg)', linewidth=2)
    ax1.set_xlim(0, MAX_POINTS)
    ax1.set_ylim(-45, 45) # Typical balance range
    ax1.set_title("Robot Angle (Theta)")
    ax1.set_ylabel("Degrees")
    ax1.legend(loc='upper right')
    ax1.grid(True)

    # PWM Plot
    line_pwm, = ax2.plot([], [], 'b-', label='PWM', linewidth=2)
    ax2.set_xlim(0, MAX_POINTS)
    ax2.set_ylim(-1100, 1100) # PWM bounds in code are -1023 to 1023
    ax2.set_title("Motor PWM")
    ax2.set_ylabel("Duty Cycle")
    ax2.set_xlabel("Time (Samples)")
    ax2.legend(loc='upper right')
    ax2.grid(True)

    def animate(frame):
        if len(times) == 0:
            return line_theta, line_pwm
            
        x_data = list(times)
        
        # Auto-scroll X axis
        current_max = x_data[-1]
        x_min = max(0, current_max - MAX_POINTS)
        x_max = max(MAX_POINTS, current_max)
        
        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)

        # Update data
        line_theta.set_data(x_data, list(thetas))
        line_pwm.set_data(x_data, list(pwms))
        
        return line_theta, line_pwm

    # Update plot every 50ms (20 FPS)
    ani = animation.FuncAnimation(fig, animate, interval=50, blit=False, cache_frame_data=False)
    
    plt.tight_layout()
    try:
        plt.show() # This blocks until the window is closed
    except KeyboardInterrupt:
        pass
    finally:
        send_queue.put("QUIT")
        print("Plot closed. Exiting...")

if __name__ == "__main__":
    main()