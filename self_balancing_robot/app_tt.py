import streamlit as st
import asyncio
import threading
import re
import pandas as pd
import time
from collections import deque
from bleak import BleakClient, BleakScanner

# --- Configuration ---
DEVICE_NAME = "ESP32_SelfBalancingBot"
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
MAX_POINTS = 100

# --- Shared State Class (Persists across Reruns) ---
class GlobalState:
    def __init__(self):
        self.lock = threading.Lock()
        self.thetas = deque(maxlen=MAX_POINTS)
        self.pwms = deque(maxlen=MAX_POINTS)
        self.times = deque(maxlen=MAX_POINTS)
        self.send_queue = []
        self.time_counter = 0
        self.connected = False

# Use Streamlit's cache to keep this object alive across reruns
@st.cache_resource
def get_state():
    return GlobalState()

state = get_state()

# --- BLE Background Logic ---
def notification_handler(sender, data):
    try:
        text = data.decode('utf-8').strip()
        # print(f"Raw: {text}") # Uncomment to debug raw strings

        if text.startswith(">>"):
            return 
            
        match = re.search(r"Theta:\s*([-\d.]+).*?PWM:\s*([-\d.]+)", text)
        if match:
            t_val = float(match.group(1))
            p_val = float(match.group(2))
            
            with state.lock:
                state.thetas.append(t_val)
                state.pwms.append(p_val)
                state.times.append(state.time_counter)
                state.time_counter += 1
    except Exception as e:
        print(f"Parsing error: {e}")

async def ble_worker():
    print("Searching for robot...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    if not device: 
        print("Device not found.")
        return
    
    async with BleakClient(device) as client:
        with state.lock:
            state.connected = True
        
        print("Connected to ESP32!")
        await client.start_notify(TX_CHAR_UUID, notification_handler)
        
        while client.is_connected:
            with state.lock:
                msgs = list(state.send_queue)
                state.send_queue.clear()
            
            for msg in msgs:
                await client.write_gatt_char(RX_CHAR_UUID, msg.encode())
            
            await asyncio.sleep(0.05)
        
        with state.lock:
            state.connected = False

def start_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_worker())

# Start the thread only once
if 'thread_started' not in st.session_state:
    thread = threading.Thread(target=start_ble_thread, daemon=True)
    thread.start()
    st.session_state.thread_started = True

# --- Streamlit UI Layout ---
st.set_page_config(page_title="Robot Control", layout="wide")
st.title("Robot Tuning Dashboard")

if 'history' not in st.session_state:
    st.session_state.history = []

col1, col2, col3 = st.columns([1, 1, 2])

with col1:
    st.subheader("System Control")
    if st.button("START SYSTEM", use_container_width=True):
        with state.lock: state.send_queue.append("S")
    if st.button("STOP", type="primary", use_container_width=True):
        with state.lock: state.send_queue.append("X")
    
    # Check the state object's connection status
    with state.lock:
        is_conn = state.connected

    if is_conn:
        st.success("Status: Connected")
    else:
        st.warning("Status: Searching/Disconnected")

with col2:
    st.subheader("PID Tuning")
    p_val = st.slider("Kp", 0.0, 50.0, 12.0)
    i_val = st.slider("Ki", 0.0, 5.0, 0.2)
    d_val = st.slider("Kd", 0.0, 5.0, 1.2)
    
    if st.button("Update PID"):
        with state.lock:
            # 1. Send the data to the BLE queue
            state.send_queue.append(f"P{p_val}")
            state.send_queue.append(f"I{i_val}")
            state.send_queue.append(f"D{d_val}")
        
        # 2. Correctly log to Streamlit's history (fixed line)
        st.session_state.history.insert(0, {
            "Time": time.strftime("%H:%M:%S"), 
            "Kp": p_val, 
            "Ki": i_val, 
            "Kd": d_val
        })

with col3:
    st.subheader("Live Telemetry")
    with state.lock:
        local_times = list(state.times)
        local_thetas = list(state.thetas)
        local_pwms = list(state.pwms)

    if local_times:
        df = pd.DataFrame({
            'Time': local_times,
            'Theta': local_thetas,
            'PWM': local_pwms
        }).set_index('Time')
        st.line_chart(df['Theta'])
        st.line_chart(df['PWM'])
    else:
        st.info("Waiting for data from robot...")

st.divider()
st.subheader("📜 Tuning History")
if st.session_state.history:
    st.table(pd.DataFrame(st.session_state.history))

# Auto-refresh
time.sleep(0.1)
st.rerun()