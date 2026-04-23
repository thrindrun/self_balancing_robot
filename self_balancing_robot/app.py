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

# --- Global State for Streamlit ---
# We use st.session_state to keep data between website refreshes
if 'times' not in st.session_state:
    st.session_state.times = deque(maxlen=MAX_POINTS)
    st.session_state.thetas = deque(maxlen=MAX_POINTS)
    st.session_state.pwms = deque(maxlen=MAX_POINTS)
    st.session_state.time_counter = 0
    st.session_state.send_queue = []
    st.session_state.lock = threading.Lock()
    st.session_state.connected = False

if 'history' not in st.session_state:
    st.session_state.history = []


# --- BLE Background Logic ---
def notification_handler(sender, data):
    try:
        text = data.decode('utf-8').strip()
        if text.startswith(">>"):
            return # Skip confirmation messages for the graph
            
        match = re.search(r"Theta:\s*([-\d.]+),\s*PWM:\s*([-\d.]+)", text)
        if match:
            with st.session_state.lock:
                st.session_state.thetas.append(float(match.group(1)))
                st.session_state.pwms.append(float(match.group(2)))
                st.session_state.times.append(st.session_state.time_counter)
                st.session_state.time_counter += 1
    except:
        pass

async def ble_worker():
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    if not device: return
    
    async with BleakClient(device) as client:
        st.session_state.connected = True
        await client.start_notify(TX_CHAR_UUID, notification_handler)
        while True:
            # Check for messages to send to robot
            with st.session_state.lock:
                msgs = list(st.session_state.send_queue)
                st.session_state.send_queue.clear()
            
            for msg in msgs:
                await client.write_gatt_char(RX_CHAR_UUID, msg.encode())
            
            await asyncio.sleep(0.05)

def start_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_worker())

# Start the thread only once
if 'ble_thread' not in st.session_state:
    thread = threading.Thread(target=start_ble_thread, daemon=True)
    thread.start()
    st.session_state.ble_thread = True

# --- Streamlit UI Layout ---
st.set_page_config(page_title="Robot Control", layout="wide")
st.title("🤖 Robot Tuning Dashboard")

# Top Row: Controls
col1, col2, col3 = st.columns([1, 1, 2])

with col1:
    st.subheader("System Control")
    if st.button("🚀 START SYSTEM", use_container_width=True):
        with st.session_state.lock: st.session_state.send_queue.append("S")
    if st.button("🛑 STOP (Emergency)", type="primary", use_container_width=True):
        with st.session_state.lock: st.session_state.send_queue.append("X")

with col2:
    st.subheader("PID Tuning")
    # Using sliders for real-time feel
    p = st.slider("Kp", 0.0, 50.0, 12.0)
    i = st.slider("Ki", 0.0, 5.0, 0.2)
    d = st.slider("Kd", 0.0, 5.0, 1.2)
    
    if st.button("Update PID"):
        with st.session_state.lock:
            st.session_state.send_queue.append(f"P{p}")
            st.session_state.send_queue.append(f"I{i}")
            st.session_state.send_queue.append(f"D{d}")
        
        new_log = {"Time": time.strftime("%H:%M:%S"), "Kp": p, "Ki": i, "Kd": d}
        st.session_state.history.insert(0, new_log) # Add to history

with col3:
    st.subheader("Live Telemetry")
    # Create dataframes for charts
    if len(st.session_state.times) > 0:
        df = pd.DataFrame({
            'Time': list(st.session_state.times),
            'Theta': list(st.session_state.thetas),
            'PWM': list(st.session_state.pwms)
        }).set_index('Time')
        
        st.line_chart(df['Theta'])
        st.line_chart(df['PWM'])
    else:
        st.info("Waiting for data from robot...")

st.divider()
st.subheader("PID Tuning History")
if st.session_state.history:
    st.table(pd.DataFrame(st.session_state.history))
else:
    st.write("No tuning history yet.")
# Auto-refresh the page to show new data
time_interval = 0.1 
st.empty() # Placeholder
time.sleep(time_interval)
st.rerun()