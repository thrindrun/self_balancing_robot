import streamlit as st
import asyncio
import threading
import re
import pandas as pd
import time
from collections import deque
from bleak import BleakClient, BleakScanner

# --- Configuration ---
DEVICE_NAME = "ESP32_SMC_Bot"
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
MAX_POINTS = 150

# --- Shared State Class ---
class GlobalState:
    def __init__(self):
        self.lock = threading.Lock()
        self.times = deque(maxlen=MAX_POINTS)
        self.thetas = deque(maxlen=MAX_POINTS)
        self.pwms = deque(maxlen=MAX_POINTS)
        self.positions = deque(maxlen=MAX_POINTS)
        self.velocities = deque(maxlen=MAX_POINTS)
        self.send_queue = []
        self.time_counter = 0
        self.connected = False
        self.active_mode = "Hie" 

@st.cache_resource
def get_state():
    return GlobalState()

state = get_state()

# --- BLE Logic ---
def notification_handler(sender, data):
    try:
        text = data.decode('utf-8').strip()
        if text.startswith(">>"): return 
        
        match = re.search(r"The:([-\d.]+),\s*PWM:([-\d.]+),\s*Pos:([-\d.]+),\s*Vel:([-\d.]+)", text)
        if match:
            with state.lock:
                state.thetas.append(float(match.group(1)))
                state.pwms.append(float(match.group(2)))
                state.positions.append(float(match.group(3)))
                state.velocities.append(float(match.group(4)))
                state.times.append(state.time_counter)
                state.time_counter += 1
    except Exception: pass

async def ble_worker():
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    if not device: return
    async with BleakClient(device) as client:
        with state.lock: state.connected = True
        await client.start_notify(TX_CHAR_UUID, notification_handler)
        while client.is_connected:
            with state.lock:
                msgs = list(state.send_queue)
                state.send_queue.clear()
            for msg in msgs:
                await client.write_gatt_char(RX_CHAR_UUID, msg.encode())
            await asyncio.sleep(0.05)
        with state.lock: state.connected = False

if 'thread_started' not in st.session_state:
    threading.Thread(target=lambda: asyncio.run(ble_worker()), daemon=True).start()
    st.session_state.thread_started = True

# --- UI Layout ---
st.set_page_config(page_title="SMC Control", layout="wide")

# Sidebar for Connection and Main Controls
with st.sidebar:
    st.title("Robot Terminal")
    if state.connected: st.success("Robot Connected")
    else: st.error("Searching for Robot...")
    
    st.divider()
    if st.button("START SYSTEM", use_container_width=True):
        with state.lock: state.send_queue.append("S")
    if st.button("STOP SYSTEM", type="primary", use_container_width=True):
        with state.lock: state.send_queue.append("X")
    
    st.divider()
    mode_map = {"Classic": "1", "Hybrid": "2", "Hierarchical": "3", "PID": "4", "LQR": "5"}
    
    selected_mode = st.radio(
        "Control Strategy", 
        list(mode_map.keys()), 
        key="strategy_selector"
    )
    
    if st.button("Apply Mode"):
        with state.lock:
            state.active_mode = selected_mode
            state.send_queue.append(f"M{mode_map[selected_mode]}")

# Main Dashboard split
col_plots, col_tuning = st.columns([2, 1])

with col_plots:
    st.subheader("Live Telemetry")
    chart_angle = st.empty()
    chart_pwm = st.empty()
    chart_pos = st.empty()

    with state.lock:
        df = pd.DataFrame({
            'Time': list(state.times),
            'Angle': list(state.thetas),
            'PWM': list(state.pwms),
            'Pos': list(state.positions),
        }).set_index('Time')
    
    if not df.empty:
        chart_angle.line_chart(df[['Angle']])
        chart_pwm.line_chart(df[['PWM']])
        chart_pos.line_chart(df[['Pos']])
    else:
        st.info("Awaiting telemetry stream...")

# --- Fixed-Footprint Tuning Panel ---
with col_tuning:
    st.subheader(f"Tuning: {selected_mode}")
    
    # Establish dynamic labels array based strictly on selection
    if selected_mode == "Classic":
        labels = ["C1", "C2", "C3", "C4", "Eta", "Phi"]
    elif selected_mode == "Hybrid":
        labels = ["K1", "K2", "K3", "K4", "Lambda1", "Lambda2"]
    elif selected_mode == "Hierarchical":
        labels = ["K1", "K2", "Lambda1", "Lambda2", "Eta", "Phi"]
    elif selected_mode == "PID":
        labels = ["Ang Kp", "Ang Ki", "Ang Kd", "Pos Kp", "Pos Ki", "Pos Kd"]
    elif selected_mode == "LQR":
        labels = ["K1", "K2", "K3", "K4"]

    raw_vals = {}
    
    # FIX: Keep exactly one static container form. It never changes its structure,
    # meaning Streamlit never gets confused trying to clean up elements.
    with st.form(key="master_tuning_form", clear_on_submit=False):
        
        # Inputs 1 to 4 are universally shared across all modes
        raw_vals[1] = st.number_input(labels[0], value=0.0, format="%.4f", key="gain_slot_1")
        raw_vals[2] = st.number_input(labels[1], value=0.0, format="%.4f", key="gain_slot_2")
        raw_vals[3] = st.number_input(labels[2], value=0.0, format="%.4f", key="gain_slot_3")
        raw_vals[4] = st.number_input(labels[3], value=0.0, format="%.4f", key="gain_slot_4")
        
        # Inputs 5 and 6 only render if the current mode actually requires them
        if len(labels) == 6:
            raw_vals[5] = st.number_input(labels[4], value=0.0, format="%.4f", key="gain_slot_5")
            raw_vals[6] = st.number_input(labels[5], value=0.0, format="%.4f", key="gain_slot_6")
            
        submit_button = st.form_submit_button("Update Gains", use_container_width=True)

    # Process and append data cleanly
    if submit_button:
        with state.lock:
            for idx, val in raw_vals.items():
                state.send_queue.append(f"{idx}{val:.4f}")
        
        if 'history' not in st.session_state: 
            st.session_state.history = []
        entry = {"Time": time.strftime("%H:%M:%S"), "Mode": selected_mode}
        for i, label in enumerate(labels): 
            entry[label] = raw_vals[i+1]
        st.session_state.history.insert(0, entry)

st.divider()
st.subheader("Gain History")
if 'history' in st.session_state and st.session_state.history:
    st.dataframe(pd.DataFrame(st.session_state.history), use_container_width=True)

# Auto-refresh loop
time.sleep(0.1)
st.rerun()