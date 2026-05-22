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
        self.active_mode = "Hie" # Default matches your ESP32

@st.cache_resource
def get_state():
    return GlobalState()

state = get_state()

# --- BLE Logic ---
def notification_handler(sender, data):
    try:
        text = data.decode('utf-8').strip()
        if text.startswith(">>"): return 
        
        # Matches: The:%.2f, PWM:%.0f, Pos:%.2f, Vel:%.2f
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
    if st.button("START SYSTEM", width="stretch"):
        with state.lock: state.send_queue.append("S")
    if st.button("STOP SYSTEM", type="primary", width="stretch"):
        with state.lock: state.send_queue.append("X")
    
    st.divider()
    mode_map = {"Classic": "1", "Hybrid": "2", "Hierarchical": "3", "PID": "4", "LQR": "5"}
    selected_mode = st.radio("Control Strategy", list(mode_map.keys()))
    
    if st.button("Apply Mode"):
        with state.lock:
            state.active_mode = selected_mode
            state.send_queue.append(f"M{mode_map[selected_mode]}")

# Main Dashboard
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

with col_tuning:
    st.subheader(f"Tuning: {selected_mode}")
    
    # 1. Establish labels based strictly on the selected mode
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

    # Track current mode in session state to handle clean visual transitions
    if "current_mode" not in st.session_state:
        st.session_state.current_mode = selected_mode

    # If the user changed the radio button selection, manually purge old elements 
    # from the active session dictionary before rendering the new form.
    if st.session_state.current_mode != selected_mode:
        # Scan and destroy any leftover field keys from the previous run
        keys_to_clear = [k for k in st.session_state.keys() if "field_" in k or "tuning_form_" in k]
        for k in keys_to_clear:
            del st.session_state[k]
        st.session_state.current_mode = selected_mode
        st.rerun()

    inputs = {}
    
    # 2. Use a unique form structure that destroys itself on mode changes
    with st.form(key=f"tuning_form_instance_{selected_mode}", clear_on_submit=False):
        for i, label in enumerate(labels):
            # Form elements must have an isolated, strict naming structure
            inputs[i+1] = st.number_input(
                f"{label}", 
                value=0.0, 
                format="%.4f", 
                key=f"field_{selected_mode}_{label}"
            )
            
        submit_button = st.form_submit_button("Update Gains", use_container_width=True)

    # 3. Process data cleanly when the form is submitted
    if submit_button:
        with state.lock:
            for idx, val in inputs.items():
                state.send_queue.append(f"{idx}{val:.4f}")
        
        if 'history' not in st.session_state: 
            st.session_state.history = []
        entry = {"Time": time.strftime("%H:%M:%S"), "Mode": selected_mode}
        for i, label in enumerate(labels): 
            entry[label] = inputs[i+1]
        st.session_state.history.insert(0, entry)

st.divider()
st.subheader("Gain History")
if 'history' in st.session_state and st.session_state.history:
    st.dataframe(pd.DataFrame(st.session_state.history), width="stretch")

# Auto-refresh
time.sleep(0.1)
st.rerun()