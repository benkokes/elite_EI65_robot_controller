import os
import platform
import threading
import paramiko
import time
import pyte
import re
import socket
import tkinter as tk
from tkinter import scrolledtext, Entry, Label, Button, Frame, StringVar
import numpy as np
from scipy.optimize import least_squares
import scipy.spatial.transform
import queue

# Terminal dimensions
ROWS, COLS = 24, 84

# SSH Configuration
SSH_HOST = "192.168.1.200"
# Construct the absolute path to the credentials file
CREDENTIALS_FILE = os.path.join(os.path.dirname(__file__), "robot_credentials")
COMMAND = "cd /rbctrl && ./robotmon"

# TCP destination
TCP_IP = "192.168.1.200"
TCP_PORT = 8055

DEBUG = True

def clear_screen():
    if platform.system() == "Windows":
        os.system("cls")
    else:
        os.system("clear")

screen = pyte.Screen(COLS, ROWS)
stream = pyte.Stream(screen)
joint_angle_queue = queue.Queue()
plc_state_queue = queue.Queue()

def ssh_stream_function():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        username, password = get_robot_credentials()
        ssh.connect(SSH_HOST, username=username, password=password)
    except ValueError as e:
        log_message(f"SSH Error: {e}")
        return
    except paramiko.AuthenticationException:
        log_message("SSH Authentication failed.")
        return
    except paramiko.SSHException as e:
        log_message(f"SSH connection error: {e}")
        return
    except Exception as e:
        log_message(f"General SSH error: {e}")
        return

    channel = ssh.invoke_shell()

    time.sleep(1)
    channel.send(COMMAND + "\n")
    time.sleep(2)

    try:
        while True:
            if channel.recv_ready():
                raw_data = channel.recv(4096)
                decoded_data = raw_data.decode(errors="ignore")
                stream.feed(decoded_data)
                angles = extract_joint_angles_alternative()
                if angles:
                    joint_angle_queue.put(angles)
                
                plc_states = extract_plc_states(screen.display) # Extract PLC states
                if plc_states:
                    plc_state_queue.put(plc_states) # Put into new queue
                    if DEBUG: log_message(f"SSH: Put PLC states in queue: {plc_states}")
                else:
                    if DEBUG: log_message("SSH: No PLC states extracted in this cycle.")

    
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("SSH stream stopped by user.")
    finally:
        ssh.close()

def extract_joint_angles_alternative():
    full_text = "\n".join(screen.display)
    lines = full_text.splitlines()
    if len(lines) < 5:
        return None
    joint_text = lines[3] + " " + lines[4]
    pattern = re.compile(
        r"S:\s*([-\d\.]+)\s*L:\s*([-\d\.]+)\s*U:\s*([-\d\.]+)\s*R:\s*([-\d\.]+)\s*B:\s*([-\d\.]+)\s*T:\s*([-\d\.]+\s*[-\d\.]+)\s*J7:\s*([-\d\.]+)\s*J8:\s*([-\d\.]+)",
        re.DOTALL
    )
    match = pattern.search(joint_text)
    if match:
        try:
            t_val = ''.join(match.group(6).split())
            return {
                "S": float(match.group(1)), "L": float(match.group(2)),
                "U": float(match.group(3)), "R": float(match.group(4)),
                "B": float(match.group(5)), "T": float(t_val),
                "J7": float(match.group(7)), "J8": float(match.group(8))
            }
        except Exception as e:
            log_message("Error parsing joint angles: " + str(e))
    return None

root = tk.Tk()
root.title("Joint Positions & Robot Control")
original_bg_color = root.cget("background")

def extract_plc_states(screen_display_lines):
    """
    Extracts PLC IN and PLC OUT bit states from the raw screen display lines.
    Args:
        screen_display_lines: A list of strings, representing lines from pyte.Screen.display.
    Returns:
        A dictionary with 'plc_in' and 'plc_out' keys, each containing the
        concatenated bit string (e.g., "X-------" -> "10000000").
    """
    full_text = "\n".join(screen_display_lines)
    
    plc_in_pattern = re.compile(r"?PLC IN?.*?0x0000:\s*([X\- ]+)", re.DOTALL)
    plc_out_pattern = re.compile(r"?PLC OUT?.*?0x0000:\s*([X\- ]+)", re.DOTALL)

    plc_in_match = plc_in_pattern.search(full_text)
    plc_out_match = plc_out_pattern.search(full_text)
    
    plc_states = {}
    
    if DEBUG: log_message(f"PLC Regex: Searching for PLC IN pattern: '{plc_in_pattern.pattern}'")
    if DEBUG: log_message(f"PLC Regex: Searching for PLC OUT pattern: '{plc_out_pattern.pattern}'")
    # log_message(f"Full text for PLC search:\n{full_text}") # Too verbose, uncomment if needed

    if plc_in_match:
        raw_bits = plc_in_match.group(1)
        cleaned_bits = raw_bits.replace(' ', '').replace('X', '1').replace('-', '0')
        plc_states['plc_in'] = cleaned_bits
        if DEBUG: log_message(f"PLC Regex: Matched PLC IN. Raw: '{raw_bits}', Cleaned: '{cleaned_bits}'")
    else:
        plc_states['plc_in'] = None
        if DEBUG: log_message("PLC Regex: No match found for PLC IN.")

    if plc_out_match:
        raw_bits = plc_out_match.group(1)
        cleaned_bits = raw_bits.replace(' ', '').replace('X', '1').replace('-', '0')
        plc_states['plc_out'] = cleaned_bits
        if DEBUG: log_message(f"PLC Regex: Matched PLC OUT. Raw: '{raw_bits}', Cleaned: '{cleaned_bits}'")
    else:
        plc_states['plc_out'] = None
        if DEBUG: log_message("PLC Regex: No match found for PLC OUT.")
    return plc_states

def set_background_red():
    root.config(bg="light coral")

def reset_background_color():
    root.config(bg=original_bg_color)

data_frame = tk.Frame(root)
data_frame.pack(pady=10)

mode_label = tk.Label(data_frame, text="MODE:", font=("Helvetica", 12))
mode_label.pack(side=tk.LEFT, padx=0)
mode_value = StringVar(value="")
mode_display = tk.Label(data_frame, textvariable=mode_value, font=("Helvetica", 12, "bold"))
mode_display.pack(side=tk.LEFT, padx=0)

speed_label = tk.Label(data_frame, text="SPEED:", font=("Helvetica", 12))
speed_label.pack(side=tk.LEFT, padx=10)
speed_value = StringVar(value="")  # Use StringVar
speed_display = tk.Label(data_frame, textvariable=speed_value, font=("Helvetica", 12, "bold"))
speed_display.pack(side=tk.LEFT, padx=0)

coord_label = tk.Label(data_frame, text="COORD:", font=("Helvetica", 12))
coord_label.pack(side=tk.LEFT, padx=10)
coord_value = StringVar(value="")  # Use StringVar
coord_display = tk.Label(data_frame, textvariable=coord_value, font=("Helvetica", 12, "bold"))
coord_display.pack(side=tk.LEFT, padx=0)

status_label = tk.Label(data_frame, text="SERVO:", font=("Helvetica", 12))
status_label.pack(side=tk.LEFT, padx=10)
servo_status_value = StringVar(value="")  # Use StringVar
status_display = tk.Label(data_frame, textvariable=servo_status_value, font=("Helvetica", 12, "bold"))
status_display.pack(side=tk.LEFT, padx=0)

joint_label = tk.Label(root, font=("Helvetica", 14))
joint_label.pack(pady=10)

# --- PLC Display (Two Lines) ---
PLC_BITS = 64

# Function to format 64 bits into a string with spaces every 8 bits
def add_spaces_to_bits(bit_string):
    if not bit_string or len(bit_string) != PLC_BITS:
        # Return a placeholder string with correct spacing if data is missing
        placeholder_segment = "--------" # 8 dashes for 8 bits
        return " ".join([placeholder_segment] * (PLC_BITS // 8))
    
    spaced_bits = ""
    for i in range(0, PLC_BITS, 8):
        spaced_bits += bit_string[i:i+8] + " "
    return spaced_bits.strip() # Remove trailing space

# Frame for PLC display
plc_display_frame = tk.Frame(root, bd=2, relief="groove")
plc_display_frame.pack(pady=5, padx=10, fill="x")

# PLC IN
tk.Label(plc_display_frame, text="PLC IN:", font=("Helvetica", 12, "bold")).pack(anchor='w', padx=5, pady=(5,0))
plc_in_display_label = tk.Label(plc_display_frame, text=add_spaces_to_bits(""), font=("Courier", 14), fg="black")
plc_in_display_label.pack(anchor='w', padx=5)

# PLC OUT
tk.Label(plc_display_frame, text="PLC OUT:", font=("Helvetica", 12, "bold")).pack(anchor='w', padx=5, pady=(5,0))
plc_out_display_label = tk.Label(plc_display_frame, text=add_spaces_to_bits(""), font=("Courier", 14), fg="black")
plc_out_display_label.pack(anchor='w', padx=5)
# --- End PLC Display (Two Lines) ---



input_frame = tk.Frame(root)
input_frame.pack(pady=10)

joint_names = ["S", "L", "U", "R", "B", "T", "J7", "J8"]
entries = {}



def format_joint_entry(event):
    widget = event.widget
    try:
        value = float(widget.get())
        widget.delete(0, tk.END)
        widget.insert(0, f"{value:.6f}")
    except ValueError:
        pass

for joint in joint_names:
    frm = tk.Frame(input_frame)
    frm.pack(side=tk.LEFT, padx=5)
    tk.Label(frm, text=joint, font=("Helvetica", 12)).pack()
    ent = tk.Entry(frm, width=10, font=("Helvetica", 12))
    ent.pack()
    ent.bind("<FocusOut>", format_joint_entry)
    entries[joint] = ent

button_frame = tk.Frame(root)
button_frame.pack(pady=10)

def send_move_command():
    values = []
    for joint in joint_names:
        val = entries[joint].get().strip()
        if not val:
            val = "0"
        values.append(val)
    command_str = "runForward " + " ".join(values) + "\r\n"  # Add CRLF
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        sock.sendall(command_str.encode())
        sock.close()
        log_message("Sent command: " + command_str)
        reset_background_color()
    except Exception as e:
        log_message("Error sending command: " + str(e))
        set_background_red()

def send_stop_command():
    command_str = "stop\r\n"  # Add CRLF
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        sock.sendall(command_str.encode())
        sock.close()
        log_message("Sent command: " + command_str)
        reset_background_color()
    except Exception as e:  # Corrected line
        log_message("Error sending command: " + str(e))
        set_background_red()

def send_and_receive_tcp(command, timeout=0.5):
    """Sends a TCP command and returns the relevant response."""
    try:
        command_with_crlf = command + "\r\n"
        if DEBUG:
            print(f"TCP: Sending command: {command_with_crlf!r}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        sock.sendall(command_with_crlf.encode())
        if DEBUG:
            print(f"TCP: Command encoded: {command_with_crlf.encode()!r}")
        time.sleep(0.25)
        response = b""
        start_time = time.time()
        while True:
            try:
                sock.settimeout(timeout)
                chunk = sock.recv(1024)
                if not chunk:
                    break
                response += chunk
                if b">" in chunk:
                    break
                if time.time() - start_time > timeout:
                    if DEBUG:
                        print("TCP: Timeout reached, stopping recv")
                    break
            except socket.timeout:
                if DEBUG:
                    print("TCP: recv timed out, trying again")
                if time.time() - start_time > timeout:
                    if DEBUG:
                        print("TCP: Timeout reached, stopping recv")
                    break
                continue
        sock.close()
        response = response.decode(errors='ignore')
        if DEBUG:
            print(f"TCP: Received full response: {response!r}")
        response = response.strip()
        if DEBUG:
            print(f"TCP: Response stripped: {response!r}")

        if command == "speed":
            match = re.search(r"mcserver>\s*(\d+)", response)
            if match:
                result = match.group(1)
                if DEBUG:
                    print(f"TCP: Speed extracted: {result!r}")
                return result
            else:
                result = "ERROR: No speed found"
                if DEBUG:
                    print("TCP: No speed found")
                return result
        elif command == "servo":
            match = re.search(r"servo\s*(on|off)", response, re.IGNORECASE)
            if match:
                result = match.group(1).upper()
                if DEBUG:
                    print(f"TCP: Servo status extracted: {result!r}")
                return result
            else:
                result = "ERROR: Servo status unknown"
                if DEBUG:
                    print("TCP: Servo status unknown")
                return result
        elif command == "coord":
            pattern = re.compile(
                r"mcserver>current mode:\s*(joint|cart|cyl|tool|user|cylinder)",
                re.IGNORECASE
            )
            match = pattern.search(response)
            if match:
                result = match.group(1).upper()
                if DEBUG:
                    print(f"TCP: Coord mode extracted: {result!r}")
                return result
            else:
                result = "ERROR: Coord mode unknown"
                if DEBUG:
                    print("TCP: Coord mode unknown")
                return result
        elif command == "mode":
            pattern = re.compile(r"mcserver>current mode:\s*(\w+)", re.IGNORECASE)
            match = pattern.search(response)
            if match:
                result = match.group(1).upper()
                if DEBUG:
                    print(f"TCP: Mode extracted: {result!r}")
                return result
            else:
                result = "ERROR: Mode unknown"
                if DEBUG:
                    print("TCP: Mode unknown")
                return result
        else:
            if DEBUG:
                print(f"TCP: Returning raw response for {command!r}")
            return response
    except Exception as e:
        log_message(f"Error sending/receiving {command}: {e}")
        if DEBUG:
            print(f"TCP: Exception: {e!r}")
        set_background_red()
        return "ERROR"
    finally:
        reset_background_color()

def sync_joint_values():
    angles = extract_joint_angles_alternative()
    if angles:
        for joint in joint_names:
            entries[joint].delete(0, tk.END)
            entries[joint].insert(0, f"{angles[joint]:.6f}")
        log_message("Synced joint values.")
    else:
        log_message("No joint data to sync.")

    # --- Send commands and update display ---
    mode = send_and_receive_tcp("mode")
    mode_value.set(mode)
    speed = send_and_receive_tcp("speed")
    speed_value.set(speed)
    coord = send_and_receive_tcp("coord")
    coord_value.set(coord)
    servo_status = send_and_receive_tcp("servo")
    servo_status_value.set(servo_status)

# --- New: Send Speed Command ---
def send_speed_command():
    """Sends the speed command with the value from the input box."""
    speed_to_send = speed_entry.get()
    if DEBUG:
        print(f"send_speed_command: Speed to send: {speed_to_send!r}")
    try:
        speed_value = int(speed_to_send)
        if DEBUG:
            print(f"send_speed_command: Speed value (int): {speed_value}")
        if 0 <= speed_value <= 10000:
            command_str = f"speed {speed_to_send}\r\n"
            if DEBUG:
                print(f"send_speed_command: Command string: {command_str!r}")
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if DEBUG:
                    print(f"send_speed_command: Connecting to {TCP_IP}:{TCP_PORT}")
                sock.connect((TCP_IP, TCP_PORT))
                sock.sendall(command_str.encode())
                if DEBUG:
                    print(f"send_speed_command: Sending data: {command_str.encode()!r}")
                sock.close()
                if DEBUG:
                    print("send_speed_command: Socket closed")
                log_message(f"Sent speed command: {command_str.strip()}")
                reset_background_color()
            except Exception as e:
                log_message(f"Error sending speed command: {e}")
                if DEBUG:
                    print(f"send_speed_command: Error sending: {e!r}")
                set_background_red()
        else:
            log_message("Speed value out of range (0-10000)")
            if DEBUG:
                print("send_speed_command: Speed out of range")
    except ValueError:
        log_message("Invalid speed value (must be an integer)")
        if DEBUG:
            print("send_speed_command: Invalid input")
        set_background_red()

move_button = tk.Button(button_frame, text="Move", command=send_move_command, font=("Helvetica", 14))
move_button.pack(side=tk.LEFT, padx=5)
stop_button = tk.Button(button_frame, text="Stop", command=send_stop_command, font=("Helvetica", 14))
stop_button.pack(side=tk.LEFT, padx=5)
sync_button = tk.Button(button_frame, text="Sync", command=sync_joint_values, font=("Helvetica", 14))
sync_button.pack(side=tk.LEFT, padx=5)

# --- New Speed Control ---
speed_entry = Entry(button_frame, width=10, font=("Helvetica", 12), state=tk.DISABLED)  # Initially disabled
speed_entry.pack(side=tk.LEFT, padx=5)
set_speed_button = Button(button_frame, text="Set Speed", command=send_speed_command, font=("Helvetica", 12), state=tk.DISABLED)  # Initially disabled
set_speed_button.pack(side=tk.LEFT, padx=5)

log_text = scrolledtext.ScrolledText(root, width=90, height=8, font=("Helvetica", 10))
log_text.pack(pady=10)
log_text.config(state=tk.DISABLED)

def log_message(msg):
    log_text.config(state=tk.NORMAL)
    timestamp = time.strftime("%H:%M:%S")
    log_text.insert(tk.END, f"[{timestamp}] {msg}\n")
    log_text.see(tk.END)
    log_text.config(state=tk.DISABLED)

initial_population_done = False
default_joint_values = {"S": 0.0, "L": 0.0, "U": 0.0, "R": 0.0, "B": 0.0, "T": 0.0, "J7": 0.0, "J8": 0.0}

def update_gui():
    global initial_population_done
    try:
        while not joint_angle_queue.empty():
            angles = joint_angle_queue.get_nowait()
            if angles:
                txt = "   ".join([f"{j}: {angles.get(j, default_joint_values[j]):.6f}" for j in joint_names])
                joint_label.config(text=txt)
                if not initial_population_done:
                    for joint in joint_names:
                        entries[joint].delete(0, tk.END)
                        entries[joint].insert(0, f"{angles.get(joint, default_joint_values[joint]):.6f}")
                    initial_population_done = True
        while not plc_state_queue.empty():
            plc_states = plc_state_queue.get_nowait()
            # Update PLC IN
            if plc_states['plc_in'] is not None:
                formatted_plc_in = add_spaces_to_bits(plc_states['plc_in'])
                plc_in_display_label.config(text=formatted_plc_in, fg="green" if '1' in plc_states['plc_in'] else "red")
            else:
                plc_in_display_label.config(text=add_spaces_to_bits(""), fg="grey")

            # Update PLC OUT
            if plc_states['plc_out'] is not None:
                formatted_plc_out = add_spaces_to_bits(plc_states['plc_out'])
                plc_out_display_label.config(text=formatted_plc_out, fg="green" if '1' in plc_states['plc_out'] else "red")
            else:
                plc_out_display_label.config(text=add_spaces_to_bits(""), fg="grey")
    except queue.Empty:
        pass
    root.after(10, update_gui)

def get_robot_credentials():
    """Reads robot credentials from a text file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script's directory
    credentials_path = os.path.join(script_dir, CREDENTIALS_FILE)  # Construct full path
    try:
        with open(credentials_path, "r") as f:
            lines = f.readlines()
            if len(lines) >= 2:
                username = lines[0].strip()
                password = lines[1].strip()
                return username, password
            else:
                raise ValueError("Credentials file is incomplete.")
    except FileNotFoundError:
        raise ValueError(f"Credentials file '{credentials_path}' not found.")
    except Exception as e:
        raise ValueError(f"Error reading credentials file: {e}")

for joint in joint_names:
    entries[joint].delete(0, tk.END)
    entries[joint].insert(0, f"{default_joint_values[joint]:.6f}")

ssh_thread = threading.Thread(target=ssh_stream_function, daemon=True)
ssh_thread.start()
root.after(10, update_gui)
root.mainloop()