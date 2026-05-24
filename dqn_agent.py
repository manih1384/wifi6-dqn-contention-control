import socket

def start_ai_server():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 9999

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"[*] Python AI Agent listening on {UDP_IP}:{UDP_PORT}...")
    print("[*] Waiting for ns-3 simulation to start...")

    while True:
        # 1. Wait for ns-3 to send the current State
        data, addr = sock.recvfrom(1024) 
        state_str = data.decode('utf-8')
        
        print(f"\n[ns-3] State received: {state_str}")
        
        # --- AI LOGIC GOES HERE IN STEP 3 ---
        # The paper defines Actions: 0 (Decrease), 1 (Keep), 2 (Increase)
        # For testing the bridge, we will hardcode the action to "1" (Keep)
        action = "1" 
        
        # 2. Send the chosen Action back to ns-3
        sock.sendto(action.encode('utf-8'), addr)
        print(f"[AI] Action sent: {action}")

if __name__ == "__main__":
    start_ai_server()