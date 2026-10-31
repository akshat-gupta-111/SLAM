import socket
import json
import csv
import sys

UDP_IP = "127.0.0.1"
UDP_PORT = 8001
MAX_SAMPLES = 15000

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    csv_filename = "rf2o_raw_data.csv"
    
    print(f"RF2O Data Collector Started.")
    print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")
    print(f"Target: Automatically stopping after {MAX_SAMPLES} samples.\n")
    
    sample_count = 0
    
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp_ns", "vx", "vy", "omega"])
        
        try:
            while sample_count < MAX_SAMPLES:
                data, addr = sock.recvfrom(1024) 
                try:
                    payload = json.loads(data.decode('utf-8'))
                    t_ns = payload.get("timestamp_ns", 0)
                    vx = payload.get("vx", 0.0)
                    vy = payload.get("vy", 0.0)
                    omega = payload.get("omega", 0.0)
                    
                    writer.writerow([t_ns, vx, vy, omega])
                    sample_count += 1
                    
                    # Print every 100th sample to keep terminal clean but show progress
                    if sample_count % 100 == 0:
                        print(f"Progress: {sample_count}/{MAX_SAMPLES} | Time: {t_ns} | vx: {vx: .4f} | vy: {vy: .4f} | omega: {omega: .4f}")
                        
                except json.JSONDecodeError:
                    pass # Ignore bad JSON quietly during fast collection
                    
            print(f"\n✅ Target reached! Successfully saved {MAX_SAMPLES} samples to {csv_filename}")
            
        except KeyboardInterrupt:
            print(f"\nData collection stopped early by user.")
            print(f"Saved {sample_count} samples to {csv_filename}")

if __name__ == '__main__':
    main()
