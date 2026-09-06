import os
import socket

HOST = "0.0.0.0"          # listen on all local interfaces
PORT = 5000
OUTPUT_FILE = "scan_received.txt"

def main():
    output_path = os.path.abspath(OUTPUT_FILE)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Listening on port {PORT} ... waiting for ESP32-S3 to connect.")
    print(f"Points will be saved to: {output_path}")
    print("(No popup or notification will appear — watch this window for progress.)")

    conn, addr = server.accept()
    print(f"Connected by {addr}")

    buffer = ""
    point_count = 0

    with open(OUTPUT_FILE, "w") as f:
        while True:
            data = conn.recv(1024)
            if not data:
                print("Connection closed by ESP32.")
                break

            buffer += data.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                f.write(line + "\n")
                f.flush()
                point_count += 1
                if point_count % 50 == 0:
                    print(f"  {point_count} points received...")

    print(f"Done. {point_count} points saved to: {output_path}")

    try:
        os.startfile(os.path.dirname(output_path) or ".")
    except Exception:
        pass

    conn.close()
    server.close()

if __name__ == "__main__":
    main()
