"""ROS 2 topic relay between Isaac Sim on Windows and ROS 2 / Nav2 in WSL2 over one localhost TCP connection.

Why: DDS does not cross the Windows / WSL2 boundary reliably (mirrored networking: same IP on both sides, data never
arrives; NAT: Windows firewall + no multicast). A single TCP connection through WSL localhost forwarding does work, so
serialized messages are forwarded over it (same ROS distro, Jazzy, on both sides -> identical CDR bytes).

  WSL (start first, TCP server):    python3 ros_tcp_relay.py wsl [--port 47100]
  Windows (Isaac's bundled rclpy):  python ros_tcp_relay.py win [--port 47100]   (connects to 127.0.0.1)
Run each side in its own ROS_DOMAIN_ID (e.g. Windows 101, WSL 102) so the two DDS worlds never half-discover
each other.
"""
import argparse, importlib, socket, struct, threading, time, queue
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# topic, type, direction (w2l = Isaac -> Nav2, l2w = Nav2 -> Isaac), qos kind
TOPICS = [
    ("/clock", "rosgraph_msgs/msg/Clock", "w2l", "sensor"),
    ("/tf", "tf2_msgs/msg/TFMessage", "w2l", "reliable"),
    ("/tf_static", "tf2_msgs/msg/TFMessage", "w2l", "static"),
    ("/chassis/odom", "nav_msgs/msg/Odometry", "w2l", "reliable"),
    ("/chassis/imu", "sensor_msgs/msg/Imu", "w2l", "sensor"),
    ("/front_3d_lidar/lidar_points", "sensor_msgs/msg/PointCloud2", "w2l", "sensor"),
    ("/front_2d_lidar/scan", "sensor_msgs/msg/LaserScan", "w2l", "sensor"),
    ("/sim_task/status", "std_msgs/msg/String", "w2l", "reliable"),
    ("/sim_task/result", "std_msgs/msg/String", "w2l", "reliable"),
    ("/cmd_vel", "geometry_msgs/msg/Twist", "l2w", "reliable"),
    ("/sim_task/command", "std_msgs/msg/String", "l2w", "reliable"),
]


def qos(kind, publisher):
    if kind == "static":
        return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST)
    if kind == "sensor":   # subscribe best effort (matches any publisher); republish reliable (matches any subscriber)
        return QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE if publisher else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


def msg_class(type_name):
    pkg, _, name = type_name.split("/")
    return getattr(importlib.import_module(f"{pkg}.msg"), name)


def send_frame(sock, lock, idx, payload):
    with lock:
        sock.sendall(struct.pack("!HI", idx, len(payload)) + payload)


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=["win", "wsl"])
    ap.add_argument("--port", type=int, default=47100)
    a = ap.parse_args()

    if a.role == "wsl":                      # WSL is the TCP server: Windows reaches it via WSL localhost forwarding
        srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", a.port)); srv.listen(1)
        print(f"[relay:wsl] waiting for Windows on :{a.port}", flush=True)
        sock, _ = srv.accept()
    else:
        while True:
            try:
                sock = socket.create_connection(("127.0.0.1", a.port), timeout=5); break
            except OSError:
                time.sleep(1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1); sock.settimeout(None)
    print(f"[relay:{a.role}] connected", flush=True)

    rclpy.init()
    node = rclpy.create_node(f"tcp_relay_{a.role}")
    lock = threading.Lock()
    outgoing = "w2l" if a.role == "win" else "l2w"
    pubs, counts = {}, {t[0]: 0 for t in TOPICS}
    for idx, (topic, tname, direction, kind) in enumerate(TOPICS):
        cls = msg_class(tname)
        if direction == outgoing:
            node.create_subscription(cls, topic, lambda raw, i=idx, t=topic: (send_frame(sock, lock, i, raw),
                                     counts.__setitem__(t, counts[t] + 1)), qos(kind, False), raw=True)
        else:
            pubs[idx] = node.create_publisher(cls, topic, qos(kind, True))

    inbox = queue.Queue(maxsize=2000)
    def reader():
        while True:
            idx, n = struct.unpack("!HI", recv_exact(sock, 6))
            inbox.put((idx, recv_exact(sock, n)))
    threading.Thread(target=reader, daemon=True).start()

    last = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.002)
        try:
            while True:
                idx, payload = inbox.get_nowait()
                pubs[idx].publish(payload)            # rclpy publishes bytes as an already-serialized message
                counts[TOPICS[idx][0]] += 1
        except queue.Empty:
            pass
        if time.time() - last > 10:
            last = time.time()
            print(f"[relay:{a.role}] msgs " + ", ".join(f"{k}={v}" for k, v in counts.items() if v), flush=True)


if __name__ == "__main__":
    main()
