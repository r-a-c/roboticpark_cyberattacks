import socket
import threading
import concurrent.futures


def scan_tcp_ports(ip, ports):
    """Searchs for TCP open ports against a machine

       Parameters:
       ip: String, The ip to attack to.
       ports: List, A list of ports

       Returns: 
       List, a list of open ports.
    """
    open_ports_tcp = []
    for port in ports:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1)
            result = s.connect_ex((ip, port))
            if result == 0:
                open_ports_tcp.append(port)
    return open_ports_tcp

def scan_udp_ports(ip, ports):
    """Searchs for UDP open ports against a machine. This functions organizes the work.

       Parameters:
       ip: String, The ip to attack to.
       ports: List, A list of ports

       Returns: 
       List, a list of open ports.
    """
    open_ports_udp = []
    futures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=1000) as executor:
        
        for port in ports:
            futures.append(executor.submit(scan_udp_port,ip=ip, port=port))
            
        for future in concurrent.futures.as_completed(futures):
            if future.result():
                open_ports_udp.append(future.result())

    return open_ports_udp

def scan_udp_port(ip, port):
    """Searchs for UDP open ports against a machine. This functions makes the actual work.
       
       Parameters:
       ip: String, The ip to attack to.
       port: String, the actual port scanned.

       Returns: 
       None if no response, else the port String.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(10.0)
        try:
            s.sendto(b'', (ip, port))
            s.recvfrom(1024)
            return port
        except socket.timeout:
            print(f"Timeout  {port}")
        except socket.error as e:
            print(f"Socket error  {port}: {e}")
    return None