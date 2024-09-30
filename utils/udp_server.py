#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

address = '0.0.0.0'
port = 650
server = (address, port)
sock.bind(server)

print(f"Server {address}:{port}")

while True:
	payload, caddress = sock.recvfrom(1)
	print(f"Returning data back to {caddress}")
	sent = sock.sendto(payload, caddress)


