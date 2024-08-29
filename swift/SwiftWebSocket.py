#!/usr/bin/env python
"""A Socket Handling Class for Swift
@author Jesse Haviland (original)
@author Dasun Gunasing (modifications)
"""

import swift
import asyncio
import websockets
from threading import Thread
import json
import http.server
import socketserver
import os
from queue import Queue, Empty
from http import HTTPStatus
from typing import Union
import urllib
from pathlib import Path

class SwiftSocket:
    def __init__(self, outq, inq, run, connected):
        # Set class variables/methods
        self.pcs = set()
        self.run = run
        self.connected = connected
        self.outq = outq
        self.inq = inq
        self.USERS = set()
        self.loop = asyncio.new_event_loop()
        self.name = "SWIFT SOCKET"
        asyncio.set_event_loop(self.loop)

        # Attempt websocket serve to available port within range
        started = False
        port = 53000
        while not started and port < 62000:
            try:
                print(f"{self.name}: Starting Socket...")
                start_server = websockets.serve(self.serve, "localhost", port)
                print(f"{self.name}: Started Socket!")
                self.loop.run_until_complete(start_server)
                started = True
            except OSError:
                port += 1

        # Output the connected port via queue on completion
        self.inq.put(port)
        self.loop.run_forever()

    async def serve(self, websocket, path):
        """The main serve method running on server side
        - Awaits connection and registers client socket in set
        - Runs bi-directional comms until higher-level class updates run method

        :param websocket: _description_
        :type websocket: _type_
        :param path: _description_
        :type path: _type_
        """
        print(f"{self.name}: Socket Initialising...")
        # Initial connection handshake to available client
        # Store client as a set (non-duplicates)
        await self.register(websocket)
        recieved = await websocket.recv()
        # Send back connected status on successfull connection to client (static page)
        self.inq.put(recieved)
        
        # Now onto send, recieve cycle
        print(f"{self.name}: Socket Running...")
        while self.run():
            # Get data from Swift through producer method
            message = await self.producer()
            expected = message[0]
            msg = message[1]
            # Send data to client
            await websocket.send(json.dumps(msg))
            # Receive any responses (based on expected or not)
            await self.expect_message(websocket, expected)

        # Termination requested (updated through self.run())
        print(f"{self.name}: Socket Terminating...")
        # Add any other cleanup activites (if required)
        self.connected = False
        print(f"{self.name}: Socket Terminated")
        return

    # --- Data in and out methods
    async def expect_message(self, websocket, expected):
        """Main input method from the client

        :param websocket: _description_
        :type websocket: _type_
        :param expected: _description_
        :type expected: _type_
        """
        # Update swift with any received data that was expected
        if expected:
            recieved = await websocket.recv()
            self.inq.put(recieved)

    async def producer(self):
        """Producer method that gets data from queue (as provided by Swift)

        :return: _description_
        :rtype: _type_
        """
        # Get data from Swift if available
        data = self.outq.get()
        return data
    
    # --- Additional websocket methods
    async def register(self, websocket):
        """Registration method (connection status and websocket handler)

        :param websocket: _description_
        :type websocket: _type_
        """
        # Set the connected member variable in Swift for monitoring connection to the socket
        self.connected(True)
        # Update the tracking of client users 
        self.USERS.add(websocket)

class SwiftServer:
    """Main Swift Page Serving Class
    """
    def __init__(self, outq: Queue, inq: Queue, socket_port: int, run, verbose: bool = False) -> None:
        server_port = 52000

        # Get the root directory of the built static page from the swift package folder
        root_dir = Path(swift.__file__).parent / "out"

        class MyHttpRequestHandler(http.server.SimpleHTTPRequestHandler):
            """HTTP Request Handler Class

            :param http: _description_
            :type http: _type_
            """
            def __init__(self, *args, **kwargs):
                super(MyHttpRequestHandler, self).__init__(
                    directory=str(root_dir), *args, **kwargs
                )

            def log_message(self, format, *args):
                if verbose:
                    http.server.SimpleHTTPRequestHandler.log_message(
                        self, format, *args
                    )
                else:
                    pass

            def do_POST(self):
                # Handle RTC Offer
                if self.path == "/offer":
                    # Get the initial offer
                    length = int(self.headers.get("content-length"))  # type: ignore
                    params = json.loads(self.rfile.read(length))

                    inq.put(params)
                    answer = outq.get()

                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(str.encode(answer))

                    return

            def do_GET(self):
                if self.path == "/":
                    self.send_response(301)

                    self.send_header(
                        "Location",
                        "http://localhost:"
                        + str(server_port)
                        + "/?"
                        + str(socket_port),
                    )

                    self.end_headers()
                    return
                elif self.path == "/?" + str(socket_port):
                    self.path = "index.html"
                elif self.path.startswith("/retrieve/"):
                    # print(f"Retrieving file: {self.path[10:]}")
                    self.path = urllib.parse.unquote(self.path[9:])
                    self.send_file_via_real_path()
                    return

                self.path = Path(self.path).as_posix()

                try:
                    http.server.SimpleHTTPRequestHandler.do_GET(self)
                except BrokenPipeError:
                    # After killing this error will pop up but it's of no use
                    # to the user
                    pass

            def send_file_via_real_path(self):
                try:
                    f = open(self.path, "rb")
                except OSError:
                    self.send_error(HTTPStatus.NOT_FOUND, "File not found")
                    return None
                ctype = self.guess_type(self.path)
                try:
                    fs = os.fstat(f.fileno())
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-type", ctype)
                    self.send_header("Content-Length", str(fs[6]))
                    self.send_header(
                        "Last-Modified", self.date_time_string(fs.st_mtime)
                    )
                    self.end_headers()
                    self.copyfile(f, self.wfile)
                finally:
                    f.close()

        # Instantiate a HTTP handler
        self.handler = MyHttpRequestHandler
        self.connected = False
        while not self.connected and server_port < 62000:
            try:
                with socketserver.TCPServer(("", server_port), self.handler) as httpd:
                    inq.put(server_port)
                    self.connected = True

                    httpd.serve_forever()
            except OSError:
                server_port += 1

# -- Modified original start_servers functionality with the following changes:
# Removal of RTC (temporarily) 
# Addition of a socket connection checking thread
def start_servers(outq: Queue, inq: Queue, stop_servers, socket_status, socket_manager):
    # Start the websocket handler with a new port
    # NOTE: stop_servers method used to terminate socket via run method internally
    # NOTE: connected_check method used to update connection status internally
    socket = Thread(
        target=SwiftSocket,
        args=(
            outq,
            inq,
            stop_servers,
            socket_status,
        ),
        daemon=True,
    )
    # Name for identification
    socket.name = "Thread-Socket-Server" 
    # Start the socket thread
    socket.start()
    # Get the port (first output from socket initialisation)
    socket_port = inq.get()

    # Start a http server
    # This is the static page that acts as a 'client' to the websocket
    server = Thread(
        target=SwiftServer,
        args=(
            outq,
            inq,
            socket_port,
            stop_servers,
        ),
        daemon=True,
    )
    # Name for identification
    server.name = "Thread-Page-Client" 
    server.start()
    server_port = inq.get()
    print(f"SOCKET SETUP: Available at => http://localhost:{server_port}/?{socket_port}") 

    # Start a connection checking thread (for headless implementation)
    manager = Thread(target=socket_manager, daemon=True)
    # Name for identification
    manager.name = "Thread-Socket-Manager" 
    manager.start()

    # Return thread handlers to higher-level caller for thread termination
    return socket, server, manager
